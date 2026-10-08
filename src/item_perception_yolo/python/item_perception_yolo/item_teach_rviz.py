"""Read-only, bounded Item Teach point-cloud and candidate visualization."""

import copy
import json
import threading
import time

import numpy as np
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray

from .depth_snapshot import DEPTH_ENCODING, median_depth_snapshot
from .item_detector import validate_pair, validate_candidates, validate_candidate_selection
from .pose_guides import pose_guide_markers


PERIOD_SEC = 1.0
STALE_AFTER_SEC = 5.0
CLOUD_TOPIC = "/item_teach/voxel_cloud"
MARKER_TOPIC = "/item_teach/valid_items"
DIAGNOSTIC_TOPIC = "/item_teach/rviz_diagnostics"


def cloud_message(data, count, stamp):
    cloud = PointCloud2()
    cloud.header.frame_id, cloud.header.stamp = "base_link", stamp
    cloud.height, cloud.width = 1, count
    fields = (("x", 0, PointField.FLOAT32), ("y", 4, PointField.FLOAT32),
              ("z", 8, PointField.FLOAT32), ("rgb", 12, PointField.UINT32))
    cloud.fields = [PointField(name=name, offset=offset, datatype=kind, count=1)
                    for name, offset, kind in fields]
    cloud.is_bigendian, cloud.is_dense = False, True
    cloud.point_step, cloud.row_step, cloud.data = 16, count * 16, data
    return cloud


class TeachingRvizPreview:
    def __init__(self, node, transform_builder):
        self.node, self.transform_builder = node, transform_builder
        self.lock = threading.RLock()
        self.current = None
        self.displayed = None
        self.displayed_at = None
        self.displayed_grey = False
        self.waiting_reason = ""
        self.next_publish = 0.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_publisher = node.create_publisher(PointCloud2, CLOUD_TOPIC, qos)
        self.marker_publisher = node.create_publisher(MarkerArray, MARKER_TOPIC, 1)
        self.diagnostic_publisher = node.create_publisher(String, DIAGNOSTIC_TOPIC, 1)
        node.create_timer(.2, self.tick)

    def compute_capture(self, rgb, depth, context, timeout):
        """Reuse a pose request's owned worker and exact frames for cloud geometry only."""
        node = self.node
        view = {"observation": {"rgb": rgb, "depth": depth, "context": context,
                                "epoch": node.arm_epoch,
                                "camera_generation": node._camera_generation, "error": ""}}
        options = {"quality": copy.deepcopy(node.settings["quality"]), "settings": None,
                   "candidate_limit": None, "planning": None, "pose_error": ""}
        result = self.compute(view, options, request_owned=True, timeout=timeout)
        return {**result, "capture": True}

    def compute(self, view, options, *, request_owned=False, timeout=None):
        """Use the existing worker, owned by the GUI job or the current pose request."""
        node = self.node
        if view is None:
            return None
        if options.get("error"):
            return {"error": options["error"]}
        if request_owned:
            if (not node.request_lock.locked() or not node.operation_lock.locked()
                    or options["settings"] is not None or "observation" not in view):
                raise RuntimeError(
                    "Capture voxels require an owned request and cloud-only snapshot")
        elif (node.background_suspended() or node.request_lock.locked()
              or not node.operation_lock.acquire(blocking=False)):
            return {"error": "Pose request has priority; RViz preview waiting"}
        try:
            node._validate_sources()
            epoch, generation = node.arm_epoch, node._camera_generation
            applied = node.applied
            if applied is None:
                raise ValueError("Load station calibration for RViz")
            observation = view.get("observation")
            if observation is not None:
                if (observation["epoch"] != epoch
                        or observation["camera_generation"] != generation):
                    raise ValueError("RViz observation was invalidated")
                rgb, depth, context = (observation[key] for key in ("rgb", "depth", "context"))
                if depth is None or context is None:
                    raise ValueError(observation["error"] or "RViz requires calibrated RGB/depth")
                context = copy.deepcopy(context)
            else:
                rgb = view["source_rgb"]
                context = node._measurement_context(
                    rgb, options["quality"]["robot_tf_max_age_sec"])
                with node.condition:
                    rgb = node._bundle_rgb(rgb, options["depth_frame_count"])
                    frames, color_info, depth_info = node._depth_window(
                        rgb, options["quality"], options["depth_frame_count"])
                context = node._snapshot_context(
                    rgb, frames, color_info, depth_info, options["quality"])
                depth = median_depth_snapshot(frames, options["quality"])
            context["pick_planning"] = options["planning"]
            overlay_requested = bool(options["settings"] is not None
                                     and view.get("preview_mode") == "all"
                                     and view.get("depth_rgb"))
            header = {"operation": "teaching_rviz", "generation": epoch,
                      "passive_overlay": overlay_requested,
                      "nearby_overlay": overlay_requested, "depth_encoding": DEPTH_ENCODING,
                      "camera_generation": generation,
                      "width": rgb["width"], "height": rgb["height"], "context": context,
                      "base_from_platform": applied.platform.base_from_platform.tolist(),
                      "quality": options["quality"], "settings": options["settings"],
                      "candidate_limit": options["candidate_limit"],
                      "detections": view.get("metadata", {}).get("detections", [])}
            payload = rgb["rgb"] + depth["depth"]
            if overlay_requested:
                payload += view["rgb"] + view["depth_rgb"]
            result, data = node.native.call(
                header, payload,
                options["quality"]["request_timeout_sec"] if timeout is None else timeout)
            try:
                if (set(result) != {"state", "generation", "point_count", "candidates", "rejected",
                                    "nearby_overlay", "unchecked"}
                        or result["state"] != "ok" or result["generation"] != epoch
                        or result["nearby_overlay"] is not overlay_requested
                        or type(result["point_count"]) is not int
                        or not 0 <= result["point_count"] <= rgb["width"] * rgb["height"]
                        or len(data) != result["point_count"] * 16 + (
                            len(rgb["rgb"]) * 2 if overlay_requested else 0)
                        or type(result["candidates"]) is not list
                        or type(result["rejected"]) is not list
                        or type(result["unchecked"]) is not list):
                    raise RuntimeError("Malformed RViz preview response")
                cloud_end = result["point_count"] * 16
                points = np.frombuffer(data[:cloud_end], "<f4").reshape(-1, 4)
                if not np.isfinite(points[:, :3]).all():
                    raise RuntimeError("Non-finite RViz point cloud")
                if options["settings"] is None:
                    if result["candidates"] or result["rejected"] or result["unchecked"]:
                        raise RuntimeError("Unexpected RViz poses without valid settings")
                else:
                    validate_candidates(result, options["settings"])
                    ids = {entry["source_index"] for entry in header["detections"]}
                    validate_candidate_selection(result, options["candidate_limit"], source_ids=ids)
            except (ValueError, KeyError, TypeError, RuntimeError) as exc:
                node.native.failed = True
                node.native.close()
                raise RuntimeError(f"Invalid native RViz result: {exc}") from exc
            node._validate_sources()
            if epoch != node.arm_epoch or generation != node._camera_generation:
                raise ValueError("RViz preview invalidated during processing")
            if overlay_requested:
                view["rgb"] = data[cloud_end:cloud_end+len(rgb["rgb"])]
                view["depth_rgb"] = data[cloud_end+len(rgb["rgb"]):]
                view["passive_overlay"] = True
            return {**result, "data": data[:cloud_end], "epoch": epoch,
                    "camera_generation": generation,
                    "stamp_ns": rgb["stamp_ns"], "depth_stamp_ns": depth["stamp_ns"],
                    "base_from_platform": header["base_from_platform"],
                    "camera": context["camera"], "depth_camera": context["depth_camera"],
                    "quality": options["quality"], "pose_error": options["pose_error"]}
        except (ValueError, OSError, TransformException) as exc:
            return {"error": str(exc)}
        finally:
            if not request_owned:
                node.operation_lock.release()

    def publish(self, snapshot):
        with self.lock:
            if snapshot is None or "error" in snapshot:
                self.hold("No RViz snapshot" if snapshot is None else snapshot["error"])
                return
            if self.node.background_suspended() and not snapshot.get("capture"):
                self.hold("Production active; waiting for next pose capture")
                return
            if snapshot["point_count"] == 0:
                self.hold("No valid depth voxels; waiting for next frame")
                return
            if self.displayed is not None and all(
                    snapshot[key] == self.displayed[key] for key in ("epoch", "camera_generation")):
                if (any(snapshot[key] < self.displayed[key]
                        for key in ("stamp_ns", "depth_stamp_ns"))
                        or (self.displayed.get("capture") and not snapshot.get("capture")
                            and snapshot["stamp_ns"] == self.displayed["stamp_ns"]
                            and snapshot["depth_stamp_ns"] == self.displayed["depth_stamp_ns"])):
                    return  # A late background job cannot replace a newer request capture.
            self.current = snapshot
            self.waiting_reason = ""
            if snapshot.get("capture"):
                self.next_publish = 0.  # Every completed request, even within the idle 1 Hz slot.
            self.tick()

    def hold(self, reason):
        """Keep the last cloud during acquisition gaps; suspend candidate TF/markers."""
        with self.lock:
            if self.current is None:
                return
            if not self.waiting_reason:
                self.marker_publisher.publish(
                    MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
            # A completed result may still await the 1 Hz publication slot.
            # Retention must describe the cloud RViz actually received.
            self.current = self.displayed
            self.waiting_reason = reason

    def clear(self, reason="Teaching preview cleared"):
        """Invalidate poses immediately, retaining an explicitly untrusted grey cloud."""
        with self.lock:
            self.current = None
            self._grey_cloud(reason)
            self._retained_status()

    def _grey_cloud(self, reason):
        self.waiting_reason = reason
        if self.displayed is None or self.displayed_grey:
            return
        sample = self.displayed
        data = bytearray(sample["data"])
        np.frombuffer(data, "<u4").reshape(-1, 4)[:, 3] = 0x808080
        self.cloud_publisher.publish(cloud_message(
            bytes(data), sample["point_count"], Time(nanoseconds=sample["stamp_ns"]).to_msg()))
        self.displayed_grey = True
        self.marker_publisher.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def _retained_status(self):
        sample = self.displayed
        if sample is None:
            return
        age = max(0., (self.node.get_clock().now().nanoseconds - sample["stamp_ns"]) / 1e9)
        self.diagnostic_publisher.publish(String(data=json.dumps({
            "status": "stale_grey" if self.displayed_grey else "retained_cloud",
            "source": "pose_capture" if sample.get("capture") else "background_preview",
            "reason": self.waiting_reason, "source_stamp_ns": sample["stamp_ns"],
            "depth_stamp_ns": sample["depth_stamp_ns"], "age_sec": age,
            "refresh_age_sec": max(0., time.monotonic() - self.displayed_at),
            "voxel_size_mm": 10, "point_count": sample["point_count"],
            "candidates": [], "rejected": [], "unchecked": [],
            "pose_error": self.waiting_reason})))

    def tick(self):
        """Keep the cached cloud, greying it when refresh stops or its sources invalidate."""
        with self.lock:
            sample, node = self.current, self.node
            if sample is None:
                if time.monotonic() >= self.next_publish:
                    self.next_publish = time.monotonic() + PERIOD_SEC
                    self._retained_status()
                return
            if (sample["epoch"] != node.arm_epoch
                    or sample["camera_generation"] != node._camera_generation
                    or node.native.failed or node.fatal_error):
                self.clear("RViz preview invalidated")
                return
            if time.monotonic() < self.next_publish:
                return
            self.next_publish = time.monotonic() + PERIOD_SEC
            try:
                node._validate_sources()
                now = node.get_clock().now()
                with node.condition:
                    if ((node._color_info is not None and sample["camera"] != node._color_info)
                            or (node._depth_info is not None
                                and sample["depth_camera"] != node._depth_info)):
                        raise ValueError("CameraInfo changed since RViz snapshot")
            except (ValueError, OSError) as exc:
                self.clear(str(exc))
                return
            if not self.waiting_reason and not sample.get("capture"):
                try:
                    with node.condition:
                        validate_pair(node._image, node._depth, node._color_info, node._depth_info,
                                      now.nanoseconds, sample["quality"])
                except ValueError as exc:
                    self.hold(str(exc))
                    sample = self.current
                    if sample is None:
                        return
            source_stamp = Time(nanoseconds=sample["stamp_ns"]).to_msg()
            age = max(0., (now.nanoseconds - sample["stamp_ns"]) / 1e9)
            new_cloud = self.displayed is None or any(
                sample[key] != self.displayed[key]
                for key in ("epoch", "camera_generation", "stamp_ns", "depth_stamp_ns"))
            if not new_cloud and (self.displayed_grey
                                  or time.monotonic() - self.displayed_at >= STALE_AFTER_SEC):
                self._grey_cloud(self.waiting_reason or "No refreshed voxel data for 5 seconds")
                self._retained_status()
                return
            if self.waiting_reason:
                self._retained_status()
                return
            markers = [Marker(action=Marker.DELETEALL)]
            transforms = []
            for rank, candidate in enumerate(sample["candidates"], 1):
                frame = self.transform_builder(
                    sample["base_from_platform"], candidate, now.to_msg(),
                    child_frame_id=f"item_teach_live_candidate_{rank}")
                transforms.append(frame)
                markers.extend(pose_guide_markers(frame, "item_axes", rank))
            if transforms:
                node.selected_pose_broadcaster.sendTransform(transforms)
            # The reliable depth-one cache supplies this cloud to late-joining RViz viewers.
            if new_cloud:
                self.cloud_publisher.publish(
                    cloud_message(sample["data"], sample["point_count"], source_stamp))
                self.displayed = sample
                self.displayed_at = time.monotonic()
                self.displayed_grey = False
            self.marker_publisher.publish(MarkerArray(markers=markers))
            self.diagnostic_publisher.publish(String(data=json.dumps({
                "status": "snapshot", "source_stamp_ns": sample["stamp_ns"],
                "source": "pose_capture" if sample.get("capture") else "background_preview",
                "depth_stamp_ns": sample["depth_stamp_ns"], "age_sec": age,
                "refresh_age_sec": max(0., time.monotonic() - self.displayed_at),
                "voxel_size_mm": 10, "point_count": sample["point_count"],
                "blue_guide": "upward_surface_normal_not_pose_z",
                "candidates": sample["candidates"], "rejected": sample["rejected"],
                "unchecked": sample["unchecked"],
                "pose_error": sample["pose_error"]}, allow_nan=False)))
