"""Read-only, bounded Item Teach point-cloud and candidate visualization."""

import copy
import json
import threading
import time

import numpy as np
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray

from .item_detector import validate_pair, validate_candidates


PERIOD_SEC = 1.0
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
        self.waiting_reason = ""
        self.next_publish = 0.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cloud_publisher = node.create_publisher(PointCloud2, CLOUD_TOPIC, qos)
        self.marker_publisher = node.create_publisher(MarkerArray, MARKER_TOPIC, 1)
        self.diagnostic_publisher = node.create_publisher(String, DIAGNOSTIC_TOPIC, 1)
        node.create_timer(.2, self.tick)

    def compute(self, view, options):
        """Run in the GUI's existing single job slot, with no additional YOLO prediction."""
        node = self.node
        if view is None:
            return None
        if options.get("error"):
            return {"error": options["error"]}
        if node.request_lock.locked() or not node.operation_lock.acquire(blocking=False):
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
                    validate_pair(rgb, node._depth, node._color_info, node._depth_info,
                                  node.get_clock().now().nanoseconds, options["quality"])
                    if context["camera"] != node._color_info:
                        raise ValueError("Color CameraInfo changed during RViz capture")
                    depth = dict(node._depth)
                    context["depth_camera"] = copy.deepcopy(node._depth_info)
            context["pick_planning"] = options["planning"]
            header = {"operation": "teaching_rviz", "generation": epoch,
                      "width": rgb["width"], "height": rgb["height"], "context": context,
                      "base_from_platform": applied.platform.base_from_platform.tolist(),
                      "quality": options["quality"], "settings": options["settings"],
                      "detections": view.get("metadata", {}).get("detections", [])}
            result, data = node.native.call(
                header, rgb["rgb"] + depth["depth"], options["quality"]["request_timeout_sec"])
            try:
                if (set(result) != {"state", "generation", "point_count", "candidates", "rejected"}
                        or result["state"] != "ok" or result["generation"] != epoch
                        or type(result["point_count"]) is not int
                        or not 0 <= result["point_count"] <= rgb["width"] * rgb["height"]
                        or len(data) != result["point_count"] * 16
                        or type(result["candidates"]) is not list
                        or type(result["rejected"]) is not list):
                    raise RuntimeError("Malformed RViz preview response")
                points = np.frombuffer(data, "<f4").reshape(-1, 4)
                if not np.isfinite(points[:, :3]).all():
                    raise RuntimeError("Non-finite RViz point cloud")
                if options["settings"] is None:
                    if result["candidates"] or result["rejected"]:
                        raise RuntimeError("Unexpected RViz poses without valid settings")
                else:
                    validate_candidates(result, options["settings"])
                    ids = {entry["source_index"] for entry in header["detections"]}
                    actual = [entry["source_index"]
                              for entry in result["candidates"] + result["rejected"]]
                    if (len(actual) != len(ids) or set(actual) != ids
                            or any(type(entry["reason"]) is not str or not entry["reason"]
                                   for entry in result["rejected"])):
                        raise RuntimeError("RViz candidate/rejection identities changed")
            except (ValueError, KeyError, TypeError, RuntimeError) as exc:
                node.native.failed = True
                node.native.close()
                raise RuntimeError(f"Invalid native RViz result: {exc}") from exc
            node._validate_sources()
            if epoch != node.arm_epoch or generation != node._camera_generation:
                raise ValueError("RViz preview invalidated during processing")
            return {**result, "data": data, "epoch": epoch, "camera_generation": generation,
                    "stamp_ns": rgb["stamp_ns"], "depth_stamp_ns": depth["stamp_ns"],
                    "base_from_platform": header["base_from_platform"],
                    "camera": context["camera"], "depth_camera": context["depth_camera"],
                    "quality": options["quality"], "pose_error": options["pose_error"]}
        except (ValueError, OSError, TransformException) as exc:
            return {"error": str(exc)}
        finally:
            node.operation_lock.release()

    def publish(self, snapshot):
        with self.lock:
            if snapshot is None or "error" in snapshot:
                self.hold("No RViz snapshot" if snapshot is None else snapshot["error"])
                return
            if snapshot["point_count"] == 0:
                self.hold("No valid depth voxels; waiting for next frame")
                return
            self.current = snapshot
            self.waiting_reason = ""
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
        with self.lock:
            if self.current is None:
                return
            self.current = None
            self.displayed = None
            self.waiting_reason = ""
            stamp = self.node.get_clock().now().to_msg()
            self.cloud_publisher.publish(cloud_message(b"", 0, stamp))
            self.marker_publisher.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
            self.diagnostic_publisher.publish(String(data=json.dumps({"status": reason})))

    def tick(self):
        """Retain clouds between valid snapshots; clear only invalidated source state."""
        with self.lock:
            sample, node = self.current, self.node
            if sample is None:
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
            if not self.waiting_reason:
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
            if self.waiting_reason:
                self.diagnostic_publisher.publish(String(data=json.dumps({
                    "status": "retained_cloud", "reason": self.waiting_reason,
                    "source_stamp_ns": sample["stamp_ns"],
                    "depth_stamp_ns": sample["depth_stamp_ns"], "age_sec": age,
                    "voxel_size_mm": 10, "point_count": sample["point_count"],
                    "candidates": [], "rejected": [], "pose_error": self.waiting_reason})))
                return
            cloud = cloud_message(sample["data"], sample["point_count"], source_stamp)
            markers = [Marker(action=Marker.DELETEALL)]
            transforms = []
            for rank, candidate in enumerate(sample["candidates"], 1):
                frame = self.transform_builder(
                    sample["base_from_platform"], candidate, now.to_msg(),
                    child_frame_id=f"item_teach_live_candidate_{rank}")
                transforms.append(frame)
                for axis in range(3):
                    marker = Marker()
                    marker.header = frame.header
                    marker.ns, marker.id = "item_axes", rank * 4 + axis
                    marker.type, marker.action = Marker.ARROW, Marker.ADD
                    marker.pose.position = Point(x=frame.transform.translation.x,
                                                 y=frame.transform.translation.y,
                                                 z=frame.transform.translation.z)
                    marker.pose.orientation = frame.transform.rotation
                    endpoint = [0., 0., 0.]
                    endpoint[axis] = .04
                    marker.points = [Point(), Point(x=endpoint[0], y=endpoint[1], z=endpoint[2])]
                    marker.scale.x, marker.scale.y, marker.scale.z = .002, .004, .008
                    marker.color.a = 1.
                    setattr(marker.color, ("r", "g", "b")[axis], 1.)
                    marker.lifetime = Duration(seconds=2.5).to_msg()
                    markers.append(marker)
                label = Marker()
                label.header, label.ns, label.id = frame.header, "item_labels", rank
                label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
                label.pose.position = Point(x=frame.transform.translation.x,
                                            y=frame.transform.translation.y,
                                            z=frame.transform.translation.z + .05)
                label.pose.orientation.w, label.scale.z = 1., .018
                label.color.r = label.color.g = label.color.b = label.color.a = 1.
                label.text = (f"P{rank} {candidate['class_name']} {candidate['confidence']:.2f}"
                              f" | snapshot {age:.1f}s")
                label.lifetime = Duration(seconds=2.5).to_msg()
                markers.append(label)
            if transforms:
                node.selected_pose_broadcaster.sendTransform(transforms)
            self.cloud_publisher.publish(cloud)
            self.displayed = sample
            self.marker_publisher.publish(MarkerArray(markers=markers))
            self.diagnostic_publisher.publish(String(data=json.dumps({
                "status": "snapshot", "source_stamp_ns": sample["stamp_ns"],
                "depth_stamp_ns": sample["depth_stamp_ns"], "age_sec": age,
                "voxel_size_mm": 10, "point_count": sample["point_count"],
                "candidates": sample["candidates"], "rejected": sample["rejected"],
                "pose_error": sample["pose_error"]}, allow_nan=False)))
