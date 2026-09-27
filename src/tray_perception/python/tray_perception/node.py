"""Read-only ROS camera adapter for tray teaching; no hardware command clients."""

import copy
from pathlib import Path
import threading
import time

import numpy as np
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from camera_calibration_gui.calibration_core import workspace_root
from item_perception_yolo.item_detector import (
    stamp_ns, transform_matrix, validate_camera_info, validate_pair)
from item_perception_yolo.item_native_client import NativeClient
from item_perception_yolo.item_preview import frame_from_message, validate_prefix
from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS, file_sha256
from item_perception_yolo.platform_teach_core import (
    load_camera_calibration, load_robot_lan1_ip, resolve_base_from_camera_link)

from .core import EventLogger, load_profile, validate_plane, validate_preview
from .rviz import TrayRvizPreview


class TrayTeachNode(Node):
    def __init__(self):
        super().__init__("tray_teach")
        self.root = workspace_root()
        load_robot_lan1_ip(self.root)
        self.events = EventLogger(self.root)
        self.native = NativeClient(self.events, worker_package="tray_perception",
                                   worker_executable="tray_worker")
        self.lock = threading.RLock()
        self.generation = 0
        self.connection = 0
        self.camera_prefix = None
        self.camera = self.model = self.model_metadata = self.position = self.plane = None
        self.rgb = self.depth = self.color_info = self.depth_info = None
        self.subscriptions_owned = []
        self.selected = None
        self._published_key = None
        self.fatal_error = ""
        self.camera_status = "Enter a camera prefix and Connect RGB"
        self.tf_buffer = Buffer(cache_time=Duration(seconds=15))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.broadcaster = TransformBroadcaster(self)
        self.rviz = TrayRvizPreview(self)
        self.create_timer(.2, self._tick)
        self.events.record("INFO", "started", "Tray Teach started; read-only, no command clients")

    def invalidate(self, reason="Tray settings or sources changed"):
        with self.lock:
            self.generation += 1
            self.selected = None
            self._published_key = None
        self.rviz.invalidate(reason)

    def apply_camera(self, path):
        camera = load_camera_calibration(Path(path), root=self.root)
        with self.lock:
            self.invalidate()
            self.camera, self.plane = camera, None
            generation = self.generation
        self.connect_camera(camera.settings.camera_prefix)
        self.events.record("INFO", "camera_loaded", str(camera.path), generation=generation)
        return camera

    def connect_camera(self, prefix):
        validate_prefix(prefix)
        with self.lock:
            self.invalidate("Camera reconnected")
            if self.camera_prefix != prefix:
                self.plane = None
            self.camera_prefix = prefix
            self.connection += 1
            connection = self.connection
            self.rgb = self.depth = self.color_info = self.depth_info = None
        for subscription in self.subscriptions_owned:
            self.destroy_subscription(subscription)
        self.subscriptions_owned = []
        for kind, topic, message_type in (("rgb", "color/image_raw", Image),
                                          ("depth", "depth/image_raw", Image),
                                          ("color_info", "color/camera_info", CameraInfo),
                                          ("depth_info", "depth/camera_info", CameraInfo)):
            self.subscriptions_owned.append(self.create_subscription(
                message_type, f"/{prefix}/{topic}",
                lambda msg, k=kind, p=prefix, c=connection: self._receive(msg, k, p, c),
                qos_profile_sensor_data))
        self.camera_status = f"Connected /{prefix}; waiting for RGB and CameraInfo"
        self.events.record("INFO", "camera_connected", self.camera_status)
        return prefix

    def _receive(self, message, kind, prefix, connection):
        if connection != self.connection:
            return
        try:
            if kind == "rgb":
                value = frame_from_message(message, prefix, self.get_clock().now().nanoseconds)
            elif kind == "depth":
                if (message.header.frame_id != f"{prefix}_color_optical_frame"
                        or message.encoding != "16UC1" or message.is_bigendian != 0
                        or not 0 < message.width <= 4096 or not 0 < message.height <= 4096
                        or message.step != message.width * 2
                        or len(message.data) != message.width * message.height * 2):
                    raise ValueError("Depth must be packed registered 16UC1 millimetres")
                value = {"width": message.width, "height": message.height,
                         "depth": bytes(message.data), "stamp_ns": stamp_ns(message.header.stamp),
                         "received_at": time.monotonic()}
            else:
                value = validate_camera_info(message, prefix)
            with self.lock:
                if connection == self.connection:
                    previous = getattr(self, kind)
                    if kind in ("color_info", "depth_info") and previous != value:
                        self.invalidate("CameraInfo changed")
                    setattr(self, kind, value)
                    self.camera_status = f"Receiving /{prefix}"
        except ValueError as exc:
            with self.lock:
                if connection == self.connection:
                    setattr(self, kind, None)
                    if kind in ("rgb", "color_info"):
                        self.selected = None
                    if kind in ("color_info", "depth_info"):
                        self.invalidate("CameraInfo rejected")
                    self.camera_status = str(exc)
            self.events.record("WARNING", "input_rejected", str(exc), stream=kind)

    def inspect_model(self, path, expected_sha256=None):
        self.invalidate()
        path = Path(path).expanduser().resolve(strict=True)
        if path.suffix != ".pt" or not path.stat().st_size:
            raise ValueError("Select a non-empty local .pt model")
        config = {"path": str(path), "sha256": file_sha256(path)}
        if expected_sha256 is not None and config["sha256"] != expected_sha256:
            raise ValueError("Paired tray model changed")
        result, data = self.native.call({"operation": "inspect", "model": config})
        if (data or result.get("sha256") != config["sha256"]
                or result.get("task") not in ("detect", "segment", "obb")
                or not result.get("classes")):
            raise RuntimeError("Invalid native tray model metadata")
        with self.lock:
            self.model = {**config, "task": result["task"]}
            self.model_metadata = result
        self.events.record("INFO", "model_loaded", str(path), sha256=config["sha256"])
        return result

    def validate_sources(self, *, check_plane=True):
        if self.fatal_error or self.native.failed:
            raise RuntimeError(self.fatal_error or "Native tray worker failed")
        if self.camera is None:
            raise ValueError("Load a camera calibration")
        if self.camera_prefix != self.camera.settings.camera_prefix:
            raise ValueError("Connected prefix does not match camera calibration")
        if file_sha256(self.camera.path) != self.camera.sha256:
            raise ValueError("Camera calibration changed; reload it and re-teach the plane")
        if (check_plane and self.plane is not None and self.color_info is not None
                and self.plane["camera"] != self.color_info):
            self.selected = None
            raise ValueError("CameraInfo differs from taught plane; re-teach the plane")

    def raw_snapshot(self):
        with self.lock:
            rgb, depth = copy.copy(self.rgb), copy.copy(self.depth)
            info, depth_info = copy.deepcopy(self.color_info), copy.deepcopy(self.depth_info)
            generation, connection = self.generation, self.connection
        if (rgb is None or not 0 <= (self.get_clock().now().nanoseconds - rgb["stamp_ns"]) /
                1e9 <= .5 or time.monotonic() - rgb["received_at"] > .5):
            raise ValueError("Waiting for fresh RGB on the connected prefix")
        return {"rgb": rgb, "depth": depth, "info": info, "depth_info": depth_info,
                "generation": generation, "connection": connection, "camera_context": None,
                "metric_error": "", "depth_error": ""}

    def snapshot(self, *, depth_required=False, view=None):
        self.validate_sources(check_plane=False)
        view = self.raw_snapshot() if view is None else view
        rgb, depth, info, depth_info = (
            view[key] for key in ("rgb", "depth", "info", "depth_info"))
        camera = self.camera
        now = self.get_clock().now().nanoseconds
        if depth_required:
            validate_pair(rgb, depth, info, depth_info, now, QUALITY_DEFAULTS)
        elif (rgb is None or info is None
              or not 0 <= (now - rgb["stamp_ns"]) / 1e9 <= .5
              or time.monotonic() - rgb["received_at"] > .5
              or (info["width"], info["height"]) != (rgb["width"], rgb["height"])):
            raise ValueError("Fresh RGB and matching CameraInfo required")
        instant = Time(nanoseconds=rgb["stamp_ns"])
        # Robot TF may trail RGB by a few milliseconds. Wait only in the GUI's
        # background worker, at this exact timestamp, with one shared 100 ms budget.
        tf_deadline = time.monotonic() + .1
        internal = self.tf_buffer.lookup_transform(
            camera.settings.camera_link_frame, camera.settings.optical_frame, instant,
            timeout=Duration(seconds=max(0., tf_deadline - time.monotonic())))
        robot = None
        if camera.calibration_mode == "camera_on_hand":
            observed = self.tf_buffer.lookup_transform(
                "base_link", "Link6", instant,
                timeout=Duration(seconds=max(0., tf_deadline - time.monotonic())))
            now = self.get_clock().now().nanoseconds
            age = (now - stamp_ns(observed.header.stamp)) / 1e9
            if not 0 <= age <= 1.0:
                raise ValueError("Camera-on-hand requires fresh RGB-time robot TF")
            robot = transform_matrix(observed)
        now = self.get_clock().now().nanoseconds
        if depth_required:
            validate_pair(rgb, depth, info, depth_info, now, QUALITY_DEFAULTS)
        elif (not 0 <= (now - rgb["stamp_ns"]) / 1e9 <= .5
              or time.monotonic() - rgb["received_at"] > .5):
            raise ValueError("RGB expired while waiting for its timestamped TF")
        base_from_optical = resolve_base_from_camera_link(camera, robot) @ \
            transform_matrix(internal)
        return {**view,
                "camera_context": {"camera": info, "depth_camera": depth_info,
                                   "base_from_optical": base_from_optical.tolist()}}

    def _check_snapshot(self, view, *, calibrated=True):
        if calibrated:
            self.validate_sources(check_plane=False)
        if view["generation"] != self.generation:
            raise ValueError("Tray observation was invalidated; acquire another frame")
        if calibrated and view["camera_context"]["camera"] != self.color_info:
            raise ValueError("CameraInfo changed; acquire another frame")

    def freeze_for_plane(self):
        if self.position is None:
            raise ValueError("Copy Tray Teach Position from Item Teach before teaching a plane")
        # Re-teaching must remain possible after intrinsics changed.
        previous = self.plane
        self.plane = None
        try:
            view = self.snapshot(depth_required=True)
            return {**view, **self.visuals(view, [], cloud=False)}
        except Exception:
            self.plane = previous
            raise

    def capture_plane(self, view, pixels):
        self._check_snapshot(view)
        if view["camera_context"]["depth_camera"] != self.depth_info:
            raise ValueError("Depth CameraInfo changed; acquire another frame")
        rgb, depth = view["rgb"], view["depth"]
        result, data = self.native.call({
            "operation": "tray_plane", "width": rgb["width"], "height": rgb["height"],
            "camera_context": view["camera_context"], "pixels": pixels,
            "source_stamp_ns": rgb["stamp_ns"], "depth_stamp_ns": depth["stamp_ns"]},
            depth["depth"], timeout=10)
        if data or set(result) != {"state", "plane", "error"}:
            raise RuntimeError("Invalid native tray plane reply")
        if result["plane"] is None:
            raise ValueError(result["error"])
        try:
            validate_plane(result["plane"])
        except (ValueError, KeyError, TypeError) as exc:
            raise RuntimeError(f"Invalid native tray plane evidence: {exc}") from exc
        self._check_snapshot(view)
        with self.lock:
            self.plane = result["plane"]
            self.invalidate()
        self.events.record("INFO", "plane_taught", "Four corners captured in base_link",
                           max_error_mm=self.plane["max_error_mm"])
        return self.plane

    def preview(self, settings=None, *, generation):
        view = self.raw_snapshot()
        if view["generation"] != generation:
            raise ValueError("Tray preview was invalidated before processing")
        try:
            view = self.snapshot(view=view)
        except (ValueError, OSError, TransformException) as exc:
            view["metric_error"] = str(exc)
        try:
            validate_pair(view["rgb"], view["depth"], view["info"], view["depth_info"],
                          self.get_clock().now().nanoseconds, QUALITY_DEFAULTS)
        except ValueError as exc:
            view["depth"], view["depth_error"] = None, str(exc)
        rgb, context = view["rgb"], view["camera_context"]
        with self.lock:
            plane, model = copy.deepcopy(self.plane), copy.deepcopy(self.model)
        if context is None:
            plane = None
        elif plane is not None and plane["camera"] != context["camera"]:
            plane = None
            view["metric_error"] = "CameraInfo differs from taught plane; re-teach the plane"
        request = {"width": rgb["width"], "height": rgb["height"], "plane": plane,
                   "camera_context": context}
        if settings is None:
            request["operation"] = "tray_overlay"
        else:
            validate_preview(settings)
            if model is None or model["task"] != settings["model_task"]:
                raise ValueError("Load a matching YOLO model first")
            request.update(operation="tray_preview", settings=settings,
                           model={**model, "yolo": settings["yolo"]})
        result, pixels = self.native.call(request, rgb["rgb"], timeout=10)
        fields = {"state", "width", "height", "error"} if settings is None else {
            "state", "width", "height", "count", "detections", "selected", "reason",
            "inference_ms"}
        if (set(result) != fields or result.get("width") != rgb["width"]
                or result.get("height") != rgb["height"]
                or len(pixels) != rgb["width"] * rgb["height"] * 3):
            raise RuntimeError("Invalid native tray image reply")
        selected = result.get("selected")
        if settings is not None and (type(result["detections"]) is not list
                                     or type(result["count"]) is not int
                                     or not 0 <= result["count"] <=
                                     settings["yolo"]["max_detections"]):
            raise RuntimeError("Invalid native tray detection count")
        try:
            if selected is not None:
                if (type(selected) is not dict or selected not in result["detections"]
                        or not selected.get("valid") or len(selected["position"]) != 3
                        or len(selected["quaternion"]) != 4
                        or not np.isfinite(selected["position"] + selected["quaternion"]).all()
                        or abs(np.linalg.norm(selected["quaternion"]) - 1) > 1e-6):
                    raise RuntimeError("Invalid native selected tray pose")
        except (ValueError, KeyError, TypeError) as exc:
            raise RuntimeError(f"Invalid native selected tray pose: {exc}") from exc
        if settings is not None:
            try:
                identities = []
                for detection in result["detections"]:
                    polygon = np.asarray(detection["polygon"], dtype=float)
                    if (polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3
                            or not np.isfinite(polygon).all()
                            or type(detection["source_index"]) is not int
                            or type(detection["class_name"]) is not str
                            or type(detection["reason"]) is not str
                            or type(detection["valid"]) is not bool
                            or detection["size_status"] not in ("pass", "fail", "unchecked")
                            or not 0 <= detection["confidence"] <= 1):
                        raise ValueError("Malformed detection")
                    if "length_mm" in detection and not (
                            np.isfinite([detection["length_mm"], detection["width_mm"]]).all()
                            and detection["length_mm"] >= detection["width_mm"] >= 0):
                        raise ValueError("Malformed dimensions")
                    identities.append(detection["source_index"])
                if len(identities) > result["count"] or len(set(identities)) != len(identities):
                    raise ValueError("Duplicate or excess detections")
            except (ValueError, TypeError, KeyError) as exc:
                raise RuntimeError(f"Invalid native tray detections: {exc}") from exc
        visuals = self.visuals(view, result.get("detections", []), cloud=context is not None)
        self._check_snapshot(view, calibrated=context is not None)
        with self.lock:
            if view["generation"] != self.generation:
                raise ValueError("Tray preview was invalidated while processing")
        return {**view, **visuals, "overlay": pixels, "result": result}

    def accept_view(self, view):
        """Only the observation accepted by the GUI may replace the displayed cloud/pose."""
        self._check_snapshot(view, calibrated=view["camera_context"] is not None)
        with self.lock:
            if view["generation"] != self.generation:
                raise ValueError("Tray view changed before display")
            selected = view["result"].get("selected")
            self.selected = None if selected is None else (
                view["generation"], selected, view["rgb"]["stamp_ns"], time.monotonic())
            if view["cloud"] is not None:
                self.rviz.accept(view["cloud"])
            else:
                self.rviz.hold(view["metric_error"] or view["depth_error"] or "No valid voxels")

    def visuals(self, view, detections, *, cloud, pixels=()):
        rgb, depth = view["rgb"], view["depth"]
        if depth is None:
            return {"depth_overlay": b"", "samples": [], "cloud": None}
        result, data = self.native.call({
            "operation": "tray_visuals", "width": rgb["width"], "height": rgb["height"],
            "camera_context": view["camera_context"], "cloud": cloud,
            "detections": detections, "pixels": list(pixels)}, rgb["rgb"] + depth["depth"],
            timeout=10)
        size = rgb["width"] * rgb["height"] * 3
        if (set(result) != {"state", "width", "height", "point_count", "samples"}
                or result["width"] != rgb["width"] or result["height"] != rgb["height"]
                or type(result["point_count"]) is not int
                or not 0 <= result["point_count"] <= rgb["width"] * rgb["height"]
                or len(data) != 2 * size + result["point_count"] * 16
                or type(result["samples"]) is not list or len(result["samples"]) != len(pixels)
                or (not cloud and result["point_count"] != 0)
                or not np.isfinite(np.frombuffer(
                    data[2 * size:], "<f4").reshape(-1, 4)[:, :3]).all()):
            raise RuntimeError("Invalid native tray visualization reply")
        for index, sample in enumerate(result["samples"], 1):
            if (type(sample) is not dict or set(sample) != {
                    "index", "depth_pixel", "accepted", "median_mm", "reason"}
                    or sample["index"] != index or type(sample["accepted"]) is not int
                    or not 0 <= sample["accepted"] <= 49
                    or type(sample["reason"]) is not str
                    or type(sample["depth_pixel"]) is not list or len(sample["depth_pixel"]) != 2
                    or any(type(v) is not int for v in sample["depth_pixel"])
                    or (sample["median_mm"] is not None and (
                        type(sample["median_mm"]) not in (float, int)
                        or not np.isfinite(sample["median_mm"])))):
                raise RuntimeError("Invalid native tray corner evidence")
        sample = None if not cloud else {
            "data": data[2 * size:], "point_count": result["point_count"],
            "stamp_ns": rgb["stamp_ns"], "depth_stamp_ns": depth["stamp_ns"],
            "connection": view["connection"], "detections": detections}
        return {"depth_overlay": data[:size], "corner_overlay": data[size:2 * size],
                "samples": result["samples"], "cloud": sample}

    def corner_preview(self, view, pixels):
        self._check_snapshot(view)
        result = self.visuals(view, [], cloud=False, pixels=pixels)
        self._check_snapshot(view)
        return result

    def load_saved(self, path):
        profile = load_profile(path, self.root)
        camera_path = self.root / "calibration" / profile["camera_calibration"]["filename"]
        camera = load_camera_calibration(camera_path, root=self.root)
        if camera.sha256 != profile["camera_calibration"]["sha256"]:
            raise ValueError("Tray profile's camera calibration hash changed")
        metadata = self.inspect_model(Path(path).with_suffix(".pt"), profile["model"]["sha256"])
        settings = profile["settings"]
        if metadata["task"] != settings["model_task"] or any(
                str(i) not in metadata["classes"] for i in settings["yolo"]["class_ids"]):
            raise ValueError("Tray profile settings do not match the verified model")
        self.apply_camera(camera.path)
        with self.lock:
            self.position = copy.deepcopy(profile["tray_teach_position"])
            self.plane = copy.deepcopy(profile["reference_plane"])
        return profile

    def _tick(self):
        process = self.native.process
        if (process is not None and process.poll() is not None and not self.native.closed
                and not self.fatal_error):
            self.fatal_error = "Tray native worker exited; restart the node"
            self.native.failed = True
            self.events.record("FATAL", "worker_exit", self.fatal_error, code=process.returncode)
        if self.native.failed or self.fatal_error:
            self.rviz.invalidate("Tray native worker failed")
        elif self.rviz.displayed is not None:
            try:
                self.validate_sources(check_plane=False)
            except (ValueError, OSError, RuntimeError) as exc:
                self.rviz.invalidate(str(exc))
        self.rviz.tick()
        with self.lock:
            selected = self.selected
        if selected is None:
            return
        generation, pose, source_stamp, receipt = selected
        if (generation != self.generation or time.monotonic() - receipt > 2.5
                or self.native.failed or self.fatal_error):
            self.selected = None
            return
        try:
            self.validate_sources()
        except (ValueError, OSError, RuntimeError):
            self.selected = None
            return
        if self._published_key == (generation, source_stamp):
            return
        message = TransformStamped()
        message.header.frame_id, message.child_frame_id = "base_link", "tray_teach_selected_tray"
        message.header.stamp = Time(nanoseconds=source_stamp).to_msg()
        t, q = message.transform.translation, message.transform.rotation
        t.x, t.y, t.z = map(float, pose["position"])
        q.x, q.y, q.z, q.w = map(float, pose["quaternion"])
        self.broadcaster.sendTransform(message)
        self._published_key = (generation, source_stamp)

    def close(self):
        self.invalidate("Tray Teach closed")
        self.native.close()
