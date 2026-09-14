"""Shared GUI/headless read-only detector. No robot command clients or cv2 imports."""

import copy
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener, TransformException

from item_perception_interfaces.msg import ItemCandidate
from item_perception_interfaces.srv import GetItemPoses
from .bin_teach_core import (
    load_bin_teach, load_bin_teach_calibration_context,
    validate_applied_sources, compose_platform_from_optical, place_bin_roi,
)
from .platform_teach_core import PackageEventLogger, resolve_base_from_camera_link
from .item_teach_core import (file_sha256, load_item_profile, settings_from_profile,
                              validate_detection_settings, detection_settings, validate_quality)
from .item_preview import frame_from_message, validate_prefix, validate_preview_settings
from .item_native_client import NativeClient


SERVICE_NAME = "/item_detect/get_item_poses"
# Visible initial form values, never a fallback for invalid/missing inputs.
INITIAL_PREVIEW_YOLO = {"confidence": 0.25, "iou": 0.7, "image_size": 640,
                        "max_detections": 100}
PREVIEW_MAX_AGE_SEC = 0.5


def validate_roi_status(status):
    if (type(status) is not dict or set(status) != {"visible", "reason"}
            or type(status["visible"]) is not bool or type(status["reason"]) is not str
            or status["visible"] == bool(status["reason"])):
        raise RuntimeError("Malformed ROI overlay status")


def validate_preview_detections(detections, count, class_ids):
    if type(detections) is not list or len(detections) > count:
        raise RuntimeError("Malformed clickable detection list")
    seen = set()
    for item in detections:
        index = item["source_index"]
        if (type(index) is not int or not 0 <= index < count or index in seen
                or item["class_id"] not in class_ids or type(item["class_name"]) is not str
                or not math.isfinite(item["confidence"]) or not 0 <= item["confidence"] <= 1):
            raise RuntimeError("Invalid clickable detection identity")
        seen.add(index)
        for field in ("rectangle", "polygon"):
            points = item[field]
            if (len(points) < 3 or (field == "rectangle" and len(points) != 4)
                    or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points)):
                raise RuntimeError("Malformed clickable geometry")
        measured = item["measurement"]
        if type(item["measurement_error"]) is not str:
            raise RuntimeError("Missing measurement availability reason")
        if measured is not None:
            if (set(measured) != {"length_mm", "width_mm"}
                    or not all(math.isfinite(v) and v > 0 for v in measured.values())
                    or measured["length_mm"] < measured["width_mm"]
                    or item["measurement_error"]):
                raise RuntimeError("Malformed plane measurement")
        elif not item["measurement_error"]:
            raise RuntimeError("Unavailable measurement has no reason")
        if (item["size_valid"] is not None and type(item["size_valid"]) is not bool
                or type(item["size_reason"]) is not str or not item["size_reason"]):
            raise RuntimeError("Malformed size check")
        circle, reason = item["sampling_circle"], item["sampling_circle_error"]
        if type(reason) is not str or (circle is None) != bool(reason):
            raise RuntimeError("Malformed sampling circle availability")
        if circle is not None and (
                type(circle) is not list or len(circle) != 96
                or any(len(p) != 2 or any(type(v) not in (int, float)
                       or not math.isfinite(v) or abs(v) > 2_000_000_000 for v in p)
                       for p in circle)):
            raise RuntimeError("Malformed sampling circle pixels")


def validate_candidates(result, settings):
    candidates = result["candidates"]
    if len(candidates) > settings["yolo"]["max_detections"]:
        raise RuntimeError("Native candidate count exceeded detection cap")
    seen = set()
    last_key = None
    for candidate in candidates:
        index = candidate["source_index"]
        if type(index) is not int or index < 0 or index in seen:
            raise RuntimeError("Invalid/duplicate native candidate ID")
        seen.add(index)
        for name, count in (("position", 3), ("quaternion", 4), ("pixel", 2)):
            values = candidate[name]
            if len(values) != count or not all(type(v) in (int, float) and math.isfinite(v)
                                               for v in values):
                raise RuntimeError("Malformed native pose")
        if abs(sum(v*v for v in candidate["quaternion"]) - 1) > 1e-6:
            raise RuntimeError("Native quaternion is not normalized")
        for name in ("confidence", "length", "width", "center_distance",
                     "filtered_camera_depth", "depth_sigma"):
            if not math.isfinite(candidate[name]) or candidate[name] < 0:
                raise RuntimeError("Non-finite/negative native candidate metric")
        if (candidate["class_id"] not in settings["yolo"]["class_ids"]
                or not settings["yolo"]["confidence"] <= candidate["confidence"] <= 1):
            raise RuntimeError("Native candidate failed class/confidence contract")
        good, bad = candidate["accepted_depth_count"], candidate["rejected_depth_count"]
        if (type(good) is not int or type(bad) is not int or bad < 0
                or good < settings["quality"]["minimum_depth_samples"]
                or good / (good+bad) < settings["quality"]["minimum_depth_fraction"]):
            raise RuntimeError("Native candidate failed depth-quality contract")
        key = (candidate["center_distance"], -candidate["confidence"], index)
        if last_key is not None and key < last_key:
            raise RuntimeError("Native candidates are not in priority order")
        last_key = key


def stamp_ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("Invalid timestamp")
    value = stamp.sec * 1_000_000_000 + stamp.nanosec
    if value <= 0:
        raise ValueError("Timestamp must be non-zero")
    return value


def transform_matrix(message):
    t, q = message.transform.translation, message.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    if not all(math.isfinite(v) for v in (t.x, t.y, t.z, x, y, z, w)):
        raise ValueError("Non-finite TF")
    if abs(x*x + y*y + z*z + w*w - 1) > 1e-5:
        raise ValueError("TF quaternion is not normalized")
    matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def validate_camera_info(message, prefix):
    if message.header.frame_id != f"{prefix}_color_optical_frame":
        raise ValueError("CameraInfo is not registered to the selected color optical frame")
    stamp_ns(message.header.stamp)
    k, d = list(message.k), list(message.d)
    if (message.distortion_model not in ("plumb_bob", "rational_polynomial")
            or len(d) not in (4, 5, 8, 12, 14) or len(k) != 9
            or not all(math.isfinite(v) for v in k + d)
            or k[0] <= 0 or k[4] <= 0 or k[1] != 0 or k[3] != 0
            or k[6:] != [0.0, 0.0, 1.0]
            or not 0 < message.width <= 4096 or not 0 < message.height <= 4096
            or not 0 <= k[2] < message.width or not 0 <= k[5] < message.height):
        raise ValueError("Invalid color/registered-depth intrinsics or distortion")
    return {"width": message.width, "height": message.height, "k": k, "d": d,
            "distortion_model": message.distortion_model}


def validate_pair(rgb, depth, color_info, depth_info, now_ns, quality):
    if any(value is None for value in (rgb, depth, color_info, depth_info)):
        raise ValueError("Waiting for RGB, registered depth, and both CameraInfo topics")
    for label, image in (("RGB", rgb), ("depth", depth)):
        age = (now_ns - image["stamp_ns"]) / 1e9
        if not 0 <= age <= quality["input_max_age_sec"]:
            raise ValueError(f"{label} source timestamp stale/future: {age:.3f}s")
        if time.monotonic() - image["received_at"] > quality["input_max_age_sec"]:
            raise ValueError(f"{label} receipt stale")
    if abs(rgb["stamp_ns"] - depth["stamp_ns"]) / 1e9 > quality["sync_tolerance_sec"]:
        raise ValueError("RGB/depth synchronization tolerance exceeded")
    dimensions = (rgb["width"], rgb["height"])
    if any((v["width"], v["height"]) != dimensions for v in (depth, color_info, depth_info)):
        raise ValueError("Registered depth and CameraInfo must exactly match RGB dimensions")
    if color_info["k"] != depth_info["k"]:
        raise ValueError("Registered-depth intrinsics K must exactly match color CameraInfo; "
                         "check camera D2C alignment settings")
    # D2C can publish rectified depth alongside distorted raw RGB. Preserve
    # both validated models: the worker maps rays, never assumes equal pixels.


class ItemDetectNode(Node):
    def __init__(self, name="item_detect"):
        super().__init__(name)
        self.events = PackageEventLogger(node_name=name)
        self.native = NativeClient(self.events)
        self._feedback_lock = threading.Lock()
        self.request_lock = threading.Lock()
        self.operation_lock = threading.Lock()
        self.condition = threading.Condition(self._feedback_lock)
        self.camera_prefix = None
        self._camera_generation = 0
        self._image_sequence = 0
        self._image = self._depth = self._color_info = self._depth_info = None
        self._camera_subscriptions = []
        self.camera_status = "Camera not connected"
        self.model_metadata = self.model_config = None
        self.applied = self.bin_artifact = None
        self.settings = self.profile_path = None
        self.profile_digest = None
        self.yolo_enabled = False
        self.preview_mode = "all"
        self.preview_source = "none"
        self.preview_yolo = None
        self.preview_geometry = self.preview_quality = None
        self.preview_depth_diameter = None
        self.service = None
        self.arm_epoch = 0
        self.last_view = None
        self.fatal_error = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=15))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_timer(0.1, self._check_worker)

    def _check_worker(self):
        process = self.native.process
        if (process is not None and process.poll() is not None and not self.native.closed
                and self.fatal_error is None):
            self.fatal_error = f"Item worker PID {process.pid} exited: {process.returncode}"
            self.native.failed = True
            self.disarm()
            self.events.record("FATAL", "item_worker_exit", self.fatal_error,
                               pid=process.pid, exit_code=process.returncode)
            self.get_logger().fatal(self.fatal_error)

    def connect_camera(self, prefix):
        validate_prefix(prefix)
        self.disarm()
        self.yolo_enabled = False
        if self.applied is not None and prefix != self.applied.camera.settings.camera_prefix:
            self.applied = self.bin_artifact = None
        with self.condition:
            self._camera_generation += 1
            generation = self._camera_generation
            self.camera_prefix = prefix
            self._image = self._depth = self._color_info = self._depth_info = None
            self.last_view = None
            self.condition.notify_all()
        for subscription in self._camera_subscriptions:
            self.destroy_subscription(subscription)
        self._camera_subscriptions = []
        for kind, topic, message_type in (
            ("rgb", "color/image_raw", Image), ("depth", "depth/image_raw", Image),
            ("color_info", "color/camera_info", CameraInfo),
            ("depth_info", "depth/camera_info", CameraInfo),
        ):
            self._camera_subscriptions.append(self.create_subscription(
                message_type, f"/{prefix}/{topic}",
                lambda message, k=kind: self._receive(message, prefix, generation, k),
                qos_profile_sensor_data))
        self.camera_status = f"Connected /{prefix}; waiting for streams"

    def _receive(self, message, prefix, generation, kind):
        field = {"rgb": "_image", "depth": "_depth", "color_info": "_color_info",
                 "depth_info": "_depth_info"}[kind]
        try:
            if kind == "rgb":
                # Callback validates layout only; freshness uses explicit profile limits below.
                value = frame_from_message(message, prefix, stamp_ns(message.header.stamp))
            elif kind == "depth":
                if (message.header.frame_id != f"{prefix}_color_optical_frame"
                        or message.encoding != "16UC1" or message.is_bigendian != 0
                        or not 0 < message.width <= 4096 or not 0 < message.height <= 4096
                        or message.step != message.width * 2
                        or len(message.data) != message.width * message.height * 2):
                    raise ValueError(
                        "Depth must be packed registered little-endian 16UC1 millimetres")
                value = {"width": message.width, "height": message.height,
                         "depth": bytes(message.data), "stamp_ns": stamp_ns(message.header.stamp),
                         "received_at": time.monotonic()}
            else:
                value = validate_camera_info(message, prefix)
        except ValueError as exc:
            with self.condition:
                if generation != self._camera_generation:
                    return
                setattr(self, field, None)
                self.camera_status = str(exc)
                self.condition.notify_all()
            self.events.record("WARNING", "item_message_rejected", str(exc), stream=kind)
            return
        with self.condition:
            if generation != self._camera_generation:
                return
            if kind == "rgb":
                self._image_sequence += 1
                value["sequence"] = self._image_sequence
            setattr(self, field, value)
            self.camera_status = f"Receiving /{prefix}"
            self.condition.notify_all()

    def camera_snapshot(self):
        with self._feedback_lock:
            return self._image, self.camera_status

    def inspect_model(self, path):
        self.disarm()
        self.yolo_enabled = False
        path = Path(path).expanduser().resolve(strict=True)
        if path.suffix != ".pt" or path.stat().st_size == 0:
            raise ValueError("Select a non-empty .pt model")
        config = {"path": str(path), "sha256": file_sha256(path)}
        metadata, payload = self.native.call({"operation": "inspect", "model": config})
        if payload or not metadata.get("classes") or metadata["sha256"] != config["sha256"]:
            raise RuntimeError("Invalid model metadata")
        self.model_metadata = metadata
        self.model_config = {**config, "task": metadata["task"]}
        self.events.record("INFO", "item_model_loaded", "Loaded trusted local model",
                           **config, task=metadata["task"], classes=metadata["classes"])
        return metadata

    def apply_station(self, platform_path, bin_path):
        self.disarm()
        applied = load_bin_teach_calibration_context(Path(platform_path))
        template = load_bin_teach(Path(bin_path))
        place_bin_roi(template, applied.platform)
        self.applied, self.bin_artifact = applied, template
        self.connect_camera(applied.camera.settings.camera_prefix)
        self.events.record("INFO", "item_station_applied", "Validated station and bin ROI",
                           platform=str(applied.platform.path), bin=str(template.path))

    def enable_yolo(self, settings):
        validate_detection_settings(settings, geometry_required=self.applied is not None)
        if self.model_config is None or self.model_metadata is None:
            raise ValueError("Load/inspect the model first")
        if settings["model_task"] != self.model_metadata["task"]:
            raise ValueError("Selected task does not match loaded model")
        names = {int(key) for key in self.model_metadata["classes"]}
        if not set(settings["yolo"]["class_ids"]).issubset(names):
            raise ValueError("Checked class not present in loaded model")
        self.settings = copy.deepcopy(settings)
        self.yolo_enabled = True

    def enable_preview(self, source, yolo, *, geometry, quality, diameter_mm):
        if self.model_config is None or self.model_metadata is None:
            raise ValueError("Load/inspect the model first")
        if source != "none" and source not in self.model_metadata["geometry_sources"]:
            raise ValueError("Selected preview geometry is unavailable")
        # Display all classes, including size failures. Production and manual
        # pose validation remain separate from the displayed detections.
        yolo = {**yolo, "class_ids": sorted(int(k) for k in self.model_metadata["classes"])}
        validate_preview_settings(self.model_metadata["task"], yolo)
        validate_quality(quality)
        if not math.isfinite(diameter_mm) or diameter_mm <= 0:
            raise ValueError("pickdepth_radius must be a positive diameter in millimetres")
        if geometry is not None:
            validate_detection_settings({"model_task": self.model_metadata["task"],
                                         "yolo": yolo, "quality": quality,
                                         "geometry_source": source, "geometry": geometry},
                                        geometry_required=source != "none")
        self.preview_yolo = copy.deepcopy(yolo)
        self.preview_geometry = copy.deepcopy(geometry)
        self.preview_quality = copy.deepcopy(quality)
        self.preview_depth_diameter = diameter_mm
        self.preview_source = source
        self.preview_mode = "all"
        self.yolo_enabled = True
        self.events.record("INFO", "item_teaching_preview_enabled",
                           "All detections with size borders; pose calculation on click only",
                           yolo=self.preview_yolo, geometry_source=source)

    def disarm(self):
        self.arm_epoch += 1
        if self.service is not None:
            service, self.service = self.service, None
            self.destroy_service(service)
            self.events.record("INFO", "item_disarmed", "Pose service removed")

    def arm(self, path):
        self.disarm()
        if not self.yolo_enabled or self.native.failed or self.applied is None:
            raise ValueError("Arming requires YOLO ON, loaded model and applied station/bin")
        profile, digest = load_item_profile(Path(path))
        if detection_settings(settings_from_profile(profile)) != self.settings:
            raise ValueError("Save or load the exact currently applied item settings first")
        if (profile["model"]["sha256"] != self.model_config["sha256"]
                or self.settings["geometry_source"] not in
                self.model_metadata["geometry_sources"]):
            raise ValueError("Model/source mismatch or no verified mask/OBB output")
        self._validate_sources()
        own = (self.get_name(), self.get_namespace())
        nodes = self.get_node_names_and_namespaces()
        if nodes.count(own) > 1:
            raise ValueError("Duplicate detector node identity")
        for name, namespace in nodes:
            services = self.get_service_names_and_types_by_node(name, namespace)
            if (name, namespace) != own and any(s == SERVICE_NAME for s, _ in services):
                raise ValueError("Another detector already advertises the canonical pose service")
        self._check_arm_inputs()
        self.profile_path, self.profile_digest = Path(path), digest
        self.retry_limit = profile["retry"]["retry_limit"]
        self.service = self.create_service(GetItemPoses, SERVICE_NAME, self._request,
                                           callback_group=ReentrantCallbackGroup())
        self.events.record("INFO", "item_armed", "Pose service advertised", profile_sha256=digest)

    def _validate_sources(self):
        if self.applied is None or self.bin_artifact is None:
            raise ValueError("Valid station platform and bin teach files are required")
        try:
            validate_applied_sources(self.applied)
            if file_sha256(self.bin_artifact.path) != self.bin_artifact.sha256:
                raise ValueError("Applied bin teach changed")
        except (ValueError, OSError):
            self.last_view = None
            if self.service is not None:
                self.disarm()
            raise

    def _check_arm_inputs(self):
        """Availability gate only; generated poses always use image-timestamp TF."""
        if (self.applied is None
                or self.camera_prefix != self.applied.camera.settings.camera_prefix):
            raise ValueError("Connected camera does not match station calibration")
        quality = self.settings["quality"]
        now_ns = self.get_clock().now().nanoseconds
        with self.condition:
            validate_pair(self._image, self._depth, self._color_info, self._depth_info,
                          now_ns, quality)
        camera = self.applied.camera
        try:
            internal = self.tf_buffer.lookup_transform(
                camera.settings.camera_link_frame,
                camera.settings.optical_frame, Time())
            transform_matrix(internal)
            if camera.calibration_mode == "camera_on_hand":
                robot = self.tf_buffer.lookup_transform("base_link", "Link6", Time())
                age = (now_ns - stamp_ns(robot.header.stamp)) / 1e9
                if not 0 <= age <= quality["robot_tf_max_age_sec"]:
                    raise ValueError("On-hand robot TF is stale/future")
                transform_matrix(robot)
        except TransformException as exc:
            raise ValueError(f"Required TF unavailable: {exc}") from exc

    def _snapshot(self, after_ns, deadline, *, wait):
        quality = self.settings["quality"]
        if (self.applied is None
                or self.camera_prefix != self.applied.camera.settings.camera_prefix):
            raise ValueError("Connected camera does not match applied station calibration")
        epoch = self.arm_epoch
        selected = None
        while True:
            if epoch != self.arm_epoch:
                raise ValueError("Detector settings/arming changed while acquiring observation")
            if selected is None:
                with self.condition:
                    rgb, depth = self._image, self._depth
                    color_info, depth_info = self._color_info, self._depth_info
                    try:
                        now_ns = self.get_clock().now().nanoseconds
                        validate_pair(rgb, depth, color_info, depth_info, now_ns, quality)
                        if min(rgb["stamp_ns"], depth["stamp_ns"]) < after_ns:
                            raise ValueError("Waiting for a new observation after request")
                    except ValueError:
                        remaining = deadline - time.monotonic()
                        if not wait or remaining <= 0:
                            raise
                        self.condition.wait(timeout=min(remaining, 0.05))
                        continue
                    selected = (dict(rgb), dict(depth), copy.deepcopy(color_info),
                                copy.deepcopy(depth_info))
            rgb, depth, color_info, depth_info = selected
            now_ns = self.get_clock().now().nanoseconds
            validate_pair(rgb, depth, color_info, depth_info, now_ns, quality)
            camera = self.applied.camera
            instant = Time(nanoseconds=rgb["stamp_ns"])
            try:
                internal = self.tf_buffer.lookup_transform(camera.settings.camera_link_frame,
                                                           camera.settings.optical_frame, instant)
                robot = None
                if camera.calibration_mode == "camera_on_hand":
                    robot = self.tf_buffer.lookup_transform("base_link", "Link6", instant)
                    age = (now_ns - stamp_ns(robot.header.stamp)) / 1e9
                    if not 0 <= age <= quality["robot_tf_max_age_sec"]:
                        raise ValueError("On-hand robot TF is stale/future")
                base_camera = resolve_base_from_camera_link(
                    camera, None if robot is None else transform_matrix(robot))
                transform = compose_platform_from_optical(self.applied.platform.base_from_platform,
                                                          base_camera, transform_matrix(internal))
            except (TransformException, ValueError) as exc:
                if wait and time.monotonic() < deadline:
                    with self.condition:
                        self.condition.wait(timeout=0.02)
                    continue
                raise ValueError(f"Camera/robot TF unavailable: {exc}") from exc
            context = {"camera": color_info, "depth_camera": depth_info,
                       "platform_from_optical": transform.tolist(),
                       "roi": [[p.x_m, p.y_m] for p in self.bin_artifact.points]}
            return rgb, depth, context

    def infer(self, rgb, depth=None, context=None, timeout=30, *, preview=None):
        settings = copy.deepcopy(self.settings if preview is None else preview["settings"])
        config = {**self.model_config, "yolo": settings["yolo"]}
        header = {"operation": "detect", "generation": self.arm_epoch, "model": config,
                  "width": rgb["width"], "height": rgb["height"], "context": context,
                  "settings": settings}
        if preview is not None:
            header.update(operation="preview", geometry_source=self.preview_source,
                          measurement_context=preview["context"],
                          measurement_error=preview["error"], preview_depth=depth is not None)
        payload = rgb["rgb"] + (b"" if depth is None else depth["depth"])
        result, pixels = self.native.call(header, payload, timeout)
        frame_bytes = len(rgb["rgb"])
        required_fields = {"state", "generation", "width", "height", "count", "task",
                           "inference_ms", "geometry_sources", "candidates", "rejected",
                           "has_depth_view", "detections", "roi_overlay"}
        if (set(result) != required_fields
                or result.get("width") != rgb["width"] or result.get("height") != rgb["height"]
                or result.get("generation") != header["generation"]
                or result.get("has_depth_view") is not (depth is not None)
                or len(pixels) != frame_bytes * (2 if depth is not None else 1)
                or type(result.get("candidates")) is not list
                or type(result.get("count")) is not int
                or not 0 <= result["count"] <= settings["yolo"]["max_detections"]):
            self.native.failed = True
            self.native.close()
            raise RuntimeError("Malformed native detection response")
        try:
            if (result["task"] != config["task"]
                    or type(result["geometry_sources"]) is not list
                    or len(result["geometry_sources"]) != len(set(result["geometry_sources"]))
                    or not set(result["geometry_sources"]).issubset({"mask", "obb"})
                    or type(result["rejected"]) is not list
                    or not math.isfinite(result["inference_ms"]) or result["inference_ms"] < 0):
                raise RuntimeError("Invalid native task/source/timing diagnostics")
            validate_candidates(result, settings)
            validate_roi_status(result["roi_overlay"])
            validate_preview_detections(result["detections"], result["count"],
                                        settings["yolo"]["class_ids"])
            if preview is not None and (result["candidates"] or result["rejected"]):
                raise RuntimeError("All-detections preview must not generate pick candidates")
        except (KeyError, TypeError, ValueError, RuntimeError) as exc:
            self.native.failed = True
            self.native.close()
            self.events.record("FATAL", "item_native_result_invalid", str(exc))
            raise RuntimeError(f"Malformed native result: {exc}") from exc
        self.model_metadata["geometry_sources"] = result["geometry_sources"]
        view = {**rgb, "rgb": pixels[:frame_bytes], "metadata": result,
                "preview_mode": "all" if preview is not None else "filtered",
                "depth_rgb": pixels[frame_bytes:],
                "depth_stamp_ns": None if depth is None else depth["stamp_ns"]}
        return view

    def _measurement_context(self, rgb, robot_max_age=1.0):
        """RGB-time plane geometry only: no depth, ROI filtering or taught dimensions."""
        if self.applied is None:
            raise ValueError("Select valid station calibration to measure on platform Z=0")
        if self.camera_prefix != self.applied.camera.settings.camera_prefix:
            raise ValueError("Camera does not match applied station")
        self._validate_sources()
        with self._feedback_lock:
            info = copy.deepcopy(self._color_info)
        if info is None or (info["width"], info["height"]) != (rgb["width"], rgb["height"]):
            raise ValueError("Matching color CameraInfo required for measurement")
        camera = self.applied.camera
        instant = Time(nanoseconds=rgb["stamp_ns"])
        internal = self.tf_buffer.lookup_transform(camera.settings.camera_link_frame,
                                                   camera.settings.optical_frame, instant)
        robot = None
        if camera.calibration_mode == "camera_on_hand":
            robot = self.tf_buffer.lookup_transform("base_link", "Link6", instant)
            age = (self.get_clock().now().nanoseconds - stamp_ns(robot.header.stamp)) / 1e9
            if not 0 <= age <= robot_max_age:
                raise ValueError(f"Measurement needs on-hand robot TF <= {robot_max_age:g}s")
        base_camera = resolve_base_from_camera_link(
            camera, None if robot is None else transform_matrix(robot))
        transform = compose_platform_from_optical(self.applied.platform.base_from_platform,
                                                  base_camera, transform_matrix(internal))
        return {"camera": info, "platform_from_optical": transform.tolist(),
                "roi": [[p.x_m, p.y_m] for p in self.bin_artifact.points]}

    def roi_once(self):
        """YOLO-OFF station inspection; the same lifetime worker projects geometry only."""
        if self.yolo_enabled or self.applied is None or self.request_lock.locked():
            return None
        if not self.operation_lock.acquire(blocking=False):
            return None
        epoch = self.arm_epoch
        try:
            rgb, _status = self.camera_snapshot()
            if rgb is None:
                return None
            context, error = None, ""
            try:
                age = (self.get_clock().now().nanoseconds - rgb["stamp_ns"]) / 1e9
                if (not 0 <= age <= PREVIEW_MAX_AGE_SEC
                        or time.monotonic() - rgb["received_at"] > PREVIEW_MAX_AGE_SEC):
                    raise ValueError("RGB is stale; loaded ROI hidden")
                context = self._measurement_context(rgb)
            except (ValueError, OSError, TransformException) as exc:
                error = str(exc)
            status = {"visible": False, "reason": error}
            pixels = rgb["rgb"]
            if context is not None:
                result, pixels = self.native.call({
                    "operation": "overlay_roi", "generation": epoch,
                    "width": rgb["width"], "height": rgb["height"],
                    "context": context, "error": ""}, pixels)
                try:
                    if (set(result) != {"state", "generation", "width", "height", "roi_overlay"}
                            or result["generation"] != epoch
                            or (result["width"], result["height"]) != (rgb["width"], rgb["height"])
                            or len(pixels) != len(rgb["rgb"])):
                        raise RuntimeError("Malformed ROI projection result")
                    validate_roi_status(result["roi_overlay"])
                    status = result["roi_overlay"]
                except (KeyError, TypeError, RuntimeError) as exc:
                    self.native.failed = True
                    self.native.close()
                    self.events.record("FATAL", "item_native_result_invalid", str(exc))
                    raise
            if epoch != self.arm_epoch or self.yolo_enabled:
                return None
            view = {**rgb, "rgb": pixels, "preview_mode": "roi",
                    "metadata": {"roi_overlay": status}}
            self.last_view = view
            return view
        finally:
            self.operation_lock.release()

    def preview_once(self):
        if not self.yolo_enabled or self.request_lock.locked():
            return None
        if not self.operation_lock.acquire(blocking=False):
            return None
        epoch = self.arm_epoch
        try:
            rgb, _status = self.camera_snapshot()
            if rgb is None:
                return None
            age = (self.get_clock().now().nanoseconds - rgb["stamp_ns"]) / 1e9
            max_age = (self.preview_quality["input_max_age_sec"] if self.preview_mode == "all"
                       else self.settings["quality"]["input_max_age_sec"])
            if (not 0 <= age <= max_age
                    or time.monotonic() - rgb["received_at"] > max_age):
                raise ValueError("RGB preview is stale")
            if self.preview_mode == "all":
                measurement, error = None, ""
                try:
                    measurement = self._measurement_context(
                        rgb, self.preview_quality["robot_tf_max_age_sec"])
                except (ValueError, OSError, TransformException) as exc:
                    # Measurement is a separate optional display, not a pick-pose fallback.
                    error = str(exc)
                depth, depth_error = None, ""
                with self.condition:
                    try:
                        validate_pair(rgb, self._depth, self._color_info, self._depth_info,
                                      self.get_clock().now().nanoseconds, self.preview_quality)
                        if measurement is not None:
                            if measurement["camera"] != self._color_info:
                                raise ValueError("Color CameraInfo changed during observation")
                            measurement["depth_camera"] = copy.deepcopy(self._depth_info)
                        depth = dict(self._depth)
                    except ValueError as exc:
                        depth_error = str(exc)
                preview = {"settings": {"yolo": copy.deepcopy(self.preview_yolo),
                                        "geometry": copy.deepcopy(self.preview_geometry),
                                        "quality": copy.deepcopy(self.preview_quality),
                                        "pickdepth_radius": self.preview_depth_diameter},
                           "context": measurement, "error": error}
                view = self.infer(rgb, depth, preview=preview)
                # Only this one bounded, displayed observation can be selected.
                # Never attach a newer depth frame/TF to an older detection.
                view["observation"] = {"rgb": rgb, "depth": depth, "context": measurement,
                                       "epoch": epoch,
                                       "camera_generation": self._camera_generation,
                                       "error": error or depth_error}
                view["depth_error"] = depth_error
                if epoch != self.arm_epoch or not self.yolo_enabled:
                    return None
                self.last_view = view
                return view
            depth = context = None
            if self.applied is not None and self.settings["geometry_source"] != "none":
                self._validate_sources()
                deadline = time.monotonic() + self.settings["quality"]["input_max_age_sec"]
                rgb, depth, context = self._snapshot(0, deadline, wait=True)
            view = self.infer(rgb, depth, context)
            if epoch != self.arm_epoch or not self.yolo_enabled:
                return None
            self.last_view = view
            return view
        finally:
            self.operation_lock.release()

    def clicked_pose(self, view, detection, settings):
        """Teaching-only result from one exact displayed snapshot; never a service response."""
        validate_detection_settings(settings, geometry_required=True)
        observation = view.get("observation")
        if observation is None or observation["context"] is None or observation["depth"] is None:
            raise ValueError("Pose unavailable: " + (
                observation["error"] if observation else "no RGB/depth observation"))
        if (observation["epoch"] != self.arm_epoch
                or observation["camera_generation"] != self._camera_generation):
            raise ValueError("Selection belongs to changed settings/camera; resume live")
        if detection not in view["metadata"]["detections"]:
            raise ValueError("Selection does not belong to the displayed result")
        self._validate_sources()
        rgb, depth = observation["rgb"], observation["depth"]

        def check_age():
            age = (self.get_clock().now().nanoseconds - min(
                rgb["stamp_ns"], depth["stamp_ns"])) / 1e9
            if not 0 <= age <= settings["quality"]["result_max_age_sec"]:
                raise ValueError("Selected snapshot is too old for a pose; click to resume live")
        check_age()
        if not self.operation_lock.acquire(blocking=False):
            raise ValueError("Pose operation busy; no selected pose published")
        try:
            header = {"operation": "selected_pose", "generation": self.arm_epoch,
                      "width": rgb["width"], "height": rgb["height"],
                      "detection": detection, "settings": settings,
                      "context": observation["context"]}
            result, pixels = self.native.call(header, rgb["rgb"] + depth["depth"],
                                              settings["quality"]["request_timeout_sec"])
            try:
                if (set(result) != {"state", "generation", "width", "height",
                                    "candidates", "rejected"}
                        or result["generation"] != header["generation"]
                        or (result["width"], result["height"]) != (rgb["width"], rgb["height"])
                        or len(pixels) != len(rgb["rgb"])
                        or type(result["candidates"]) is not list
                        or type(result["rejected"]) is not list
                        or len(result["candidates"]) + len(result["rejected"]) != 1):
                    raise RuntimeError("Malformed selected pose response")
                validate_candidates(result, settings)
                for entry in result["candidates"] + result["rejected"]:
                    if entry["source_index"] != detection["source_index"]:
                        raise RuntimeError("Selected pose identity changed")
                if result["rejected"] and (type(result["rejected"][0]["reason"]) is not str
                                           or not result["rejected"][0]["reason"]):
                    raise RuntimeError("Selected rejection has no reason")
            except (KeyError, TypeError, ValueError, RuntimeError) as exc:
                self.native.failed = True
                self.native.close()
                self.events.record("FATAL", "item_selected_result_invalid", str(exc))
                raise RuntimeError(f"Malformed selected pose result: {exc}") from exc
            check_age()
            self._validate_sources()
            if observation["epoch"] != self.arm_epoch or not self.yolo_enabled:
                raise ValueError("Selection invalidated while calculating pose")
            return {"candidate": result["candidates"][0] if result["candidates"] else None,
                    "reason": result["rejected"][0]["reason"] if result["rejected"] else "",
                    "depth_rgb": pixels, "stamp_ns": rgb["stamp_ns"],
                    "epoch": observation["epoch"]}
        finally:
            self.operation_lock.release()

    def _request(self, request, response):
        if not self.request_lock.acquire(blocking=False):
            response.status, response.message = "BUSY", "One request is already active"
            return response
        epoch = self.arm_epoch
        acquired = False
        try:
            if self.service is None or not self.yolo_enabled:
                raise ValueError("Detector is not armed")
            if request.profile_sha256 != self.profile_digest:
                raise ValueError("Requested item profile SHA-256 mismatch")
            if not 1 <= request.max_candidates <= self.retry_limit:
                raise ValueError("Requested count exceeds taught total candidate-attempt limit")
            start_ns = self.get_clock().now().nanoseconds
            deadline = time.monotonic() + self.settings["quality"]["request_timeout_sec"]
            acquired = self.operation_lock.acquire(timeout=max(0.001, deadline - time.monotonic()))
            if not acquired:
                raise ValueError("Request deadline reached waiting for preview")
            self._validate_sources()
            _profile, digest = load_item_profile(self.profile_path)
            if digest != self.profile_digest:
                self.disarm()
                raise ValueError("Armed item profile changed")
            rgb, depth, context = self._snapshot(start_ns, deadline, wait=True)
            view = self.infer(rgb, depth, context, timeout=max(0.001, deadline-time.monotonic()))
            result = view["metadata"]
            if epoch != self.arm_epoch or self.service is None or not self.yolo_enabled:
                raise ValueError("Detector was disarmed or settings changed during request")
            self._validate_sources()
            if file_sha256(self.profile_path) != self.profile_digest:
                self.disarm()
                raise ValueError("Item profile changed during request")
            age = (self.get_clock().now().nanoseconds -
                   min(rgb["stamp_ns"], depth["stamp_ns"])) / 1e9
            if not 0 <= age <= self.settings["quality"]["result_max_age_sec"]:
                raise ValueError(f"Observation expired during processing: {age:.3f}s")
            if time.monotonic() > deadline:
                raise ValueError("Request deadline exceeded")
            response.batch_id = uuid.uuid4().hex
            response.header.frame_id = "platform_reference"
            response.header.stamp = Time(nanoseconds=rgb["stamp_ns"]).to_msg()
            response.depth_stamp = Time(nanoseconds=depth["stamp_ns"]).to_msg()
            response.detected_count = result["count"]
            response.valid_count = len(result["candidates"])
            for index, value in enumerate(result["candidates"][:request.max_candidates]):
                candidate = ItemCandidate()
                candidate.id = f"{response.batch_id}:{value['source_index']}"
                candidate.priority = index + 1
                for field in ("class_id", "class_name", "confidence", "length", "width",
                              "center_distance", "filtered_camera_depth", "depth_sigma",
                              "accepted_depth_count", "rejected_depth_count"):
                    setattr(candidate, field, value[field])
                position = candidate.pose.position
                position.x, position.y, position.z = value["position"]
                q = candidate.pose.orientation
                q.x, q.y, q.z, q.w = value["quaternion"]
                response.candidates.append(candidate)
            response.success = True
            if not response.candidates:
                response.status = "NO_VALID_ITEMS"
            elif len(response.candidates) < request.max_candidates:
                response.status = "SHORTAGE"
            else:
                response.status = "OK"
            response.message = (f"Returned {len(response.candidates)} of "
                                f"{request.max_candidates} requested")
            evidence = {"profile_sha256": self.profile_digest,
                        "model_sha256": self.model_config["sha256"],
                        "camera_sha256": self.applied.camera.sha256,
                        "platform_sha256": self.applied.platform.sha256,
                        "bin_sha256": self.bin_artifact.sha256, "snapshot_context": context,
                        "rejected": result["rejected"], "inference_ms": result["inference_ms"]}
            response.diagnostics_json = json.dumps(evidence, allow_nan=False)
            if self.preview_mode == "filtered":
                self.last_view = view
            self.events.record("INFO", "item_pose_batch", response.message,
                               batch_id=response.batch_id, status=response.status,
                               evidence=evidence,
                               candidates=result["candidates"][:request.max_candidates])
        except Exception as exc:
            response.success, response.status, response.message = False, "ERROR", str(exc)
            response.candidates = []
            self.events.record("ERROR", "item_pose_request_failed", str(exc))
            if self.native.failed:
                self.disarm()
                self.fatal_error = str(exc)
                self.get_logger().fatal(str(exc))
        finally:
            if acquired:
                self.operation_lock.release()
            self.request_lock.release()
        return response

    def close_runtime(self):
        self.disarm()
        self.yolo_enabled = False
        self.native.close()


def main(args=None):
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("ROS_LOCALHOST_ONLY=1 is required")
    rclpy.init(args=args)
    node = ItemDetectNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        paths = {key: node.declare_parameter(key, "").value for key in
                 ("item_teach_file", "platform_teach_file", "bin_teach_file")}
        if not all(paths.values()):
            raise ValueError("All three explicit teaching artifact paths are required")
        if not node.declare_parameter("trusted_model", False).value:
            raise ValueError(
                "Set trusted_model:=true only for a trusted .pt; weights can execute code")
        profile, _digest = load_item_profile(Path(paths["item_teach_file"]))
        node.inspect_model(Path(paths["item_teach_file"]).parent / profile["model"]["filename"])
        node.apply_station(paths["platform_teach_file"], paths["bin_teach_file"])
        node.enable_yolo(detection_settings(settings_from_profile(profile)))
        if not node.declare_parameter("armed", False).value:
            raise ValueError("Headless service requires explicit armed:=true; never inferred")
        # Startup has a bounded input-readiness deadline; no retry/restart.
        node._snapshot(0, time.monotonic() +
                       node.settings["quality"]["request_timeout_sec"], wait=True)
        node.arm(Path(paths["item_teach_file"]))
        while rclpy.ok() and node.fatal_error is None:
            time.sleep(0.1)
        if node.fatal_error:
            raise RuntimeError(node.fatal_error)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.events.record("FATAL", "item_detector_failed", str(exc))
        node.get_logger().fatal(str(exc))
        raise
    finally:
        node.close_runtime()
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
