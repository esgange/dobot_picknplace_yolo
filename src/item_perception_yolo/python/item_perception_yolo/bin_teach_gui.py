import math
import os
import signal
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from geometry_msgs.msg import TransformStamped
from python_qt_binding import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)

from camera_calibration_gui.opencv_worker import (
    ArucoMarkerSettings,
    OpenCvWorkerClient,
    OpenCvWorkerFailure,
    OpenCvWorkerOperationError,
    REQUIRED_ARUCO_API,
)

from .teach_ui import visual_teach_layout, update_teach_feedback, paint_teach_gate
from .bin_teach_core import (
    PLATFORM_FRAME,
    TARGET_MAX_AGE_SEC,
    ROI_BORDER_SAMPLES_PER_EDGE,
    ROI_GEOMETRY_METHOD,
    AppliedBinTeachCalibration,
    BinArucoSettings,
    BinTeachCapture,
    BinTeachArtifact,
    bin_output_path,
    bin_border_in_optical,
    bin_teach_directory,
    compose_platform_from_optical,
    load_bin_teach_calibration_context,
    load_bin_teach,
    load_bin_ui_state,
    place_bin_roi,
    select_outside_roi_corners,
    project_marker_rays_to_plane,
    validate_applied_sources,
    write_bin_state,
    write_bin_teach,
)
from .platform_teach_core import (
    BASE_FRAME,
    CAMERA_ON_HAND,
    CAMERA_TO_HAND,
    PackageEventLogger,
    ROBOT_TF_MAX_AGE_SEC,
    TOOL_FRAME,
    calibration_directory,
    resolve_base_from_camera_link,
    rotation_matrix_to_quaternion,
    ui_state_path,
)
from .ui_state import ARUCO_5X5_DICTIONARIES


EXECUTOR_THREAD_COUNT = 2
BIN_CORNER_FRAMES = tuple(f"bin_corner_{index}" for index in range(1, 5))
TF_PREVIEW_PERIOD_SEC = 0.1


def build_bin_preview_transforms(
    base_from_platform: np.ndarray,
    points,
    stamp,
) -> list[TransformStamped]:
    matrix = np.asarray(base_from_platform, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("Bin TF preview requires a finite 4x4 platform transform")
    points = tuple(points)
    if len(points) != len(BIN_CORNER_FRAMES):
        raise ValueError("Bin TF preview requires exactly four ROI points")

    platform = TransformStamped()
    platform.header.stamp = stamp
    platform.header.frame_id = BASE_FRAME
    platform.child_frame_id = PLATFORM_FRAME
    platform.transform.translation.x = float(matrix[0, 3])
    platform.transform.translation.y = float(matrix[1, 3])
    platform.transform.translation.z = float(matrix[2, 3])
    qx, qy, qz, qw = rotation_matrix_to_quaternion(matrix[:3, :3])
    platform.transform.rotation.x = qx
    platform.transform.rotation.y = qy
    platform.transform.rotation.z = qz
    platform.transform.rotation.w = qw
    transforms = [platform]

    for frame_name, point in zip(BIN_CORNER_FRAMES, points):
        x_m = float(point.x_m)
        y_m = float(point.y_m)
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            raise ValueError("Bin TF preview ROI points must be finite")
        corner = TransformStamped()
        corner.header.stamp = stamp
        corner.header.frame_id = PLATFORM_FRAME
        corner.child_frame_id = frame_name
        corner.transform.translation.x = x_m
        corner.transform.translation.y = y_m
        corner.transform.translation.z = 0.0
        corner.transform.rotation.x = 0.0
        corner.transform.rotation.y = 0.0
        corner.transform.rotation.z = 0.0
        corner.transform.rotation.w = 1.0
        transforms.append(corner)
    return transforms


def _create_executor(node: Node) -> MultiThreadedExecutor:
    executor = MultiThreadedExecutor(
        num_threads=EXECUTOR_THREAD_COUNT,
        context=node.context,
    )
    executor.add_node(node)
    return executor


def _quaternion_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    if not np.all(np.isfinite(quaternion)):
        raise ValueError("TF quaternion is not finite")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError("TF quaternion has zero length")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [
                1.0 - (2.0 * ((y * y) + (z * z))),
                2.0 * ((x * y) - (z * w)),
                2.0 * ((x * z) + (y * w)),
            ],
            [
                2.0 * ((x * y) + (z * w)),
                1.0 - (2.0 * ((x * x) + (z * z))),
                2.0 * ((y * z) - (x * w)),
            ],
            [
                2.0 * ((x * z) - (y * w)),
                2.0 * ((y * z) + (x * w)),
                1.0 - (2.0 * ((x * x) + (y * y))),
            ],
        ],
        dtype=np.float64,
    )


def _transform_message_to_matrix(transform) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = _quaternion_to_rotation(
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    )
    translation = np.asarray(
        [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(translation)):
        raise ValueError("TF translation is not finite")
    matrix[:3, 3] = translation
    return matrix


def _rgb8_image_from_message(message: Image) -> np.ndarray:
    if message.encoding != "rgb8":
        raise ValueError(
            f"Color image encoding is {message.encoding!r}; required exact 'rgb8'."
        )
    if int(message.is_bigendian) != 0:
        raise ValueError("Color image is_bigendian must be exactly 0")
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    if height <= 0 or width <= 0:
        raise ValueError("Color image dimensions must be positive")
    expected_step = width * 3
    if step != expected_step:
        raise ValueError(
            f"Color image step is {step}; required exactly width*3={expected_step}"
        )
    try:
        flat = np.frombuffer(memoryview(message.data), dtype=np.uint8)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Color image data is not a contiguous byte buffer: {exc}") from exc
    expected_size = height * step
    if flat.size != expected_size:
        raise ValueError(
            f"Color image contains {flat.size} bytes; required exactly {expected_size}"
        )
    return np.array(
        flat.reshape(height, width, 3),
        dtype=np.uint8,
        order="C",
        copy=True,
    )


class BinTeachNode(Node):
    def __init__(self) -> None:
        super().__init__("bin_teach")
        self._event_logger = PackageEventLogger(node_name="bin_teach")
        try:
            self._opencv_worker = OpenCvWorkerClient()
        except OpenCvWorkerFailure as exc:
            self._event_logger.record(
                "ERROR",
                "opencv_worker_start_failed",
                str(exc),
                operation=exc.operation,
                worker_pid=exc.worker_pid,
                exit_code=exc.exit_code,
                signal=exc.signal_name,
            )
            raise
        if self._opencv_worker.runtime.get("bin_teach_aruco_api") != REQUIRED_ARUCO_API:
            actual = self._opencv_worker.runtime.get("bin_teach_aruco_api")
            message = (
                "Private OpenCV worker ArUco API mismatch: "
                f"required={REQUIRED_ARUCO_API!r}, actual={actual!r}"
            )
            self._event_logger.record(
                "ERROR",
                "opencv_worker_start_failed",
                message,
                worker_pid=self._opencv_worker.pid,
                required_aruco_api=REQUIRED_ARUCO_API,
                actual_aruco_api=actual,
            )
            self._opencv_worker.close()
            raise RuntimeError(message)
        self._event_logger.record(
            "INFO",
            "opencv_worker_started",
            "Started exact package-private OpenCV worker for bin teaching",
            worker_pid=self._opencv_worker.pid,
            **self._opencv_worker.runtime,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._lock = threading.RLock()
        self._color_subscription = None
        self._camera_info_subscription = None
        self._configuration_generation = 0
        self._applied: AppliedBinTeachCalibration | None = None
        self._aruco_settings: BinArucoSettings | None = None
        self._camera_matrix = None
        self._distortion = None
        self._camera_distortion_model = None
        self._camera_info_size = None
        self._latest_overlay = None
        self._latest_points = None
        self._latest_selected_pixels = None
        self._latest_planar_border_pixels = None
        self._latest_ready_time = None
        self._latest_color_stamp = None
        self._latest_base_from_tool = None
        self._latest_robot_tf_stamp = None
        self._latest_base_from_camera_link = None
        self._latest_camera_link_from_optical = None
        self._latest_platform_from_optical = None
        self._latest_detected_ids = ()
        self._detection_status = "Select a platform calibration and apply ArUco settings."
        self._last_detection_event = None
        self._frame_sequence = 0
        self._last_frame_metadata = None
        self._capture: BinTeachCapture | None = None
        self._loaded_bin: BinTeachArtifact | None = None
        self._loaded_border_pixels = None
        self._loaded_border_time = None
        self._loaded_border_stamp_ns = None
        self._loaded_border_robot_stamp_ns = None
        self._loaded_border_status = "Waiting for a valid RGB frame."
        self._saved_path: Path | None = None
        self._fatal_error = None
        self._preview_timer = self.create_timer(
            TF_PREVIEW_PERIOD_SEC,
            self._rebroadcast_bin_preview,
        )
        self._event_logger.record(
            "INFO",
            "node_started",
            "Bin teach GUI started; waiting for explicit platform/settings Apply",
            local_only=True,
            platform_frame=PLATFORM_FRAME,
            required_marker_ids=[0, 1, 2, 3],
            corner_selection="outside",
            preview_frames=list(BIN_CORNER_FRAMES),
            executor_thread_count=EXECUTOR_THREAD_COUNT,
        )

    def load_last_session(self):
        path = ui_state_path()
        try:
            state = load_bin_ui_state(path)
        except ValueError as exc:
            message = f"Invalid bin-teach last-session state: {exc}"
            self._event_logger.record(
                "ERROR",
                "ui_state_load_failed",
                message,
                ui_state_path=str(path),
            )
            raise RuntimeError(message) from exc
        if state is None:
            self._event_logger.record(
                "INFO",
                "ui_state_absent",
                "No prior bin-teach UI state; using blank unapplied fields",
                ui_state_path=str(path),
            )
            return None
        self._event_logger.record(
            "INFO",
            "ui_state_loaded",
            "Restored unapplied bin-teach settings",
            platform_calibration_filename=state.platform_calibration_filename,
            aruco_dictionary=state.dictionary_name,
            marker_size_mm=state.marker_size_mm,
        )
        return state

    def _destroy_camera_subscriptions(self) -> None:
        if self._color_subscription is not None:
            self.destroy_subscription(self._color_subscription)
            self._color_subscription = None
        if self._camera_info_subscription is not None:
            self.destroy_subscription(self._camera_info_subscription)
            self._camera_info_subscription = None

    def _clear_runtime_state(self) -> None:
        self._camera_matrix = None
        self._distortion = None
        self._camera_distortion_model = None
        self._camera_info_size = None
        self._latest_overlay = None
        self._latest_points = None
        self._latest_selected_pixels = None
        self._latest_planar_border_pixels = None
        self._latest_ready_time = None
        self._latest_color_stamp = None
        self._latest_base_from_tool = None
        self._latest_robot_tf_stamp = None
        self._latest_base_from_camera_link = None
        self._latest_camera_link_from_optical = None
        self._latest_platform_from_optical = None
        self._latest_detected_ids = ()
        self._last_detection_event = None
        self._frame_sequence = 0
        self._last_frame_metadata = None
        self._capture = None
        self._loaded_bin = None
        self._loaded_border_pixels = None
        self._loaded_border_time = None
        self._loaded_border_stamp_ns = None
        self._loaded_border_robot_stamp_ns = None
        self._loaded_border_status = "Waiting for a valid RGB frame."
        self._saved_path = None

    def apply_settings(
        self,
        platform_path: Path,
        dictionary_name: str,
        marker_size_mm: float,
    ) -> str:
        aruco_settings = BinArucoSettings(dictionary_name, marker_size_mm)
        aruco_settings.validate()
        applied = load_bin_teach_calibration_context(platform_path)
        self._destroy_camera_subscriptions()
        with self._lock:
            self._configuration_generation += 1
            self._applied = None
            self._aruco_settings = None
            self._clear_runtime_state()
        try:
            self._opencv_worker.configure_aruco(
                ArucoMarkerSettings(
                    dictionary_name=aruco_settings.dictionary_name,
                    marker_size_mm=aruco_settings.marker_size_mm,
                )
            )
        except (OpenCvWorkerFailure, OpenCvWorkerOperationError) as exc:
            self._hard_fail_worker(exc)
            raise RuntimeError(str(exc)) from exc
        camera_settings = applied.camera.settings
        with self._lock:
            self._applied = applied
            self._aruco_settings = aruco_settings
            self._detection_status = (
                f"Waiting for {camera_settings.color_topic} and "
                f"{camera_settings.camera_info_topic}."
            )
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            camera_settings.camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self._color_subscription = self.create_subscription(
            Image,
            camera_settings.color_topic,
            self._on_color_image,
            qos_profile_sensor_data,
        )
        try:
            write_bin_state(
                ui_state_path(),
                applied.platform.path.name,
                aruco_settings.dictionary_name,
                aruco_settings.marker_size_mm,
            )
        except (OSError, ValueError) as exc:
            self._destroy_camera_subscriptions()
            with self._lock:
                self._applied = None
                self._aruco_settings = None
            raise RuntimeError(f"Failed to save bin-teach UI state: {exc}") from exc
        message = (
            f"Applied {applied.platform.path.name}; camera "
            f"{camera_settings.camera_prefix}; mode "
            f"{applied.camera.calibration_mode}; "
            f"{aruco_settings.dictionary_name}; "
            f"marker size {aruco_settings.marker_size_mm:.3f} mm."
        )
        self._event_logger.record(
            "INFO",
            "settings_applied",
            message,
            platform_calibration_file=str(applied.platform.path),
            platform_calibration_sha256=applied.platform.sha256,
            camera_calibration_file=str(applied.camera.path),
            camera_calibration_sha256=applied.camera.sha256,
            calibration_mode=applied.camera.calibration_mode,
            robot_tf_required=applied.camera.calibration_mode == CAMERA_ON_HAND,
            camera_prefix=camera_settings.camera_prefix,
            aruco_dictionary=aruco_settings.dictionary_name,
            marker_size_mm=aruco_settings.marker_size_mm,
        )
        return message

    def _skip_sensor_frame(
        self,
        event: str,
        message: str,
        *,
        generation: int,
        applied: AppliedBinTeachCalibration,
        clear_camera_info: bool = False,
        **fields,
    ) -> None:
        with self._lock:
            if generation != self._configuration_generation or self._applied is not applied:
                return
            if clear_camera_info:
                self._camera_matrix = None
                self._distortion = None
                self._camera_distortion_model = None
                self._camera_info_size = None
            self._loaded_border_pixels = None
            self._loaded_border_time = None
            self._loaded_border_status = message
            self._latest_points = None
            self._latest_selected_pixels = None
            self._latest_planar_border_pixels = None
            self._latest_ready_time = None
            self._latest_color_stamp = None
            self._latest_base_from_tool = None
            self._latest_robot_tf_stamp = None
            self._latest_base_from_camera_link = None
            self._latest_camera_link_from_optical = None
            self._latest_platform_from_optical = None
            self._detection_status = message
        self._event_logger.record("WARNING", event, message, **fields)

    def _on_camera_info(self, message: CameraInfo) -> None:
        with self._lock:
            applied = self._applied
            generation = self._configuration_generation
        if applied is None:
            return
        camera_settings = applied.camera.settings
        if message.header.frame_id != camera_settings.optical_frame:
            self._skip_sensor_frame(
                "camera_info_skipped",
                f"CameraInfo frame is {message.header.frame_id!r}; required "
                f"{camera_settings.optical_frame!r}.",
                generation=generation,
                applied=applied,
                clear_camera_info=True,
                reason="frame_id_mismatch",
            )
            return
        matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(message.d, dtype=np.float64)
        width = int(message.width)
        height = int(message.height)
        if (
            not np.all(np.isfinite(matrix))
            or matrix[0, 0] <= 0.0
            or matrix[1, 1] <= 0.0
            or width <= 0
            or height <= 0
            or distortion.size < 4
            or not np.all(np.isfinite(distortion))
        ):
            self._skip_sensor_frame(
                "camera_info_skipped",
                "CameraInfo must contain finite intrinsics and at least four "
                "distortion coefficients.",
                generation=generation,
                applied=applied,
                clear_camera_info=True,
                reason="invalid_intrinsics",
            )
            return
        with self._lock:
            if generation != self._configuration_generation or self._applied is not applied:
                return
            self._camera_matrix = matrix
            self._distortion = distortion
            self._camera_distortion_model = message.distortion_model
            self._camera_info_size = (height, width)

    def _set_detection_status(
        self,
        state: str,
        message: str,
        *,
        generation: int,
        applied: AppliedBinTeachCalibration,
        **fields,
    ) -> None:
        with self._lock:
            if generation != self._configuration_generation or self._applied is not applied:
                return
            self._detection_status = message
            changed = state != self._last_detection_event
            if changed:
                self._last_detection_event = state
        if changed:
            self._event_logger.record(
                "INFO" if state in ("ready", "loaded_border_ready") else "WARNING",
                f"target_{state}",
                message,
                **fields,
            )

    def _lookup_fresh_base_from_tool(self) -> tuple[np.ndarray, int, int, float]:
        try:
            transform = self._tf_buffer.lookup_transform(
                BASE_FRAME,
                TOOL_FRAME,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            raise ValueError(
                f"Required robot TF {BASE_FRAME} <- {TOOL_FRAME} is unavailable: {exc}"
            ) from exc
        stamp = Time.from_msg(transform.header.stamp)
        if stamp.nanoseconds == 0:
            raise ValueError(
                f"Robot TF {BASE_FRAME} <- {TOOL_FRAME} has a zero timestamp"
            )
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if not math.isfinite(age) or age < 0.0 or age > ROBOT_TF_MAX_AGE_SEC:
            raise ValueError(
                f"Robot TF {BASE_FRAME} <- {TOOL_FRAME} is stale or invalid: "
                f"age={age:.3f}s, maximum={ROBOT_TF_MAX_AGE_SEC:.3f}s"
            )
        return (
            _transform_message_to_matrix(transform.transform),
            int(transform.header.stamp.sec),
            int(transform.header.stamp.nanosec),
            age,
        )

    def _on_color_image(self, message: Image) -> None:
        with self._lock:
            applied = self._applied
            generation = self._configuration_generation
            camera_matrix = None if self._camera_matrix is None else self._camera_matrix.copy()
            distortion = None if self._distortion is None else self._distortion.copy()
            distortion_model = self._camera_distortion_model
            template = self._loaded_bin
            camera_info_size = self._camera_info_size
            fatal_error = self._fatal_error
        if applied is None or fatal_error is not None:
            return
        camera_settings = applied.camera.settings
        if message.header.frame_id != camera_settings.optical_frame:
            self._skip_sensor_frame(
                "color_frame_skipped",
                f"Color image frame is {message.header.frame_id!r}; required "
                f"{camera_settings.optical_frame!r}.",
                generation=generation,
                applied=applied,
                reason="frame_id_mismatch",
            )
            return
        try:
            image_rgb = _rgb8_image_from_message(message)
            stamp = Time.from_msg(message.header.stamp)
            if stamp.nanoseconds == 0:
                raise ValueError("Color image timestamp must be non-zero")
        except ValueError as exc:
            self._skip_sensor_frame(
                "color_frame_skipped",
                f"Color image contract failed: {exc}",
                generation=generation,
                applied=applied,
                reason="message_contract",
                frame_id=message.header.frame_id,
                width=int(message.width),
                height=int(message.height),
                encoding=message.encoding,
                step=int(message.step),
            )
            return
        if camera_matrix is None or distortion is None:
            with self._lock:
                if generation != self._configuration_generation or self._applied is not applied:
                    return
                self._latest_overlay = image_rgb
            self._skip_sensor_frame(
                "waiting_camera_info",
                "Color stream is live; waiting for valid CameraInfo.",
                generation=generation,
                applied=applied,
            )
            return
        if camera_info_size != image_rgb.shape[:2]:
            with self._lock:
                if generation != self._configuration_generation or self._applied is not applied:
                    return
                self._latest_overlay = image_rgb
            self._skip_sensor_frame(
                "color_camera_info_pair_skipped",
                "CameraInfo dimensions do not match color: "
                f"camera_info={camera_info_size}, color={image_rgb.shape[:2]}.",
                generation=generation,
                applied=applied,
                reason="dimension_mismatch",
            )
            return
        with self._lock:
            if generation != self._configuration_generation or self._applied is not applied:
                return
            self._frame_sequence += 1
            frame_sequence = self._frame_sequence
            self._last_frame_metadata = {
                "sequence": frame_sequence,
                "width": int(message.width),
                "height": int(message.height),
                "encoding": message.encoding,
                "step": int(message.step),
                "color_stamp_ns": stamp.nanoseconds,
            }
        if template is not None:
            self._render_loaded_border(
                image_rgb, stamp, camera_matrix, distortion, distortion_model,
                template=template, applied=applied, generation=generation,
            )
            return
        try:
            result = self._opencv_worker.detect_aruco(
                frame_sequence=frame_sequence,
                image_rgb=image_rgb,
                camera_matrix=camera_matrix,
                distortion=distortion,
            )
        except (OpenCvWorkerFailure, OpenCvWorkerOperationError) as exc:
            self._hard_fail_worker(exc)
            return
        with self._lock:
            if generation != self._configuration_generation or self._applied is not applied:
                return
            self._latest_overlay = result.overlay_rgb.copy()
            self._latest_detected_ids = result.detected_ids
        if result.state != "ready":
            with self._lock:
                self._latest_points = None
                self._latest_selected_pixels = None
                self._latest_planar_border_pixels = None
                self._latest_ready_time = None
                self._latest_color_stamp = None
                self._latest_base_from_tool = None
                self._latest_robot_tf_stamp = None
                self._latest_base_from_camera_link = None
                self._latest_camera_link_from_optical = None
                self._latest_platform_from_optical = None
            self._set_detection_status(
                result.state,
                result.message,
                generation=generation,
                applied=applied,
                detected_ids=list(result.detected_ids),
            )
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                camera_settings.camera_link_frame,
                camera_settings.optical_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
            camera_link_from_optical = _transform_message_to_matrix(transform.transform)
            base_from_tool = None
            robot_tf_stamp = None
            robot_tf_age = None
            if applied.camera.calibration_mode == CAMERA_ON_HAND:
                (
                    base_from_tool,
                    robot_tf_stamp_sec,
                    robot_tf_stamp_nanosec,
                    robot_tf_age,
                ) = self._lookup_fresh_base_from_tool()
                robot_tf_stamp = (robot_tf_stamp_sec, robot_tf_stamp_nanosec)
            base_from_camera_link = resolve_base_from_camera_link(
                applied.camera,
                base_from_tool,
            )
            platform_from_optical = compose_platform_from_optical(
                applied.platform.base_from_platform,
                base_from_camera_link,
                camera_link_from_optical,
            )
            if distortion_model not in {"plumb_bob", "rational_polynomial"}:
                raise ValueError(
                    f"Unsupported RGB projection distortion model: {distortion_model!r}"
                )
            image_points = np.concatenate([o.image_corners_px for o in result.observations])
            rays = self._opencv_worker.image_rays(
                image_points=image_points, camera_matrix=camera_matrix, distortion=distortion,
            )
            platform_corners = project_marker_rays_to_plane(
                platform_from_optical,
                {o.marker_id: rays[index * 4:(index + 1) * 4]
                 for index, o in enumerate(result.observations)},
            )
            points = select_outside_roi_corners(platform_corners)
            # Preview the exact saved-plane border with the same projection as Load.
            projected_border = self._opencv_worker.project_points(
                optical_points=bin_border_in_optical(points, platform_from_optical),
                camera_matrix=camera_matrix, distortion=distortion,
            )
            projected_corners = projected_border[::ROI_BORDER_SAMPLES_PER_EDGE]
            observations = {
                observation.marker_id: observation for observation in result.observations
            }
            detected_pixels = np.array([
                observations[p.source_marker_id].image_corners_px[p.source_marker_corner_index]
                for p in points
            ])
            roundtrip_error = np.linalg.norm(projected_corners - detected_pixels, axis=1)
            selected_pixels = tuple((float(pixel[0]), float(pixel[1]), point.source_marker_id)
                                    for point, pixel in zip(points, projected_corners))
        except (OpenCvWorkerFailure, OpenCvWorkerOperationError) as exc:
            self._hard_fail_worker(exc)
            return
        except TransformException as exc:
            message_text = (
                "Required camera-internal TF "
                f"{camera_settings.camera_link_frame} <- {camera_settings.optical_frame} "
                f"is unavailable: {exc}"
            )
            self._skip_sensor_frame(
                "camera_internal_tf_unavailable",
                message_text,
                generation=generation,
                applied=applied,
            )
            return
        except ValueError as exc:
            message_text = f"Bin ROI geometry is blocked: {exc}"
            self._skip_sensor_frame(
                "roi_geometry_rejected",
                message_text,
                generation=generation,
                applied=applied,
            )
            return
        with self._lock:
            if generation != self._configuration_generation or self._applied is not applied:
                return
            self._latest_points = points
            self._latest_selected_pixels = selected_pixels
            self._latest_planar_border_pixels = projected_border
            self._latest_ready_time = time.monotonic()
            self._latest_color_stamp = (
                int(message.header.stamp.sec),
                int(message.header.stamp.nanosec),
            )
            self._latest_base_from_tool = (
                None if base_from_tool is None else base_from_tool.copy()
            )
            self._latest_robot_tf_stamp = robot_tf_stamp
            self._latest_base_from_camera_link = base_from_camera_link.copy()
            self._latest_camera_link_from_optical = camera_link_from_optical.copy()
            self._latest_platform_from_optical = platform_from_optical.copy()
        self._set_detection_status(
            "ready",
            "IDs 0,1,2,3 ready; outside corners intersect platform Z=0. "
            "Yellow border previews the saved-plane ROI.",
            generation=generation,
            applied=applied,
            detected_ids=list(result.detected_ids),
            calibration_mode=applied.camera.calibration_mode,
            robot_tf_required=applied.camera.calibration_mode == CAMERA_ON_HAND,
            robot_tf_age_sec=robot_tf_age,
            roi_points=[
                {"x_m": point.x_m, "y_m": point.y_m}
                for point in points
            ],
            roi_geometry_method=ROI_GEOMETRY_METHOD,
            corner_roundtrip_error_px=roundtrip_error.tolist(),
        )

    def _render_loaded_border(
        self, image_rgb, stamp, camera_matrix, distortion, distortion_model,
        *, template, applied, generation,
    ) -> None:
        with self._lock:
            if generation != self._configuration_generation or self._loaded_bin is not template:
                return
            self._latest_overlay = image_rgb
            self._loaded_border_pixels = None
            self._loaded_border_status = "Projecting Loaded Bin Teach border."
        try:
            image_age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if not 0.0 <= image_age <= TARGET_MAX_AGE_SEC:
                raise ValueError(f"RGB frame is stale or invalid: age={image_age:.3f}s")
            if distortion_model not in {"plumb_bob", "rational_polynomial"}:
                raise ValueError(
                    f"Unsupported RGB projection distortion model: {distortion_model!r}"
                )
            camera = applied.camera.settings
            transform = self._tf_buffer.lookup_transform(
                camera.camera_link_frame, camera.optical_frame, Time(),
                timeout=Duration(seconds=0.2),
            )
            camera_link_from_optical = _transform_message_to_matrix(transform.transform)
            base_from_tool = None
            robot_stamp_ns = None
            if applied.camera.calibration_mode == CAMERA_ON_HAND:
                base_from_tool, sec, nanosec, _age = self._lookup_fresh_base_from_tool()
                robot_stamp_ns = sec * 1_000_000_000 + nanosec
            platform_from_optical = compose_platform_from_optical(
                applied.platform.base_from_platform,
                resolve_base_from_camera_link(applied.camera, base_from_tool),
                camera_link_from_optical,
            )
            optical_points = bin_border_in_optical(template.points, platform_from_optical)
        except (ValueError, TransformException) as exc:
            self._skip_sensor_frame(
                "loaded_bin_border_unavailable", f"Loaded border hidden: {exc}",
                generation=generation, applied=applied,
            )
            return
        try:
            pixels = self._opencv_worker.project_points(
                optical_points=optical_points, camera_matrix=camera_matrix,
                distortion=distortion,
            )
        except (OpenCvWorkerFailure, OpenCvWorkerOperationError) as exc:
            self._hard_fail_worker(exc)
            return
        with self._lock:
            if (
                generation != self._configuration_generation
                or self._loaded_bin is not template
                or self._fatal_error is not None
            ):
                return
            self._loaded_border_pixels = pixels
            self._loaded_border_time = time.monotonic()
            self._loaded_border_stamp_ns = stamp.nanoseconds
            self._loaded_border_robot_stamp_ns = robot_stamp_ns
            self._loaded_border_status = "Green border: Loaded Bin Teach."
        self._set_detection_status(
            "loaded_border_ready", "Loaded Bin Teach border projected onto live RGB.",
            generation=generation, applied=applied,
            template_filename=template.path.name,
        )

    def _hard_fail_worker(self, error: Exception) -> None:
        with self._lock:
            if self._fatal_error is not None:
                return
            metadata = dict(self._last_frame_metadata or {})
            self._capture = None
            self._loaded_bin = None
            self._loaded_border_pixels = None
            self._latest_points = None
            self._latest_ready_time = None
            self._fatal_error = str(error)
            self._detection_status = str(error)
        fields = {"frame_metadata": metadata}
        if isinstance(error, OpenCvWorkerFailure):
            fields.update(
                operation=error.operation,
                worker_pid=error.worker_pid,
                exit_code=error.exit_code,
                signal=error.signal_name,
            )
        self._event_logger.record(
            "ERROR",
            "opencv_worker_terminal_failure",
            str(error),
            **fields,
        )
        self.get_logger().fatal(str(error))

    def status_snapshot(self) -> dict:
        if self._fatal_error is None:
            try:
                self._opencv_worker.check_health()
            except (OpenCvWorkerFailure, OpenCvWorkerOperationError) as exc:
                self._hard_fail_worker(exc)
        with self._lock:
            target_ready = False
            gate = self._detection_status
            if self._applied is not None and self._latest_ready_time is not None:
                age = time.monotonic() - self._latest_ready_time
                if age <= TARGET_MAX_AGE_SEC:
                    robot_tf_block = None
                    if self._applied.camera.calibration_mode == CAMERA_ON_HAND:
                        if self._latest_robot_tf_stamp is None:
                            robot_tf_block = (
                                f"Required robot TF {BASE_FRAME} <- {TOOL_FRAME} "
                                "has not been captured."
                            )
                        else:
                            sec, nanosec = self._latest_robot_tf_stamp
                            robot_stamp_ns = (sec * 1_000_000_000) + nanosec
                            robot_age = (
                                self.get_clock().now().nanoseconds - robot_stamp_ns
                            ) / 1e9
                            if (
                                not math.isfinite(robot_age)
                                or robot_age < 0.0
                                or robot_age > ROBOT_TF_MAX_AGE_SEC
                            ):
                                robot_tf_block = (
                                    f"Robot TF {BASE_FRAME} <- {TOOL_FRAME} is stale "
                                    f"or invalid: age={robot_age:.3f}s, maximum="
                                    f"{ROBOT_TF_MAX_AGE_SEC:.3f}s."
                                )
                    if robot_tf_block is None:
                        target_ready = True
                        gate = (
                            f"READY: exact IDs 0,1,2,3; ROI age {age:.3f}s. "
                            "Capture while all markers remain still."
                        )
                    else:
                        gate = robot_tf_block
                else:
                    gate = (
                        f"ArUco ROI is stale ({age:.3f}s > "
                        f"{TARGET_MAX_AGE_SEC:.3f}s)."
                    )
            border = self._loaded_border_pixels
            border_status = self._loaded_border_status
            if border is not None:
                now_ns = self.get_clock().now().nanoseconds
                image_age = (now_ns - self._loaded_border_stamp_ns) / 1e9
                if (
                    time.monotonic() - self._loaded_border_time > TARGET_MAX_AGE_SEC
                    or not 0.0 <= image_age <= TARGET_MAX_AGE_SEC
                ):
                    border = None
                    border_status = "Loaded border hidden: RGB frame is stale."
                elif self._loaded_border_robot_stamp_ns is not None:
                    robot_age = (now_ns - self._loaded_border_robot_stamp_ns) / 1e9
                    if not 0.0 <= robot_age <= ROBOT_TF_MAX_AGE_SEC:
                        border = None
                        border_status = "Loaded border hidden: robot TF is stale."
            return {
                "configured": self._applied is not None,
                "target_ready": target_ready,
                "gate": gate,
                "has_capture": self._capture is not None,
                "has_preview": self._capture is not None or self._loaded_bin is not None,
                "loaded_bin": self._loaded_bin,
                "loaded_border_pixels": border,
                "loaded_border_status": border_status,
                "overlay": None if self._latest_overlay is None else self._latest_overlay.copy(),
                "saved_path": self._saved_path,
                "fatal_error": self._fatal_error,
                "applied": self._applied,
                "aruco_settings": self._aruco_settings,
                "capture": self._capture,
                "selected_pixels": self._latest_selected_pixels,
                "planar_border_pixels": self._latest_planar_border_pixels,
                "detected_ids": self._latest_detected_ids,
            }

    def latest_overlay(self):
        with self._lock:
            return None if self._latest_overlay is None else self._latest_overlay.copy()

    def capture_bin_roi(self) -> tuple[bool, str]:
        with self._lock:
            if self._fatal_error is not None:
                return False, self._fatal_error
            if self._capture is not None or self._loaded_bin is not None:
                return False, "Select Retake before capturing a new bin ROI."
            applied = self._applied
            points = self._latest_points
            ready_time = self._latest_ready_time
            color_stamp = self._latest_color_stamp
            frame_sequence = self._frame_sequence
            base_from_tool = (
                None
                if self._latest_base_from_tool is None
                else self._latest_base_from_tool.copy()
            )
            robot_tf_stamp = self._latest_robot_tf_stamp
            base_from_camera_link = (
                None
                if self._latest_base_from_camera_link is None
                else self._latest_base_from_camera_link.copy()
            )
            camera_link_from_optical = (
                None
                if self._latest_camera_link_from_optical is None
                else self._latest_camera_link_from_optical.copy()
            )
            platform_from_optical = (
                None
                if self._latest_platform_from_optical is None
                else self._latest_platform_from_optical.copy()
            )
        if applied is None:
            return False, "Apply a platform calibration and ArUco settings first."
        if points is None or ready_time is None or color_stamp is None:
            return False, self._detection_status
        if (
            base_from_camera_link is None
            or camera_link_from_optical is None
            or platform_from_optical is None
        ):
            return False, "The current bin ROI transform chain is incomplete."
        if applied.camera.calibration_mode == CAMERA_ON_HAND and (
            base_from_tool is None or robot_tf_stamp is None
        ):
            return False, "The current camera-on-hand robot TF is incomplete."
        age = time.monotonic() - ready_time
        if age > TARGET_MAX_AGE_SEC:
            return False, (
                f"ArUco ROI is stale ({age:.3f}s > {TARGET_MAX_AGE_SEC:.3f}s)."
            )
        if robot_tf_stamp is not None:
            robot_stamp_ns = (
                int(robot_tf_stamp[0]) * 1_000_000_000
                + int(robot_tf_stamp[1])
            )
            robot_tf_age = (
                self.get_clock().now().nanoseconds - robot_stamp_ns
            ) / 1e9
            if (
                not math.isfinite(robot_tf_age)
                or robot_tf_age < 0.0
                or robot_tf_age > ROBOT_TF_MAX_AGE_SEC
            ):
                return False, (
                    f"Robot TF {BASE_FRAME} <- {TOOL_FRAME} is stale or invalid: "
                    f"age={robot_tf_age:.3f}s, "
                    f"maximum={ROBOT_TF_MAX_AGE_SEC:.3f}s"
                )
        try:
            validate_applied_sources(applied)
        except ValueError as exc:
            message = f"Bin ROI capture rejected: {exc}"
            self._event_logger.record("ERROR", "bin_roi_capture_rejected", message)
            return False, message
        capture = BinTeachCapture(
            captured_at_utc=datetime.now(timezone.utc)
            .isoformat(timespec="microseconds")
            .replace("+00:00", "Z"),
            color_stamp_sec=color_stamp[0],
            color_stamp_nanosec=color_stamp[1],
            frame_sequence=frame_sequence,
            calibration_mode=applied.camera.calibration_mode,
            calibration_reference_from_camera_link=(
                applied.camera.reference_from_camera_link.copy()
            ),
            base_from_tool=base_from_tool,
            robot_tf_stamp_sec=(
                None if robot_tf_stamp is None else robot_tf_stamp[0]
            ),
            robot_tf_stamp_nanosec=(
                None if robot_tf_stamp is None else robot_tf_stamp[1]
            ),
            base_from_camera_link=base_from_camera_link,
            camera_link_from_optical=camera_link_from_optical,
            platform_from_optical=platform_from_optical,
            points=points,
        )
        with self._lock:
            self._capture = capture
            self._saved_path = None
        point_text = "; ".join(
            f"P{index + 1}=({point.x_m:+.4f},{point.y_m:+.4f})"
            for index, point in enumerate(points)
        )
        message = (
            f"Captured four-point bin ROI in {PLATFORM_FRAME}: {point_text}. "
            "RViz TF preview started."
        )
        self._event_logger.record(
            "INFO",
            "bin_roi_captured",
            message,
            frame_sequence=frame_sequence,
            calibration_mode=applied.camera.calibration_mode,
            robot_tf_required=applied.camera.calibration_mode == CAMERA_ON_HAND,
            tf_preview_parent=PLATFORM_FRAME,
            tf_preview_frames=list(BIN_CORNER_FRAMES),
            tf_preview_z_m=0.0,
            roi_geometry_method=ROI_GEOMETRY_METHOD,
            points=[
                {
                    "x_m": point.x_m,
                    "y_m": point.y_m,
                    "marker_id": point.source_marker_id,
                    "marker_corner_index": point.source_marker_corner_index,
                }
                for point in points
            ],
        )
        return True, message

    def load_bin_roi(self, path: Path) -> str:
        with self._lock:
            if self._fatal_error is not None:
                raise ValueError(self._fatal_error)
            applied = self._applied
            generation = self._configuration_generation
        if applied is None:
            raise ValueError("Apply this station's platform calibration before loading a bin ROI.")
        try:
            validate_applied_sources(applied)
            template = load_bin_teach(path)
            # Only the destination platform places the portable XY geometry.
            base_points = place_bin_roi(template, applied.platform)
        except (OSError, ValueError) as exc:
            self._event_logger.record("ERROR", "bin_roi_load_rejected", str(exc))
            raise
        with self._lock:
            if (
                self._fatal_error is not None
                or generation != self._configuration_generation
                or self._applied is not applied
            ):
                raise ValueError("Bin-teach settings changed while loading the ROI.")
            self._capture = None
            self._saved_path = None
            self._loaded_bin = template
            self._configuration_generation += 1
            self._latest_overlay = None
            self._latest_selected_pixels = None
            self._latest_planar_border_pixels = None
            self._loaded_border_pixels = None
            self._loaded_border_time = None
            self._loaded_border_status = "Waiting for a valid RGB frame."
        message = (
            f"Loaded {template.path.name} onto {applied.platform.path.name}. "
            "Its border will appear green on the live RGB view as Loaded Bin Teach. "
            "Retake to teach a new ROI; loading does not create a new capture."
        )
        self._event_logger.record(
            "INFO", "bin_roi_loaded", message,
            template_filename=template.path.name,
            template_sha256=template.sha256,
            source_robot_lan1_ip=template.source_robot_lan1_ip,
            target_robot_lan1_ip=applied.platform.robot_lan1_ip,
            target_platform_filename=applied.platform.path.name,
            target_platform_sha256=applied.platform.sha256,
            preview_points_in_base_m=base_points.tolist(),
        )
        return message

    def retake(self) -> tuple[bool, str]:
        with self._lock:
            if self._capture is None and self._loaded_bin is None:
                return False, "No captured or loaded bin ROI exists to clear."
            self._capture = None
            self._loaded_bin = None
            self._saved_path = None
            self._configuration_generation += 1
            self._latest_overlay = None
            self._latest_selected_pixels = None
            self._latest_planar_border_pixels = None
            self._latest_points = None
            self._latest_ready_time = None
            self._loaded_border_pixels = None
            self._loaded_border_time = None
        message = "Cleared the captured/loaded bin ROI; RViz TF preview stopped."
        self._event_logger.record("INFO", "bin_roi_capture_cleared", message)
        return True, message

    def save(self) -> tuple[bool, str, Path | None]:
        with self._lock:
            applied = self._applied
            aruco_settings = self._aruco_settings
            capture = self._capture
        if applied is None or aruco_settings is None or capture is None:
            return False, "Capture a valid bin ROI before saving.", None
        try:
            captured_at = datetime.fromisoformat(capture.captured_at_utc[:-1] + "+00:00")
            path = bin_output_path(
                applied.platform.robot_lan1_ip,
                created_at=captured_at,
            )
            write_bin_teach(path, applied, aruco_settings, capture)
        except (OSError, ValueError) as exc:
            message = f"Failed to save bin teach: {exc}"
            self._event_logger.record("ERROR", "bin_teach_save_failed", message)
            return False, message, None
        with self._lock:
            self._saved_path = path
        message = f"Saved bin teach: {path}"
        self._event_logger.record(
            "INFO",
            "bin_teach_saved",
            message,
            output_path=str(path),
            platform_calibration_file=str(applied.platform.path),
            platform_calibration_sha256=applied.platform.sha256,
        )
        return True, message, path

    def _rebroadcast_bin_preview(self) -> None:
        with self._lock:
            applied = self._applied
            capture = self._capture
            template = self._loaded_bin
            if applied is None or (capture is None and template is None):
                return
            points = capture.points if capture is not None else template.points
            transforms = build_bin_preview_transforms(
                applied.platform.base_from_platform,
                points,
                self.get_clock().now().to_msg(),
            )
            self._tf_broadcaster.sendTransform(transforms)

    def fatal_error(self):
        with self._lock:
            return self._fatal_error

    def close(self) -> None:
        with self._lock:
            self._capture = None
            self._loaded_bin = None
            self._preview_timer.cancel()
        self._destroy_camera_subscriptions()
        self._opencv_worker.close()
        self._event_logger.record("INFO", "node_stopped", "Bin teach GUI stopped")


class BinTeachWindow(QtWidgets.QWidget):
    def __init__(self, node: BinTeachNode) -> None:
        super().__init__()
        self._node = node
        self._fatal_shutdown_requested = False
        self.setWindowTitle("Dobot Bin Teach")
        self.resize(1320, 760)
        self.setMinimumSize(1000, 620)

        self.platform_path = QtWidgets.QLineEdit()
        self.platform_path.setPlaceholderText(
            "Select platform_calibration_<timestamp>_<robot_ip>.yaml"
        )
        self.browse_button = QtWidgets.QPushButton("Browse")
        self.dictionary = QtWidgets.QComboBox()
        self.dictionary.addItem("Select 5x5 dictionary", "")
        for dictionary_name in ARUCO_5X5_DICTIONARIES:
            self.dictionary.addItem(dictionary_name, dictionary_name)
        self.marker_size = QtWidgets.QLineEdit()
        self.marker_size.setPlaceholderText("Enter physical marker size in mm")
        self.apply_button = QtWidgets.QPushButton("Apply Settings")
        self.details = QtWidgets.QLabel("No platform calibration applied.")
        self.details.setWordWrap(True)
        self.capture_button = QtWidgets.QPushButton("Capture Bin ROI")
        self.load_button = QtWidgets.QPushButton("Load Bin ROI")
        self.retake_button = QtWidgets.QPushButton("Retake")
        self.save_button = QtWidgets.QPushButton("Save YAML")
        self.status = QtWidgets.QLabel(
            "Select a platform calibration and enter the exact 5x5 marker settings."
        )
        self.status.setWordWrap(True)
        self.status.setMinimumHeight(105)
        self.output = QtWidgets.QLabel(
            f"Output: {bin_teach_directory()}/bin_teach_"
            "<UTC_TIMESTAMP>_<ROBOT_LAN1_IP>.yaml"
        )
        self.output.setWordWrap(True)
        self.video = QtWidgets.QLabel("Waiting for applied bin-teach settings.")
        self.video.setAlignment(QtCore.Qt.AlignCenter)
        self.video.setStyleSheet("background: #101010; color: white;")
        self.video.setMinimumSize(600, 450)

        path_row = QtWidgets.QHBoxLayout()
        path_row.addWidget(self.platform_path, 1)
        path_row.addWidget(self.browse_button)
        form = QtWidgets.QFormLayout()
        form.setRowWrapPolicy(QtWidgets.QFormLayout.WrapLongRows)
        form.setFieldGrowthPolicy(QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
        form.addRow(QtWidgets.QLabel("Platform calibration"))
        form.addRow(path_row)
        form.addRow("ArUco dictionary", self.dictionary)
        form.addRow("Marker size [mm]", self.marker_size)
        controls = QtWidgets.QVBoxLayout()
        controls.addLayout(form)
        controls.addWidget(self.apply_button)
        guidance = QtWidgets.QLabel(
            "Place 5x5 markers ID 0, 1, 2 and 3 at the four bin corners. "
            "Their positions and rotations may be in any order. Bin Teach selects "
            "each marker corner farthest from the four-marker center. "
            "Keep the platform board and corner markers approximately coplanar. "
            "The plane may tilt relative to the robot base. The yellow border "
            "previews the same platform Z=0 geometry that will be saved and loaded. "
            "Reuse requires the same bin size, board origin/axes, and bin offset. "
            "Absolute station height may differ. RViz previews are for teaching only."
        )
        guidance.setWordWrap(True)
        visual_teach_layout(self, "Bin Teach", "RGB / Yellow: teaching ROI · Green: loaded ROI",
                            controls, guidance,
                            [self.capture_button, self.retake_button, self.load_button],
                            "Use 5x5 IDs 0–3, in any order. Measure marker size exactly. "
                            "Markers must lie on the taught platform plane; preserve the "
                            "same origin, axes and bin offset at each station.")

        state = self._node.load_last_session()
        if state is not None:
            self.platform_path.setText(
                str(calibration_directory() / state.platform_calibration_filename)
            )
            index = self.dictionary.findData(state.dictionary_name)
            self.dictionary.setCurrentIndex(index)
            self.marker_size.setText(f"{state.marker_size_mm:.6g}")
            self.status.setText(
                "Restored previous bin-teach values as unapplied prefill. "
                "Select Apply Settings to validate and use them."
            )

        self.browse_button.clicked.connect(self._browse)
        self.apply_button.clicked.connect(self._apply)
        self.capture_button.clicked.connect(self._capture)
        self.load_button.clicked.connect(self._load)
        self.retake_button.clicked.connect(self._retake)
        self.save_button.clicked.connect(self._save)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(100)
        self._refresh()

    def _browse(self) -> None:
        path, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select platform calibration",
            str(calibration_directory()),
            "Platform calibration (platform_calibration_*.yaml)",
        )
        if path:
            self.platform_path.setText(path)

    def _apply(self) -> None:
        path_text = self.platform_path.text().strip()
        dictionary_name = str(self.dictionary.currentData() or "")
        marker_size_text = self.marker_size.text().strip()
        if not path_text or not dictionary_name or not marker_size_text:
            QtWidgets.QMessageBox.warning(
                self,
                "Complete settings required",
                "Select a platform calibration, a 5x5 dictionary, and enter marker size.",
            )
            return
        try:
            marker_size_mm = float(marker_size_text)
        except ValueError:
            QtWidgets.QMessageBox.warning(
                self,
                "Invalid marker size",
                "Marker size must be a numeric millimetre measurement.",
            )
            return
        if self._node.status_snapshot()["has_preview"]:
            answer = QtWidgets.QMessageBox.question(
                self,
                "Replace current bin ROI?",
                "Applying settings clears the captured/loaded ROI preview. Continue?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if answer != QtWidgets.QMessageBox.Yes:
                return
        try:
            message = self._node.apply_settings(
                Path(path_text),
                dictionary_name,
                marker_size_mm,
            )
        except (OSError, RuntimeError, ValueError) as exc:
            QtWidgets.QMessageBox.critical(self, "Settings rejected", str(exc))
            self.status.setText(f"ERROR: {exc}")
            return
        self.status.setText(message)

    def _capture(self) -> None:
        success, message = self._node.capture_bin_roi()
        self.status.setText(message)
        if not success:
            QtWidgets.QMessageBox.warning(self, "Capture blocked", message)

    def _retake(self) -> None:
        if not self._node.status_snapshot()["has_preview"]:
            return
        answer = QtWidgets.QMessageBox.question(
            self,
            "Clear bin ROI capture?",
            "Clear the captured/loaded four-point ROI and stop its TF preview?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return
        _success, message = self._node.retake()
        self.status.setText(message)

    def _load(self) -> None:
        snapshot = self._node.status_snapshot()
        if not snapshot["configured"] or snapshot["fatal_error"] is not None:
            return
        path, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select portable bin ROI",
            str(bin_teach_directory()),
            "Bin teach (bin_teach_*.yaml)",
        )
        if not path:
            return
        answer = QtWidgets.QMessageBox.question(
            self,
            "Load bin ROI on this station?",
            "Confirm the same bin size, platform board origin and axes, and bin "
            "placement relative to that origin. The board and corner markers "
            "must be approximately coplanar at this station.\n\n"
            "Use the applied station platform for the loaded XY points? "
            "This replaces any current capture/preview; it does not save a new file.",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return
        try:
            message = self._node.load_bin_roi(Path(path))
        except (OSError, ValueError) as exc:
            QtWidgets.QMessageBox.critical(self, "Bin ROI load rejected", str(exc))
            return
        self.status.setText(message)

    def _save(self) -> None:
        if not self._node.status_snapshot()["has_capture"]:
            QtWidgets.QMessageBox.warning(
                self,
                "Nothing to save",
                "Capture a valid four-point bin ROI first.",
            )
            return
        answer = QtWidgets.QMessageBox.question(
            self,
            "Save bin teach?",
            "Save the four outside-corner XY points in platform_reference?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return
        success, message, path = self._node.save()
        self.status.setText(message)
        if success and path is not None:
            QtWidgets.QMessageBox.information(
                self,
                "Bin teach saved",
                f"Saved successfully:\n{path}",
            )
        else:
            QtWidgets.QMessageBox.critical(self, "Save failed", message)

    def _paint_roi_overlay(self, image: QtGui.QImage, snapshot: dict) -> None:
        if snapshot["loaded_bin"] is not None:
            painter = QtGui.QPainter(image)
            painter.setRenderHint(QtGui.QPainter.Antialiasing)
            painter.setClipRect(image.rect())
            green = QtGui.QColor(0, 255, 0)
            painter.setPen(QtGui.QPen(green, 4))
            painter.setBrush(QtCore.Qt.NoBrush)
            border = snapshot["loaded_border_pixels"]
            if border is not None:
                painter.drawPolygon(QtGui.QPolygonF([
                    QtCore.QPointF(float(x), float(y)) for x, y in border
                ]))
            font = QtGui.QFont("Sans Serif", max(14, image.width() // 85))
            font.setBold(True)
            painter.setFont(font)
            metrics = painter.fontMetrics()
            label = metrics.elidedText(
                f"Loaded Bin Teach | {snapshot['loaded_bin'].path.name}",
                QtCore.Qt.ElideMiddle, max(1, image.width() - 24),
            )
            lines = [label]
            if border is None:
                lines.append(metrics.elidedText(
                    snapshot["loaded_border_status"], QtCore.Qt.ElideRight,
                    max(1, image.width() - 24),
                ))
            painter.fillRect(
                6, 6, image.width() - 12, metrics.height() * len(lines) + 12,
                QtGui.QColor(0, 0, 0, 190),
            )
            painter.setPen(green)
            for index, text in enumerate(lines):
                painter.drawText(12, 12 + metrics.ascent() + index * metrics.height(), text)
            painter.end()
            return
        selected = snapshot["selected_pixels"]
        if selected is None:
            return
        painter = QtGui.QPainter(image)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        polygon = QtGui.QPolygonF(
            [QtCore.QPointF(float(x), float(y)) for x, y in snapshot["planar_border_pixels"]]
        )
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 220, 0), 4))
        painter.drawPolygon(polygon)
        font = QtGui.QFont("Sans Serif", max(11, image.width() // 120))
        font.setBold(True)
        painter.setFont(font)
        for index, (x, y, marker_id) in enumerate(selected):
            painter.setBrush(QtGui.QBrush(QtGui.QColor(255, 60, 60)))
            painter.drawEllipse(QtCore.QPointF(x, y), 7.0, 7.0)
            painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255), 2))
            painter.drawText(
                QtCore.QPointF(x + 9.0, y - 9.0),
                f"P{index + 1}/ID{marker_id}",
            )
            painter.setPen(QtGui.QPen(QtGui.QColor(255, 220, 0), 4))
        painter.end()

    def _refresh(self) -> None:
        snapshot = self._node.status_snapshot()
        fatal_error = snapshot["fatal_error"]
        if fatal_error is not None and not self._fatal_shutdown_requested:
            self._fatal_shutdown_requested = True
            self.status.setText(f"FATAL: {fatal_error}")
            QtCore.QTimer.singleShot(0, QtWidgets.QApplication.instance().quit)
        applied = snapshot["applied"]
        aruco_settings = snapshot["aruco_settings"]
        if applied is None or aruco_settings is None:
            self.details.setText("No platform calibration or ArUco settings applied.")
        else:
            camera = applied.camera.settings
            self.details.setText(
                f"Robot LAN1: {applied.platform.robot_lan1_ip}\n"
                f"Platform: {applied.platform.path.name}\n"
                f"Camera: {camera.camera_prefix}\n"
                f"Mode: {applied.camera.calibration_mode}\n"
                + (
                    f"TF: {BASE_FRAME} <- {camera.camera_link_frame} (fixed)\n"
                    if applied.camera.calibration_mode == CAMERA_TO_HAND
                    else f"TF: {BASE_FRAME} <- {TOOL_FRAME} (live) <- "
                    f"{camera.camera_link_frame}\n"
                )
                + f"ArUco: {aruco_settings.dictionary_name}, "
                f"{aruco_settings.marker_size_mm:.3f} mm, IDs 0-3"
            )
        capture = snapshot["capture"]
        template = snapshot["loaded_bin"]
        if template is not None:
            point_text = "\n".join(
                f"P{index + 1}: ({point.x_m:+.4f}, {point.y_m:+.4f}) m"
                for index, point in enumerate(template.points)
            )
            self.status.setText(
                f"LOADED: {template.path.name}\n{point_text}\n"
                f"{snapshot['loaded_border_status']}\n"
                "Border uses this station's platform plane (z=0).\n"
                "Retake to teach a new ROI. No new file is created by loading."
            )
        elif capture is not None:
            point_text = "\n".join(
                f"P{index + 1}: ({point.x_m:+.4f}, {point.y_m:+.4f}) m "
                f"from ID {point.source_marker_id} corner "
                f"{point.source_marker_corner_index}"
                for index, point in enumerate(capture.points)
            )
            saved = snapshot["saved_path"]
            self.status.setText(
                f"CAPTURED in {PLATFORM_FRAME}\n{point_text}"
                "\nRViz TF: base_link -> platform_reference -> "
                "bin_corner_1 ... bin_corner_4 (corner z=0 m)"
                + ("" if saved is None else f"\nSaved: {saved}")
            )
        elif applied is not None:
            self.status.setText(
                f"Target gate: {'READY' if snapshot['target_ready'] else 'BLOCKED'}\n"
                f"{snapshot['gate']}"
            )
        self.capture_button.setEnabled(snapshot["target_ready"] and not snapshot["has_preview"])
        self.retake_button.setEnabled(snapshot["has_preview"])
        self.load_button.setEnabled(snapshot["configured"] and fatal_error is None)
        self.save_button.setEnabled(capture is not None and snapshot["saved_path"] is None)
        update_teach_feedback(self)

        overlay = snapshot["overlay"]
        if overlay is None:
            self.video.setText(
                "Waiting for live RGB to show Loaded Bin Teach."
                if template is not None else "Waiting for a valid RGB frame."
            )
            return
        height, width, _channels = overlay.shape
        image = QtGui.QImage(
            overlay.data,
            width,
            height,
            int(overlay.strides[0]),
            QtGui.QImage.Format_RGB888,
        ).copy()
        self._paint_roi_overlay(image, snapshot)
        if template is None:
            paint_teach_gate(image, "CAPTURED / platform XY ROI" if capture is not None else
                             ("READY / " if snapshot["target_ready"] else "BLOCKED / ")
                             + snapshot["gate"])
        pixmap = QtGui.QPixmap.fromImage(image)
        self.video.setPixmap(
            pixmap.scaled(
                self.video.size(),
                QtCore.Qt.KeepAspectRatio,
                QtCore.Qt.SmoothTransformation,
            )
        )


def main(args=None) -> None:
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "bin_teach requires ROS_LOCALHOST_ONLY=1; "
            "source scripts/source_ros_workspace.bash first"
        )
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def request_shutdown(_signum, _frame) -> None:
        application.quit()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    node = None
    executor = None
    spin_thread = None
    try:
        node = BinTeachNode()
        executor = _create_executor(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        window = BinTeachWindow(node)
        window.show()
        application_exit_code = application.exec_()
        fatal_error = node.fatal_error()
        if fatal_error is not None:
            raise RuntimeError(fatal_error)
        if application_exit_code != 0:
            raise RuntimeError(f"Bin teach GUI exited with code {application_exit_code}")
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
        if spin_thread is not None:
            spin_thread.join(timeout=1.0)
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    main()
