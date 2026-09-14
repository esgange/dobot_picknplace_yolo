import os
import math
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
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from camera_calibration_gui.opencv_worker import (
    OpenCvWorkerClient,
    OpenCvWorkerFailure,
    OpenCvWorkerOperationError,
)

from .platform_teach_core import (
    BASE_FRAME,
    CAMERA_ON_HAND,
    CAMERA_TO_HAND,
    PLATFORM_FRAME,
    ROBOT_TF_MAX_AGE_SEC,
    TARGET_MAX_AGE_SEC,
    TOOL_FRAME,
    AppliedCameraCalibration,
    PackageEventLogger,
    PlatformCapture,
    calibration_directory,
    compose_platform_transform,
    load_camera_calibration,
    load_robot_lan1_ip,
    load_ui_state,
    platform_output_path,
    rotation_matrix_to_quaternion,
    resolve_base_from_camera_link,
    transform_summary,
    ui_state_path,
    write_platform_calibration,
    write_ui_state,
)
from .teach_ui import visual_teach_layout, update_teach_feedback, paint_teach_gate


EXECUTOR_THREAD_COUNT = 2


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


class PlatformTeachNode(Node):
    def __init__(self) -> None:
        super().__init__("platform_teach")
        self._event_logger = PackageEventLogger()
        self._robot_lan1_ip = load_robot_lan1_ip()
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
        self._event_logger.record(
            "INFO",
            "opencv_worker_started",
            "Started the exact package-private OpenCV worker for platform teaching",
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
        self._applied: AppliedCameraCalibration | None = None
        self._camera_matrix = None
        self._distortion = None
        self._camera_info_size = None
        self._latest_overlay = None
        self._latest_pose = None
        self._latest_pose_time = None
        self._latest_color_stamp = None
        self._latest_corner_count = 0
        self._detection_status = "Select and apply a camera calibration."
        self._last_detection_event = None
        self._frame_sequence = 0
        self._last_frame_metadata = None
        self._capture: PlatformCapture | None = None
        self._saved_path: Path | None = None
        self._fatal_error = None
        self._preview_timer = self.create_timer(0.1, self._rebroadcast_platform_preview)
        self._event_logger.record(
            "INFO",
            "node_started",
            "Platform teach GUI started; waiting for explicit calibration selection",
            robot_lan1_ip=self._robot_lan1_ip,
            local_only=True,
            target_type="charuco_board_origin",
            output_frame=PLATFORM_FRAME,
            executor_thread_count=EXECUTOR_THREAD_COUNT,
        )

    @property
    def robot_lan1_ip(self) -> str:
        return self._robot_lan1_ip

    def load_last_session(self):
        path = ui_state_path()
        try:
            state = load_ui_state(path)
        except ValueError as exc:
            message = f"Invalid platform-teach last-session state: {exc}"
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
                "No prior platform-teach UI state; using first-run blank selection",
                ui_state_path=str(path),
            )
            return None
        self._event_logger.record(
            "INFO",
            "ui_state_loaded",
            "Restored unapplied platform-teach calibration filename",
            ui_state_path=str(path),
            camera_calibration_filename=state.camera_calibration_filename,
        )
        return state

    def _destroy_camera_subscriptions(self) -> None:
        if self._color_subscription is not None:
            self.destroy_subscription(self._color_subscription)
            self._color_subscription = None
        if self._camera_info_subscription is not None:
            self.destroy_subscription(self._camera_info_subscription)
            self._camera_info_subscription = None

    def apply_camera_calibration(self, path: Path) -> str:
        calibration = load_camera_calibration(path)
        self._destroy_camera_subscriptions()
        with self._lock:
            self._configuration_generation += 1
            self._applied = None
            self._camera_matrix = None
            self._distortion = None
            self._camera_info_size = None
            self._latest_overlay = None
            self._latest_pose = None
            self._latest_pose_time = None
            self._latest_color_stamp = None
            self._latest_corner_count = 0
            self._capture = None
            self._saved_path = None
        try:
            self._opencv_worker.configure(calibration.settings)
        except (OpenCvWorkerFailure, OpenCvWorkerOperationError) as exc:
            self._hard_fail_worker(exc)
            raise RuntimeError(str(exc)) from exc

        settings = calibration.settings
        with self._lock:
            self._applied = calibration
            self._camera_matrix = None
            self._distortion = None
            self._camera_info_size = None
            self._latest_overlay = None
            self._latest_pose = None
            self._latest_pose_time = None
            self._latest_color_stamp = None
            self._latest_corner_count = 0
            self._detection_status = (
                f"Waiting for {settings.color_topic} and {settings.camera_info_topic}."
            )
            self._last_detection_event = None
            self._frame_sequence = 0
            self._last_frame_metadata = None
            self._capture = None
            self._saved_path = None

        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            settings.camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self._color_subscription = self.create_subscription(
            Image,
            settings.color_topic,
            self._on_color_image,
            qos_profile_sensor_data,
        )
        try:
            write_ui_state(ui_state_path(), calibration.path.name)
        except (OSError, ValueError) as exc:
            self._destroy_camera_subscriptions()
            with self._lock:
                self._applied = None
            raise RuntimeError(f"Failed to save platform-teach UI state: {exc}") from exc
        message = (
            f"Applied {calibration.path.name}: {calibration.reference_frame} <- "
            f"{settings.camera_link_frame}; mode {calibration.calibration_mode}; "
            f"ChArUco {settings.dictionary_name}, "
            f"{settings.squares_x}x{settings.squares_y}, "
            f"{settings.square_length_mm:.3f}/{settings.marker_length_mm:.3f} mm."
        )
        self._event_logger.record(
            "INFO",
            "camera_calibration_applied",
            message,
            camera_calibration_file=str(calibration.path),
            camera_calibration_sha256=calibration.sha256,
            camera_calibration_created_utc=calibration.created_at_utc,
            calibration_mode=calibration.calibration_mode,
            calibration_reference_frame=calibration.reference_frame,
            camera_prefix=settings.camera_prefix,
            color_topic=settings.color_topic,
            camera_info_topic=settings.camera_info_topic,
        )
        return message

    def _skip_sensor_frame(
        self,
        event: str,
        message: str,
        *,
        configuration_generation: int,
        applied_calibration: AppliedCameraCalibration,
        clear_camera_info: bool = False,
        **fields,
    ) -> None:
        with self._lock:
            if (
                configuration_generation != self._configuration_generation
                or self._applied is not applied_calibration
            ):
                return
            if clear_camera_info:
                self._camera_matrix = None
                self._distortion = None
                self._camera_info_size = None
            self._latest_pose = None
            self._latest_pose_time = None
            self._latest_color_stamp = None
            self._latest_corner_count = 0
            self._detection_status = message
        self._event_logger.record("WARNING", event, message, **fields)

    def _on_camera_info(self, message: CameraInfo) -> None:
        with self._lock:
            calibration = self._applied
            generation = self._configuration_generation
        if calibration is None:
            return
        settings = calibration.settings
        if message.header.frame_id != settings.optical_frame:
            self._skip_sensor_frame(
                "camera_info_skipped",
                f"CameraInfo frame is {message.header.frame_id!r}; "
                f"required {settings.optical_frame!r}.",
                configuration_generation=generation,
                applied_calibration=calibration,
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
                configuration_generation=generation,
                applied_calibration=calibration,
                clear_camera_info=True,
                reason="invalid_intrinsics",
            )
            return
        with self._lock:
            if (
                generation != self._configuration_generation
                or self._applied is not calibration
            ):
                return
            self._camera_matrix = matrix
            self._distortion = distortion
            self._camera_info_size = (height, width)

    def _set_detection_status(
        self,
        state: str,
        message: str,
        *,
        configuration_generation: int,
        applied_calibration: AppliedCameraCalibration,
        **fields,
    ) -> None:
        with self._lock:
            if (
                configuration_generation != self._configuration_generation
                or self._applied is not applied_calibration
            ):
                return
            self._detection_status = message
            changed = state != self._last_detection_event
            if changed:
                self._last_detection_event = state
        if changed:
            self._event_logger.record(
                "INFO" if state == "ready" else "WARNING",
                f"target_{state}",
                message,
                **fields,
            )

    def _on_color_image(self, message: Image) -> None:
        with self._lock:
            calibration = self._applied
            generation = self._configuration_generation
            camera_matrix = (
                None if self._camera_matrix is None else self._camera_matrix.copy()
            )
            distortion = None if self._distortion is None else self._distortion.copy()
            camera_info_size = self._camera_info_size
            fatal_error = self._fatal_error
        if calibration is None or fatal_error is not None:
            return
        settings = calibration.settings
        if message.header.frame_id != settings.optical_frame:
            self._skip_sensor_frame(
                "color_frame_skipped",
                f"Color image frame is {message.header.frame_id!r}; "
                f"required {settings.optical_frame!r}.",
                configuration_generation=generation,
                applied_calibration=calibration,
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
                configuration_generation=generation,
                applied_calibration=calibration,
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
                if (
                    generation != self._configuration_generation
                    or self._applied is not calibration
                ):
                    return
                self._latest_overlay = image_rgb
                self._latest_pose = None
                self._latest_pose_time = None
                self._latest_color_stamp = None
            self._set_detection_status(
                "waiting_camera_info",
                "Color stream is live; waiting for valid CameraInfo.",
                configuration_generation=generation,
                applied_calibration=calibration,
            )
            return
        if camera_info_size != image_rgb.shape[:2]:
            with self._lock:
                if (
                    generation != self._configuration_generation
                    or self._applied is not calibration
                ):
                    return
                self._latest_overlay = image_rgb
            self._skip_sensor_frame(
                "color_camera_info_pair_skipped",
                "CameraInfo dimensions do not match color: "
                f"camera_info={camera_info_size}, color={image_rgb.shape[:2]}.",
                configuration_generation=generation,
                applied_calibration=calibration,
                reason="dimension_mismatch",
            )
            return
        with self._lock:
            if (
                generation != self._configuration_generation
                or self._applied is not calibration
            ):
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
        try:
            result = self._opencv_worker.detect(
                frame_sequence=frame_sequence,
                image_rgb=image_rgb,
                camera_matrix=camera_matrix,
                distortion=distortion,
                solution_overlay=None,
            )
        except (OpenCvWorkerFailure, OpenCvWorkerOperationError) as exc:
            self._hard_fail_worker(exc)
            return
        with self._lock:
            if (
                generation != self._configuration_generation
                or self._applied is not calibration
            ):
                return
            self._latest_overlay = result.overlay_rgb.copy()
            self._latest_corner_count = result.corner_count
            if result.camera_from_target is None:
                self._latest_pose = None
                self._latest_pose_time = None
                self._latest_color_stamp = None
            else:
                self._latest_pose = result.camera_from_target.copy()
                self._latest_pose_time = time.monotonic()
                self._latest_color_stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        message = result.message
        if result.state == "ready":
            message = "RGB ChArUco board pose ready for platform capture."
        self._set_detection_status(
            result.state,
            message,
            configuration_generation=generation,
            applied_calibration=calibration,
            detected_corners=result.corner_count,
        )

    def _hard_fail_worker(self, error: Exception) -> None:
        with self._lock:
            if self._fatal_error is not None:
                return
            metadata = dict(self._last_frame_metadata or {})
            self._capture = None
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
            if self._applied is not None and self._latest_pose_time is not None:
                age = time.monotonic() - self._latest_pose_time
                if age <= TARGET_MAX_AGE_SEC:
                    target_ready = True
                    gate = (
                        f"READY: {self._latest_corner_count} ChArUco corners, "
                        f"age {age:.3f}s. Keep the board fixed and capture."
                    )
                    if self._applied.calibration_mode == CAMERA_ON_HAND:
                        gate += (
                            f" Capture will require fresh {BASE_FRAME} <- "
                            f"{TOOL_FRAME} TF."
                        )
                else:
                    gate = (
                        f"ChArUco pose is stale ({age:.3f}s > "
                        f"{TARGET_MAX_AGE_SEC:.3f}s)."
                    )
            return {
                "configured": self._applied is not None,
                "target_ready": target_ready,
                "gate": gate,
                "detection": self._detection_status,
                "has_capture": self._capture is not None,
                "saved_path": self._saved_path,
                "fatal_error": self._fatal_error,
                "calibration": self._applied,
                "capture": self._capture,
            }

    def latest_overlay(self):
        with self._lock:
            return None if self._latest_overlay is None else self._latest_overlay.copy()

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

    def capture_platform(self) -> tuple[bool, str]:
        with self._lock:
            if self._fatal_error is not None:
                return False, self._fatal_error
            calibration = self._applied
            pose = None if self._latest_pose is None else self._latest_pose.copy()
            pose_time = self._latest_pose_time
            color_stamp = self._latest_color_stamp
            corner_count = self._latest_corner_count
            frame_sequence = self._frame_sequence
        if calibration is None:
            return False, "Apply a camera calibration first."
        if pose is None or pose_time is None or color_stamp is None:
            return False, self._detection_status
        age = time.monotonic() - pose_time
        if age > TARGET_MAX_AGE_SEC:
            return False, (
                f"ChArUco pose is stale ({age:.3f}s > {TARGET_MAX_AGE_SEC:.3f}s)."
            )
        try:
            transform = self._tf_buffer.lookup_transform(
                calibration.settings.camera_link_frame,
                calibration.settings.optical_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            message = (
                "Required camera-internal TF "
                f"{calibration.settings.camera_link_frame} <- "
                f"{calibration.settings.optical_frame} is unavailable: {exc}"
            )
            self._event_logger.record("ERROR", "platform_capture_rejected", message)
            return False, message
        try:
            camera_link_from_optical = _transform_message_to_matrix(transform.transform)
            base_from_tool = None
            robot_tf_stamp_sec = None
            robot_tf_stamp_nanosec = None
            robot_tf_age = None
            if calibration.calibration_mode == CAMERA_ON_HAND:
                (
                    base_from_tool,
                    robot_tf_stamp_sec,
                    robot_tf_stamp_nanosec,
                    robot_tf_age,
                ) = self._lookup_fresh_base_from_tool()
            base_from_camera_link = resolve_base_from_camera_link(
                calibration,
                base_from_tool,
            )
            base_from_platform = compose_platform_transform(
                base_from_camera_link,
                camera_link_from_optical,
                pose,
            )
        except ValueError as exc:
            message = f"Platform transform composition failed: {exc}"
            self._event_logger.record("ERROR", "platform_capture_rejected", message)
            return False, message
        capture = PlatformCapture(
            captured_at_utc=datetime.now(timezone.utc)
            .isoformat(timespec="microseconds")
            .replace("+00:00", "Z"),
            color_stamp_sec=int(color_stamp[0]),
            color_stamp_nanosec=int(color_stamp[1]),
            frame_sequence=frame_sequence,
            charuco_corner_count=corner_count,
            calibration_mode=calibration.calibration_mode,
            calibration_reference_from_camera_link=(
                calibration.reference_from_camera_link.copy()
            ),
            base_from_tool=(
                None if base_from_tool is None else base_from_tool.copy()
            ),
            robot_tf_stamp_sec=robot_tf_stamp_sec,
            robot_tf_stamp_nanosec=robot_tf_stamp_nanosec,
            base_from_camera_link=base_from_camera_link,
            camera_link_from_optical=camera_link_from_optical,
            optical_from_board=pose,
            base_from_platform=base_from_platform,
        )
        with self._lock:
            self._capture = capture
            self._saved_path = None
        xyz, rpy = transform_summary(base_from_platform)
        message = f"Captured {BASE_FRAME} <- {PLATFORM_FRAME}. {xyz}; {rpy}."
        self._event_logger.record(
            "INFO",
            "platform_captured",
            message,
            frame_sequence=frame_sequence,
            charuco_corner_count=corner_count,
            camera_calibration_file=str(calibration.path),
            calibration_mode=calibration.calibration_mode,
            robot_tf_required=calibration.calibration_mode == CAMERA_ON_HAND,
            robot_tf_age_sec=robot_tf_age,
            robot_lan1_ip=self._robot_lan1_ip,
            xyz=xyz,
            rpy=rpy,
        )
        return True, message

    def retake(self) -> tuple[bool, str]:
        with self._lock:
            if self._capture is None:
                return False, "No platform capture exists to clear."
            self._capture = None
            self._saved_path = None
        message = "Cleared the captured platform transform; TF preview stopped."
        self._event_logger.record("INFO", "platform_capture_cleared", message)
        return True, message

    def save(self) -> tuple[bool, str, Path | None]:
        with self._lock:
            calibration = self._applied
            capture = self._capture
        if calibration is None or capture is None:
            return False, "Capture a valid platform transform before saving.", None
        try:
            captured_at = datetime.fromisoformat(
                capture.captured_at_utc[:-1] + "+00:00"
            )
            path = platform_output_path(
                self._robot_lan1_ip,
                created_at=captured_at,
            )
            write_platform_calibration(
                path,
                calibration,
                self._robot_lan1_ip,
                capture,
            )
        except (OSError, ValueError) as exc:
            message = f"Failed to save platform calibration: {exc}"
            self._event_logger.record(
                "ERROR",
                "platform_save_failed",
                message,
            )
            return False, message, None
        with self._lock:
            self._saved_path = path
        message = f"Saved platform calibration: {path}"
        self._event_logger.record(
            "INFO",
            "platform_saved",
            message,
            output_path=str(path),
            robot_lan1_ip=self._robot_lan1_ip,
            camera_calibration_file=str(calibration.path),
            camera_calibration_sha256=calibration.sha256,
        )
        return True, message, path

    def _rebroadcast_platform_preview(self) -> None:
        with self._lock:
            capture = self._capture
        if capture is None:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = BASE_FRAME
        transform.child_frame_id = PLATFORM_FRAME
        matrix = capture.base_from_platform
        transform.transform.translation.x = float(matrix[0, 3])
        transform.transform.translation.y = float(matrix[1, 3])
        transform.transform.translation.z = float(matrix[2, 3])
        qx, qy, qz, qw = rotation_matrix_to_quaternion(matrix[:3, :3])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)

    def fatal_error(self):
        with self._lock:
            return self._fatal_error

    def close(self) -> None:
        self._destroy_camera_subscriptions()
        self._opencv_worker.close()
        self._event_logger.record("INFO", "node_stopped", "Platform teach GUI stopped")


class PlatformTeachWindow(QtWidgets.QWidget):
    def __init__(self, node: PlatformTeachNode) -> None:
        super().__init__()
        self._node = node
        self._fatal_shutdown_requested = False
        self.setWindowTitle("Dobot Platform Teach")
        self.resize(1320, 760)
        self.setMinimumSize(1000, 620)

        self.calibration_path = QtWidgets.QLineEdit()
        self.calibration_path.setPlaceholderText(
            "Select camera_to_hand or camera_on_hand calibration YAML"
        )
        self.browse_button = QtWidgets.QPushButton("Browse")
        self.apply_button = QtWidgets.QPushButton("Apply Calibration")
        self.details = QtWidgets.QLabel("No camera calibration applied.")
        self.details.setWordWrap(True)
        self.capture_button = QtWidgets.QPushButton("Capture Platform")
        self.retake_button = QtWidgets.QPushButton("Retake")
        self.save_button = QtWidgets.QPushButton("Save YAML")
        self.status = QtWidgets.QLabel(
            "Select the calibration used by the active fixed or on-hand camera."
        )
        self.status.setWordWrap(True)
        self.status.setMinimumHeight(90)
        self.output = QtWidgets.QLabel(
            f"Output: {calibration_directory()}/platform_calibration_"
            "<UTC_TIMESTAMP>_<ROBOT_LAN1_IP>.yaml"
        )
        self.output.setWordWrap(True)
        self.video = QtWidgets.QLabel("Waiting for an applied camera calibration.")
        self.video.setAlignment(QtCore.Qt.AlignCenter)
        self.video.setStyleSheet("background: #101010; color: white;")
        self.video.setMinimumSize(600, 450)

        path_row = QtWidgets.QHBoxLayout()
        path_row.addWidget(self.calibration_path, 1)
        path_row.addWidget(self.browse_button)
        controls = QtWidgets.QVBoxLayout()
        controls.addWidget(QtWidgets.QLabel("Camera calibration"))
        controls.addLayout(path_row)
        controls.addWidget(self.apply_button)
        guidance = QtWidgets.QLabel(
            "Place the ChArUco board origin at the shared bin-mount corner. "
            "Use the same board-axis directions and bin offset at every station; "
            "a common corner alone does not define matching axes. Keep the board "
            "approximately coplanar with the bin-corner markers. Absolute height "
            "relative to the robot base may differ between stations. Keep the board "
            "and robot still for capture. Camera-on-hand requires fresh "
            "base_link <- Link6 TF. RViz is a teaching preview only."
        )
        guidance.setWordWrap(True)
        visual_teach_layout(self, "Platform Teach", "RGB / ChArUco axes & platform pose",
                            controls, guidance, [self.capture_button, self.retake_button],
                            "Board origin = shared mount corner. Match axis directions across "
                            "stations; keep board and robot still for capture.")

        state = self._node.load_last_session()
        if state is not None:
            self.calibration_path.setText(
                str(calibration_directory() / state.camera_calibration_filename)
            )
            self.status.setText(
                "Restored the previous calibration filename as unapplied prefill. "
                "Select Apply Calibration to validate and use it."
            )

        self.browse_button.clicked.connect(self._browse)
        self.apply_button.clicked.connect(self._apply)
        self.capture_button.clicked.connect(self._capture)
        self.retake_button.clicked.connect(self._retake)
        self.save_button.clicked.connect(self._save)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(100)
        self._refresh()

    def _browse(self) -> None:
        path, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select camera calibration",
            str(calibration_directory()),
            "YAML calibration (*.yaml)",
        )
        if path:
            self.calibration_path.setText(path)

    def _apply(self) -> None:
        path_text = self.calibration_path.text().strip()
        if not path_text:
            QtWidgets.QMessageBox.warning(
                self,
                "Camera calibration required",
                "Select a camera-to-hand or camera-on-hand calibration YAML first.",
            )
            return
        snapshot = self._node.status_snapshot()
        if snapshot["has_capture"]:
            answer = QtWidgets.QMessageBox.question(
                self,
                "Replace current platform capture?",
                "Applying another camera calibration clears the current platform "
                "capture and stops its TF preview. Continue?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if answer != QtWidgets.QMessageBox.Yes:
                return
        try:
            message = self._node.apply_camera_calibration(Path(path_text))
        except (OSError, RuntimeError, ValueError) as exc:
            QtWidgets.QMessageBox.critical(self, "Calibration rejected", str(exc))
            self.status.setText(f"ERROR: {exc}")
            return
        self.status.setText(message)

    def _capture(self) -> None:
        success, message = self._node.capture_platform()
        self.status.setText(message)
        if not success:
            QtWidgets.QMessageBox.warning(self, "Capture blocked", message)

    def _retake(self) -> None:
        snapshot = self._node.status_snapshot()
        if not snapshot["has_capture"]:
            return
        answer = QtWidgets.QMessageBox.question(
            self,
            "Clear platform capture?",
            "Clear the captured platform transform and stop its TF preview?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return
        _success, message = self._node.retake()
        self.status.setText(message)

    def _save(self) -> None:
        snapshot = self._node.status_snapshot()
        if not snapshot["has_capture"]:
            QtWidgets.QMessageBox.warning(
                self,
                "Nothing to save",
                "Capture a valid platform transform first.",
            )
            return
        answer = QtWidgets.QMessageBox.question(
            self,
            "Save platform calibration?",
            "Save the captured base_link <- platform_reference transform as a new "
            "station YAML artifact?\n\n"
            "Confirm the board origin and axes follow the shared bin-mount "
            "reference used by your other stations.",
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
                "Platform calibration saved",
                f"Saved successfully:\n{path}",
            )
        else:
            QtWidgets.QMessageBox.critical(self, "Save failed", message)

    def _paint_video_overlay(self, image: QtGui.QImage, snapshot: dict) -> None:
        capture = snapshot["capture"]
        if capture is None:
            if not snapshot["target_ready"]:
                return
            lines = ("RGB ChArUco board pose ready for platform capture.",)
            top = max(0, image.height() - 58)
        else:
            xyz, rpy = transform_summary(capture.base_from_platform)
            lines = (
                "PLATFORM TF CAPTURED",
                f"{BASE_FRAME} <- {PLATFORM_FRAME}",
                xyz,
                rpy,
            )
            top = 0
        painter = QtGui.QPainter(image)
        font = QtGui.QFont("Sans Serif", max(12, image.width() // 95))
        font.setBold(True)
        painter.setFont(font)
        metrics = QtGui.QFontMetrics(font)
        line_height = metrics.height() + 5
        width = min(
            image.width(),
            max(metrics.horizontalAdvance(line) for line in lines) + 20,
        )
        height = (line_height * len(lines)) + 16
        painter.fillRect(0, top, width, height, QtGui.QColor(0, 0, 0, 210))
        for index, line in enumerate(lines):
            painter.setPen(
                QtGui.QColor(80, 255, 80)
                if index == 0
                else QtGui.QColor(255, 255, 255)
            )
            painter.drawText(
                10,
                top + 8 + ((index + 1) * line_height) - 4,
                line,
            )
        painter.end()

    def _refresh(self) -> None:
        snapshot = self._node.status_snapshot()
        fatal_error = snapshot["fatal_error"]
        if fatal_error is not None and not self._fatal_shutdown_requested:
            self._fatal_shutdown_requested = True
            self.status.setText(f"FATAL: {fatal_error}")
            QtCore.QTimer.singleShot(0, QtWidgets.QApplication.instance().quit)
        calibration = snapshot["calibration"]
        if calibration is None:
            self.details.setText(
                f"Robot LAN1 identity: {self._node.robot_lan1_ip}\n"
                "No camera calibration applied."
            )
        else:
            settings = calibration.settings
            transform_chain = (
                f"{BASE_FRAME} <- {settings.camera_link_frame} (fixed calibration)"
                if calibration.calibration_mode == CAMERA_TO_HAND
                else f"{BASE_FRAME} <- {TOOL_FRAME} (live) <- "
                f"{settings.camera_link_frame} (on-hand calibration)"
            )
            self.details.setText(
                f"Robot LAN1 identity: {self._node.robot_lan1_ip}\n"
                f"Camera: {settings.camera_prefix}\n"
                f"Mode: {calibration.calibration_mode}\n"
                f"TF: {transform_chain}\n"
                f"Board: {settings.dictionary_name}, {settings.squares_x}x"
                f"{settings.squares_y}, {settings.square_length_mm:.3f} mm / "
                f"{settings.marker_length_mm:.3f} mm"
            )
        capture = snapshot["capture"]
        if capture is not None:
            xyz, rpy = transform_summary(capture.base_from_platform)
            saved = snapshot["saved_path"]
            self.status.setText(
                f"CAPTURED: {BASE_FRAME} <- {PLATFORM_FRAME}\n{xyz}\n{rpy}"
                + ("" if saved is None else f"\nSaved: {saved}")
            )
        elif calibration is not None:
            self.status.setText(
                f"Target gate: {'READY' if snapshot['target_ready'] else 'BLOCKED'}\n"
                f"{snapshot['gate']}"
            )
        self.capture_button.setEnabled(snapshot["target_ready"] and capture is None)
        self.retake_button.setEnabled(capture is not None)
        self.save_button.setEnabled(capture is not None and snapshot["saved_path"] is None)
        update_teach_feedback(self)

        overlay = self._node.latest_overlay()
        if overlay is None:
            self.video.setText(self.status.text())
            return
        height, width, _channels = overlay.shape
        image = QtGui.QImage(
            overlay.data,
            width,
            height,
            int(overlay.strides[0]),
            QtGui.QImage.Format_RGB888,
        ).copy()
        self._paint_video_overlay(image, snapshot)
        if capture is None:
            paint_teach_gate(image, ("READY / " if snapshot["target_ready"] else "BLOCKED / ")
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
            "platform_teach requires ROS_LOCALHOST_ONLY=1; "
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
        node = PlatformTeachNode()
        executor = _create_executor(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        window = PlatformTeachWindow(node)
        window.show()
        application_exit_code = application.exec_()
        fatal_error = node.fatal_error()
        if fatal_error is not None:
            raise RuntimeError(fatal_error)
        if application_exit_code != 0:
            raise RuntimeError(f"Platform teach GUI exited with code {application_exit_code}")
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
