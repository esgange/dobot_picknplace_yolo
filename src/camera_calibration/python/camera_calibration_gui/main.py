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
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .calibration_core import (
    BASE_FRAME,
    CAMERA_ON_HAND,
    CAMERA_TO_HAND,
    CHARUCO_DICTIONARIES,
    JOINT_NAMES,
    JOINT_STATE_TOPIC,
    LEAVE_ONE_OUT_MINIMUM_SAMPLES,
    MINIMUM_CALIBRATION_SAMPLES,
    MINIMUM_CHARUCO_CORNERS,
    MINIMUM_SAMPLE_ROTATION_DEG,
    MOVEIT_REFERENCE_COMMIT,
    TOOL_FRAME,
    UI_STATE_FILENAME,
    AccuracyDiagnostics,
    CalibrationArtifact,
    CalibrationSample,
    CalibrationUiState,
    CharucoSettings,
    PackageEventLogger,
    load_calibration_yaml,
    load_calibration_ui_state,
    output_path_for_mode,
    output_pattern_for_mode,
    reference_frame_for_mode,
    rotation_matrix_to_quaternion,
    sample_diagnostic_rows,
    sample_rotation_conflict,
    validate_calibration_mode,
    write_calibration_yaml,
    write_calibration_ui_state,
)
from . import opencv_worker as _opencv_worker
from .automatic_capture import AutomaticCapture


OpenCvWorkerClient = _opencv_worker.OpenCvWorkerClient
OpenCvWorkerFailure = _opencv_worker.OpenCvWorkerFailure
OpenCvWorkerOperationError = _opencv_worker.OpenCvWorkerOperationError
SOLUTION_OVERLAY_TEXT_THICKNESS = _opencv_worker.SOLUTION_OVERLAY_TEXT_THICKNESS
_solution_overlay_font_scale = _opencv_worker._solution_overlay_font_scale
_solution_overlay_lines = _opencv_worker._solution_overlay_lines


TARGET_MAX_AGE_SEC = 0.5
ROBOT_TF_MAX_AGE_SEC = 1.0
ROBOT_JOINT_STATE_MAX_AGE_SEC = 1.0
CALIBRATION_EXECUTOR_THREAD_COUNT = 2
COLOR_IMAGE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def _create_calibration_executor(node: Node) -> MultiThreadedExecutor:
    """Keep TF reception independent from the serialized sensor callbacks."""
    executor = MultiThreadedExecutor(
        num_threads=CALIBRATION_EXECUTOR_THREAD_COUNT,
        context=node.context,
    )
    executor.add_node(node)
    return executor


def _quaternion_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    if norm <= 1e-12:
        raise ValueError("TF quaternion has zero length")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
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
    matrix[:3, 3] = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]
    return matrix


def _rgb8_image_from_message(message: Image) -> np.ndarray:
    if message.encoding != "rgb8":
        raise ValueError(
            f"Color image encoding is {message.encoding!r}; required exact encoding 'rgb8'."
        )
    if int(message.is_bigendian) != 0:
        raise ValueError("Color image is_bigendian must be exactly 0")
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    if height <= 0 or width <= 0:
        raise ValueError(f"Color image dimensions must be positive; received {width}x{height}")
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


def _message_stamp_nanoseconds(message, label: str) -> int:
    stamp = Time.from_msg(message.header.stamp).nanoseconds
    if stamp == 0:
        raise ValueError(f"{label} timestamp must be non-zero")
    return stamp


def _metric_text(value: float | None, suffix: str) -> str:
    return "not available" if value is None else f"{value:.3f}{suffix}"


def _diagnostics_summary_text(
    diagnostics: AccuracyDiagnostics | None,
    calibration_count: int,
) -> str:
    lines = [f"Calibration samples: {calibration_count}"]
    if diagnostics is None:
        lines.append("Fit diagnostics: waiting for a valid automatic solution.")
        lines.append(
            f"Leave-one-out: available from {LEAVE_ONE_OUT_MINIMUM_SAMPLES} "
            "calibration samples."
        )
        return "\n".join(lines)

    ax_xb = diagnostics.ax_xb
    lines.append(
        f"AX=XB RMS: {ax_xb.translation_rms_mm:.3f}mm/"
        f"{ax_xb.rotation_rms_deg:.3f}deg ({ax_xb.pair_count} adjacent pose pairs)"
    )
    fit = diagnostics.fit
    lines.append(
        f"FIT RMS: {fit.translation_rms_mm:.3f}mm/{fit.rotation_rms_deg:.3f}deg | "
        f"Max T: {fit.max_translation_residual_mm:.3f}mm "
        f"({fit.max_translation_sample_id}) | Max R: "
        f"{fit.max_rotation_residual_deg:.3f}deg ({fit.max_rotation_sample_id})"
    )
    change = diagnostics.solution_change
    if change.available:
        lines.append(
            "Change from previous solve: FIT RMS "
            f"{change.translation_rms_delta_mm:+.3f}mm/"
            f"{change.rotation_rms_delta_deg:+.3f}deg | Camera TF "
            f"{change.camera_translation_delta_mm:.3f}mm/"
            f"{change.camera_rotation_delta_deg:.3f}deg"
        )
    else:
        lines.append("Change from previous solve: not available for the first solution.")

    leave_one_out = diagnostics.leave_one_out
    if leave_one_out.status == "not_available":
        lines.append(
            f"Leave-one-out: not available until {LEAVE_ONE_OUT_MINIMUM_SAMPLES} "
            "calibration samples."
        )
    elif leave_one_out.status == "failed":
        lines.append(
            "Leave-one-out: FAILED when omitting "
            f"{', '.join(leave_one_out.failed_sample_ids)}; YAML saving disabled."
        )
    else:
        lines.append(
            "Leave-one-out TF RMS: "
            f"{leave_one_out.translation_rms_mm:.3f}mm/"
            f"{leave_one_out.rotation_rms_deg:.3f}deg | Max T: "
            f"{leave_one_out.max_translation_delta_mm:.3f}mm "
            f"({leave_one_out.max_translation_sample_id}) | Max R: "
            f"{leave_one_out.max_rotation_delta_deg:.3f}deg "
            f"({leave_one_out.max_rotation_sample_id})"
        )
    coverage = diagnostics.pose_coverage
    lines.append(
        f"Robot-pose coverage: {coverage.translation_span_mm:.1f}mm / "
        f"{coverage.rotation_span_deg:.2f}deg maximum pairwise span."
    )
    return "\n".join(lines)


class CalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_calibration")
        self._event_logger = PackageEventLogger()
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
            "Started the isolated serial OpenCV worker",
            worker_pid=self._opencv_worker.pid,
            process_start_method="spawn",
            request_timeout_sec=5.0,
            **self._opencv_worker.runtime,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._lock = threading.RLock()
        self._color_subscription = None
        self._camera_info_subscription = None
        self._settings = None
        self._calibration_mode = None
        self._reference_frame = ""
        self._minimum_samples = MINIMUM_CALIBRATION_SAMPLES
        self._camera_matrix = None
        self._distortion = None
        self._camera_info_size = None
        self._latest_joint_positions = None
        self._latest_joint_state_stamp = None
        self._latest_overlay = None
        self._latest_pose = None
        self._latest_pose_time = None
        self._latest_pose_stamp = None
        self._latest_valid_rgb_stamp = None
        self._latest_corner_count = 0
        self._detection_status = "Enter settings and select Apply Settings."
        self._last_detection_event = None
        self._calibration_samples = []
        self._next_calibration_sample_number = 1
        self._solution = None
        self._diagnostics = None
        self._fatal_error = None
        self._frame_sequence = 0
        self._last_frame_metadata = None
        self._preview_timer = self.create_timer(0.5, self._rebroadcast_solution_preview)
        self._joint_state_subscription = self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.automatic = AutomaticCapture(self)
        self._event_logger.record(
            "INFO",
            "node_started",
            "camera calibration GUI started; waiting for explicit Apply Settings",
            local_only=True,
            target_type="charuco",
            executor_thread_count=CALIBRATION_EXECUTOR_THREAD_COUNT,
            tf_listener_execution="independent_reentrant_callback_group",
        )

    @property
    def ui_state_path(self):
        return self._event_logger.path.with_name(UI_STATE_FILENAME)

    def load_last_session(self) -> CalibrationUiState | None:
        try:
            state = load_calibration_ui_state(self.ui_state_path)
        except ValueError as exc:
            message = f"Invalid camera-calibration last-session state: {exc}"
            self._event_logger.record(
                "ERROR",
                "ui_state_load_failed",
                message,
                ui_state_path=str(self.ui_state_path),
            )
            raise RuntimeError(message) from exc
        if state is None:
            self._event_logger.record(
                "INFO",
                "ui_state_absent",
                "No prior camera-calibration UI state; using first-run blank fields",
                ui_state_path=str(self.ui_state_path),
            )
            return None
        self._event_logger.record(
            "INFO",
            "ui_state_loaded",
            f"Restored camera-calibration UI fields from {self.ui_state_path}",
            ui_state_path=str(self.ui_state_path),
            saved_at_utc=state.saved_at_utc,
            camera_prefix=state.settings.camera_prefix,
            calibration_mode=state.calibration_mode,
        )
        return state

    def save_last_session(
        self,
        settings: CharucoSettings,
        calibration_mode: str,
    ) -> CalibrationUiState:
        try:
            state = write_calibration_ui_state(
                self.ui_state_path,
                calibration_mode,
                settings,
            )
        except (OSError, ValueError) as exc:
            message = f"Failed to save camera-calibration last-session state: {exc}"
            self._event_logger.record(
                "ERROR",
                "ui_state_save_failed",
                message,
                ui_state_path=str(self.ui_state_path),
            )
            raise RuntimeError(message) from exc
        self._event_logger.record(
            "INFO",
            "ui_state_saved",
            f"Saved camera-calibration UI fields to {self.ui_state_path}",
            ui_state_path=str(self.ui_state_path),
            saved_at_utc=state.saved_at_utc,
            camera_prefix=settings.camera_prefix,
            calibration_mode=calibration_mode,
        )
        return state

    def configure(
        self,
        settings: CharucoSettings,
        calibration_mode: str,
    ) -> str:
        settings.validate()
        calibration_mode = validate_calibration_mode(calibration_mode)
        reference_frame = reference_frame_for_mode(calibration_mode)
        if hasattr(self, "automatic"):
            self.automatic.clear_recipe()

        if self._color_subscription is not None:
            self.destroy_subscription(self._color_subscription)
        if self._camera_info_subscription is not None:
            self.destroy_subscription(self._camera_info_subscription)

        try:
            self._opencv_worker.configure(settings)
        except (OpenCvWorkerFailure, OpenCvWorkerOperationError) as exc:
            self._hard_fail_opencv_worker(exc)
            raise RuntimeError(str(exc)) from exc

        with self._lock:
            self._settings = settings
            self._calibration_mode = calibration_mode
            self._reference_frame = reference_frame
            self._minimum_samples = MINIMUM_CALIBRATION_SAMPLES
            self._camera_matrix = None
            self._distortion = None
            self._camera_info_size = None
            self._latest_overlay = None
            self._latest_pose = None
            self._latest_pose_time = None
            self._latest_pose_stamp = None
            self._latest_valid_rgb_stamp = None
            self._latest_corner_count = 0
            self._detection_status = (
                f"Waiting for {settings.color_topic} and "
                f"{settings.camera_info_topic}."
            )
            self._last_detection_event = None
            self._calibration_samples.clear()
            self._next_calibration_sample_number = 1
            self._solution = None
            self._diagnostics = None

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
            COLOR_IMAGE_QOS,
        )
        message = (
            f"Applied mode={calibration_mode}, reference={reference_frame}, "
            f"prefix={settings.camera_prefix}, dictionary={settings.dictionary_name}, "
            f"board={settings.squares_x}x{settings.squares_y}, "
            f"checker={settings.square_length_mm:.3f}mm, marker={settings.marker_length_mm:.3f}mm."
        )
        self._event_logger.record(
            "INFO",
            "configuration_applied",
            message,
            camera_prefix=settings.camera_prefix,
            color_topic=settings.color_topic,
            camera_info_topic=settings.camera_info_topic,
            optical_frame=settings.optical_frame,
            dictionary=settings.dictionary_name,
            squares_x=settings.squares_x,
            squares_y=settings.squares_y,
            square_length_mm=settings.square_length_mm,
            marker_length_mm=settings.marker_length_mm,
            minimum_corners=MINIMUM_CHARUCO_CORNERS,
            minimum_sample_rotation_deg=MINIMUM_SAMPLE_ROTATION_DEG,
            moveit_reference_commit=MOVEIT_REFERENCE_COMMIT,
            minimum_samples=MINIMUM_CALIBRATION_SAMPLES,
            calibration_mode=calibration_mode,
            reference_frame=reference_frame,
            base_frame=BASE_FRAME,
            tool_frame=TOOL_FRAME,
        )
        return message

    def _set_fatal_error(self, event: str, message: str, **fields) -> None:
        with self._lock:
            if self._fatal_error is not None:
                return
            self._fatal_error = message
            self._latest_valid_rgb_stamp = None
            self._detection_status = message
            self._latest_pose = None
            self._latest_pose_time = None
            self._clear_solution_locked()
        self._event_logger.record(
            "ERROR",
            event,
            message,
            **fields,
        )
        self.get_logger().fatal(message)

    def _skip_sensor_frame(
        self,
        event: str,
        message: str,
        *,
        clear_camera_info: bool = False,
        **fields,
    ) -> None:
        """Reject one sensor message without terminating the calibration session."""
        with self._lock:
            if clear_camera_info:
                self._camera_matrix = None
                self._distortion = None
                self._camera_info_size = None
            self._latest_pose = None
            self._latest_pose_time = None
            self._detection_status = message
            self._latest_valid_rgb_stamp = None
            changed = event != self._last_detection_event
            if changed:
                self._last_detection_event = event
        if changed:
            self._event_logger.record(
                "WARNING",
                event,
                message,
                frame_skipped=True,
                next_valid_frame_required=True,
                **fields,
            )
            self.get_logger().warning(message)

    def _hard_fail_opencv_worker(self, exc: Exception) -> None:
        fields = {
            "operation": getattr(exc, "operation", "unknown"),
            "worker_pid": getattr(exc, "worker_pid", self._opencv_worker.pid),
            "exit_code": getattr(exc, "exit_code", self._opencv_worker.exit_code),
            "signal": getattr(exc, "signal_name", None),
            "last_frame": self._last_frame_metadata,
            "retry_attempted": False,
        }
        self._set_fatal_error(
            "opencv_worker_failed",
            f"OpenCV worker fatal failure: {exc}",
            **fields,
        )

    def fatal_error(self) -> str | None:
        with self._lock:
            return self._fatal_error

    def close_opencv_worker(self) -> None:
        worker = getattr(self, "_opencv_worker", None)
        if worker is not None:
            worker.close()

    def _set_detection_status(self, state: str, message: str, **fields) -> None:
        with self._lock:
            self._detection_status = message
            changed = state != self._last_detection_event
            if changed:
                self._last_detection_event = state
        if changed:
            level = "INFO" if state == "ready" else "WARNING"
            self._event_logger.record(level, f"target_{state}", message, **fields)

    def _store_latest_overlay(self, overlay: np.ndarray) -> None:
        with self._lock:
            self._latest_overlay = np.array(overlay, dtype=np.uint8, order="C", copy=True)

    def _on_camera_info(self, message: CameraInfo) -> None:
        with self._lock:
            settings = self._settings
        if settings is None:
            return
        if message.header.frame_id != settings.optical_frame:
            self._skip_sensor_frame(
                "camera_info_frame_skipped",
                f"CameraInfo frame is {message.header.frame_id!r}; "
                f"required {settings.optical_frame!r}.",
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
                "camera_info_frame_skipped",
                "CameraInfo must contain finite intrinsics and at least four "
                "distortion coefficients.",
                clear_camera_info=True,
                reason="invalid_intrinsics",
            )
            return
        with self._lock:
            self._camera_matrix = matrix
            self._distortion = distortion
            self._camera_info_size = (height, width)

    def _on_joint_state(self, message: JointState) -> None:
        rejection = None
        fields = {}
        if tuple(message.name) != JOINT_NAMES:
            rejection = (
                "JointState names must be exactly joint1 through joint6 "
                "in canonical order."
            )
            fields = {
                "actual_names": list(message.name),
                "required_names": list(JOINT_NAMES),
            }
        elif len(message.position) != len(JOINT_NAMES):
            rejection = "JointState must contain exactly six positions."
            fields = {
                "actual_position_count": len(message.position),
                "required_position_count": len(JOINT_NAMES),
            }
        else:
            positions = tuple(float(position) for position in message.position)
            if any(not math.isfinite(position) for position in positions):
                rejection = "JointState positions must all be finite."
            else:
                stamp = Time.from_msg(message.header.stamp)
                if stamp.nanoseconds == 0:
                    rejection = "JointState timestamp must be non-zero."

        if rejection is not None:
            with self._lock:
                self._latest_joint_positions = None
                self._latest_joint_state_stamp = None
            self._event_logger.record(
                "ERROR",
                "joint_state_rejected",
                rejection,
                **fields,
            )
            return

        with self._lock:
            self._latest_joint_positions = positions
            self._latest_joint_state_stamp = stamp

    def _on_color_image(self, message: Image) -> None:
        with self._lock:
            settings = self._settings
            camera_matrix = None if self._camera_matrix is None else self._camera_matrix.copy()
            distortion = None if self._distortion is None else self._distortion.copy()
            camera_info_size = self._camera_info_size
            solution = None if self._solution is None else self._solution.copy()
            diagnostics = self._diagnostics
            reference_frame = self._reference_frame
            fatal_error = self._fatal_error
        if settings is None or fatal_error is not None:
            return
        if message.header.frame_id != settings.optical_frame:
            self._skip_sensor_frame(
                "color_frame_skipped",
                f"Color image frame is {message.header.frame_id!r}; "
                f"required {settings.optical_frame!r}.",
                reason="frame_id_mismatch",
            )
            return
        try:
            image_rgb = _rgb8_image_from_message(message)
            color_stamp_ns = _message_stamp_nanoseconds(message, "Color image")
        except ValueError as exc:
            self._skip_sensor_frame(
                "color_frame_skipped",
                f"Color image contract failed: {exc}",
                reason="message_contract",
                frame_id=message.header.frame_id,
                width=int(message.width),
                height=int(message.height),
                encoding=message.encoding,
                step=int(message.step),
            )
            return

        if camera_matrix is None or distortion is None:
            self._store_latest_overlay(image_rgb)
            self._set_detection_status(
                "waiting_camera_info",
                "Color stream is live; waiting for valid CameraInfo.",
            )
            return
        if camera_info_size != image_rgb.shape[:2]:
            self._store_latest_overlay(image_rgb)
            self._skip_sensor_frame(
                "color_camera_info_pair_skipped",
                "CameraInfo dimensions do not match the color stream: "
                f"camera_info={camera_info_size}, color={image_rgb.shape[:2]}.",
                reason="dimension_mismatch",
            )
            return
        with self._lock:
            # Stream freshness describes validated input, independently of how
            # long the worker needs to detect/draw this particular board view.
            self._latest_valid_rgb_stamp = color_stamp_ns
            self._frame_sequence += 1
            frame_sequence = self._frame_sequence
            self._last_frame_metadata = {
                "sequence": frame_sequence,
                "width": int(message.width),
                "height": int(message.height),
                "encoding": message.encoding,
                "step": int(message.step),
                "color_stamp_ns": color_stamp_ns,
            }
        solution_overlay = None
        if solution is not None and isinstance(diagnostics, AccuracyDiagnostics):
            solution_overlay = {
                "reference_frame": reference_frame,
                "camera_link_frame": settings.camera_link_frame,
                "reference_from_camera_link": solution,
                "quality": diagnostics.fit,
            }
        try:
            result = self._opencv_worker.detect(
                frame_sequence=frame_sequence,
                image_rgb=image_rgb,
                camera_matrix=camera_matrix,
                distortion=distortion,
                solution_overlay=solution_overlay,
            )
        except (OpenCvWorkerFailure, OpenCvWorkerOperationError) as exc:
            self._hard_fail_opencv_worker(exc)
            return

        self._store_latest_overlay(result.overlay_rgb)
        if result.camera_from_target is None:
            with self._lock:
                self._latest_corner_count = result.corner_count
                self._latest_pose = None
                self._latest_pose_time = None
            self._set_detection_status(
                result.state,
                result.message,
                detected_corners=result.corner_count,
                required_corners=(
                    MINIMUM_CHARUCO_CORNERS
                    if result.state == "insufficient_corners"
                    else None
                ),
            )
            return
        now = time.monotonic()
        with self._lock:
            self._latest_pose = result.camera_from_target.copy()
            self._latest_pose_time = now
            self._latest_pose_stamp = color_stamp_ns
            self._latest_corner_count = result.corner_count
        self._set_detection_status(
            result.state,
            result.message,
            detected_corners=result.corner_count,
        )

    def _target_gate_locked(self) -> tuple[bool, str]:
        if getattr(self, "_fatal_error", None) is not None:
            return False, self._fatal_error
        if self._settings is None:
            return False, "Apply valid settings first."
        if self._latest_pose is None or self._latest_pose_time is None:
            return False, self._detection_status
        age = time.monotonic() - self._latest_pose_time
        if age > TARGET_MAX_AGE_SEC:
            return False, f"ChArUco pose is stale ({age:.3f}s > {TARGET_MAX_AGE_SEC:.3f}s)."
        return True, (
            f"READY: {self._latest_corner_count} corners, age {age:.3f}s. "
            "Hold the robot stationary for capture."
        )

    def status_snapshot(self) -> dict:
        if getattr(self, "_fatal_error", None) is None:
            try:
                self._opencv_worker.check_health()
            except (OpenCvWorkerFailure, OpenCvWorkerOperationError) as exc:
                self._hard_fail_opencv_worker(exc)
        with self._lock:
            ready, gate = self._target_gate_locked()
            diagnostic_rows = ()
            if isinstance(self._diagnostics, AccuracyDiagnostics):
                diagnostic_rows = sample_diagnostic_rows(self._diagnostics)
            row_by_id = {row.sample_id: row for row in diagnostic_rows}
            rows = []
            for sample in self._calibration_samples:
                diagnostic = row_by_id.get(sample.sample_id)
                rows.append(
                    {
                        "sample_id": sample.sample_id,
                        "translation_residual_mm": (
                            None
                            if diagnostic is None
                            else diagnostic.translation_residual_mm
                        ),
                        "rotation_residual_deg": (
                            None if diagnostic is None else diagnostic.rotation_residual_deg
                        ),
                        "leave_one_out_translation_mm": (
                            None
                            if diagnostic is None
                            else diagnostic.leave_one_out_translation_mm
                        ),
                        "leave_one_out_rotation_deg": (
                            None
                            if diagnostic is None
                            else diagnostic.leave_one_out_rotation_deg
                        ),
                        "flags": () if diagnostic is None else diagnostic.flags,
                    }
                )
            return {
                "configured": self._settings is not None and self._fatal_error is None,
                "fatal_error": self._fatal_error,
                "target_ready": ready,
                "gate": gate,
                "detection": self._detection_status,
                "corner_count": self._latest_corner_count,
                "sample_count": len(self._calibration_samples),
                "minimum_samples": self._minimum_samples,
                "has_solution": self._solution is not None,
                "save_allowed": (
                    self._solution is not None
                    and isinstance(self._diagnostics, AccuracyDiagnostics)
                    and self._diagnostics.save_allowed
                ),
                "diagnostics": self._diagnostics,
                "sample_rows": tuple(rows),
            }

    def latest_overlay(self):
        with self._lock:
            return None if self._latest_overlay is None else self._latest_overlay.copy()

    def _clear_solution_locked(self) -> None:
        self._solution = None
        self._diagnostics = None
        self._latest_overlay = None

    def _capture_observation(
        self,
        not_before_ns=None,
        expected_joints=None,
    ) -> tuple[bool, str, CalibrationSample | None, float | None, int | None]:
        with self._lock:
            ready, reason = self._target_gate_locked()
            if not ready:
                return False, reason, None, None, None
            camera_from_target = self._latest_pose.copy()
            joint_positions = self._latest_joint_positions
            joint_state_stamp = self._latest_joint_state_stamp
            pose_stamp = self._latest_pose_stamp if not_before_ns is not None else None
        if not_before_ns is not None:
            if (pose_stamp is None or pose_stamp <= not_before_ns
                    or not 0 <= (self.get_clock().now().nanoseconds - pose_stamp) / 1e9
                    <= TARGET_MAX_AGE_SEC
                    or joint_state_stamp is None
                    or joint_state_stamp.nanoseconds <= not_before_ns):
                return False, "Waiting for RGB and joints newer than arrival", None, None, None
            if (joint_positions is None or expected_joints is None
                    or max(abs(a - b) for a, b in zip(joint_positions, expected_joints))
                    > math.radians(1.0)):
                return False, "Live joints do not match the saved position", None, None, None
        if joint_positions is None or joint_state_stamp is None:
            message = (
                f"Required canonical {JOINT_STATE_TOPIC} feedback has not been received."
            )
            self._event_logger.record("ERROR", "sample_rejected", message)
            return False, message, None, None, None
        joint_state_age = (
            self.get_clock().now() - joint_state_stamp
        ).nanoseconds / 1e9
        if (
            not math.isfinite(joint_state_age)
            or joint_state_age < 0.0
            or joint_state_age > ROBOT_JOINT_STATE_MAX_AGE_SEC
        ):
            message = (
                f"Canonical {JOINT_STATE_TOPIC} feedback is stale or invalid: "
                f"age={joint_state_age:.3f}s, "
                f"maximum={ROBOT_JOINT_STATE_MAX_AGE_SEC:.3f}s."
            )
            self._event_logger.record("ERROR", "sample_rejected", message)
            return False, message, None, None, None
        try:
            transform = self._tf_buffer.lookup_transform(
                BASE_FRAME,
                TOOL_FRAME,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            message = f"Required robot TF {BASE_FRAME} <- {TOOL_FRAME} is unavailable: {exc}"
            self._event_logger.record("ERROR", "sample_rejected", message)
            return False, message, None, None, None
        stamp = Time.from_msg(transform.header.stamp)
        if not_before_ns is not None and stamp.nanoseconds <= not_before_ns:
            return False, "Waiting for robot TF newer than arrival", None, None, None
        if stamp.nanoseconds == 0:
            message = f"Robot TF {BASE_FRAME} <- {TOOL_FRAME} has a zero timestamp."
            self._event_logger.record("ERROR", "sample_rejected", message)
            return False, message, None, None, None
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if not math.isfinite(age) or age < 0.0 or age > ROBOT_TF_MAX_AGE_SEC:
            message = (
                f"Robot TF {BASE_FRAME} <- {TOOL_FRAME} is stale or invalid: "
                f"age={age:.3f}s, maximum={ROBOT_TF_MAX_AGE_SEC:.3f}s."
            )
            self._event_logger.record("ERROR", "sample_rejected", message)
            return False, message, None, None, None
        try:
            base_from_tool = _transform_message_to_matrix(transform.transform)
        except ValueError as exc:
            message = f"Robot TF is invalid: {exc}"
            self._event_logger.record("ERROR", "sample_rejected", message)
            return False, message, None, None, None

        with self._lock:
            conflict = sample_rotation_conflict(
                base_from_tool, camera_from_target, self._calibration_samples
            )
            if conflict is not None:
                prior_id, observation, angle = conflict
                message = (
                    f"{observation} orientation is too similar to {prior_id}: "
                    f"{angle:.3f}deg; require at least {MINIMUM_SAMPLE_ROTATION_DEG:.0f}deg."
                )
                self._event_logger.record(
                    "WARNING", "sample_rejected", message,
                    existing_sample_id=prior_id, observation=observation,
                    rotation_delta_deg=angle,
                    minimum_rotation_deg=MINIMUM_SAMPLE_ROTATION_DEG,
                )
                return False, message, None, None, None
            sample_id = f"C{self._next_calibration_sample_number}"
            self._next_calibration_sample_number += 1
            corner_count = self._latest_corner_count
        sample = CalibrationSample(
            sample_id,
            base_from_tool,
            camera_from_target,
            joint_positions,
        )
        return True, "", sample, age, corner_count

    def capture_sample(self, *, not_before_ns=None, expected_joints=None,
                       run_id=None) -> tuple[bool, str]:
        if not_before_ns is None and self._automatic_busy():
            return False, "Stop automatic capture before editing samples."
        success, reason, sample, age, corner_count = (
            self._capture_observation() if not_before_ns is None
            else self._capture_observation(not_before_ns, expected_joints))
        if not success or sample is None or age is None or corner_count is None:
            return False, reason
        with self._lock:
            if not_before_ns is not None and (
                    not self.automatic.active or self.automatic.stopping
                    or run_id != self.automatic.run_id):
                return False, "Automatic capture was stopped"
            self._calibration_samples.append(sample)
            count = len(self._calibration_samples)
            if count < self._minimum_samples:
                self._clear_solution_locked()
        message = (
            f"Captured calibration sample {sample.sample_id}; "
            f"{count}/{self._minimum_samples} calibration samples."
        )
        self._event_logger.record(
            "INFO",
            "calibration_sample_captured",
            message,
            sample_id=sample.sample_id,
            sample_count=count,
            robot_tf_age_sec=age,
            charuco_corners=corner_count,
            joint_names=list(JOINT_NAMES),
            joint_positions_rad=list(sample.joint_positions_rad),
        )
        if count >= self._minimum_samples:
            computed, compute_message = self.compute()
            return computed, f"{message} {compute_message}"
        return True, message

    def undo_sample(self) -> tuple[bool, str]:
        with self._lock:
            if not self._calibration_samples:
                return False, "No calibration sample exists to remove."
            sample_id = self._calibration_samples[-1].sample_id
        return self.remove_sample(sample_id)

    def remove_sample(self, sample_id: str) -> tuple[bool, str]:
        if self._automatic_busy():
            return False, "Stop automatic capture before editing samples."
        with self._lock:
            calibration_index = next(
                (
                    index
                    for index, sample in enumerate(self._calibration_samples)
                    if sample.sample_id == sample_id
                ),
                None,
            )
            if calibration_index is None:
                return False, f"Sample {sample_id} does not exist."
            self._calibration_samples.pop(calibration_index)
            calibration_count = len(self._calibration_samples)
            if calibration_count < self._minimum_samples:
                self._clear_solution_locked()
        message = f"Removed {sample_id}; {calibration_count} calibration samples remain."
        self._event_logger.record(
            "INFO",
            "sample_removed",
            message,
            sample_id=sample_id,
            calibration_sample_count=calibration_count,
        )
        if calibration_count >= self._minimum_samples:
            computed, compute_message = self.compute()
            return computed, f"{message} {compute_message}"
        return True, message

    def _automatic_busy(self):
        automatic = getattr(self, "automatic", None)
        return automatic is not None and (
            automatic.active or automatic.capturing or automatic.stopping)

    def reset_samples(self, *, _automatic=False) -> str:
        if not _automatic and self._automatic_busy():
            return "Stop automatic capture before editing samples."
        with self._lock:
            self._calibration_samples.clear()
            self._next_calibration_sample_number = 1
            self._clear_solution_locked()
        message = "All calibration samples and the solution were cleared."
        self._event_logger.record("INFO", "samples_reset", message)
        return message

    def load_calibration(
        self,
        path: Path,
    ) -> tuple[CalibrationArtifact, bool, str]:
        try:
            artifact = load_calibration_yaml(path)
        except ValueError as exc:
            message = f"Failed to load calibration YAML: {exc}"
            self._event_logger.record(
                "ERROR",
                "calibration_load_failed",
                message,
                input_path=str(path),
            )
            raise RuntimeError(message) from exc

        self.configure(artifact.settings, artifact.calibration_mode)
        with self._lock:
            self._calibration_samples = [
                CalibrationSample(
                    sample.sample_id,
                    sample.base_from_tool.copy(),
                    sample.camera_from_target.copy(),
                    sample.joint_positions_rad,
                )
                for sample in artifact.samples
            ]
            self._next_calibration_sample_number = max(
                int(sample.sample_id[1:]) for sample in artifact.samples
            ) + 1
        self.automatic.set_recipe(artifact, path)
        computed, compute_message = self.compute()
        if computed:
            message = (
                f"Loaded {len(artifact.samples)} calibration samples from {path}. "
                f"{compute_message}"
            )
            self._event_logger.record(
                "INFO",
                "calibration_loaded",
                message,
                input_path=str(path),
                source_created_utc=artifact.created_at_utc,
                calibration_mode=artifact.calibration_mode,
                camera_prefix=artifact.settings.camera_prefix,
                calibration_sample_count=len(artifact.samples),
                next_sample_id=f"C{self._next_calibration_sample_number}",
            )
        else:
            message = (
                f"Loaded {len(artifact.samples)} calibration samples from {path}, but "
                f"the solution could not be recalculated: {compute_message}"
            )
            self._event_logger.record(
                "ERROR",
                "calibration_load_recompute_failed",
                message,
                input_path=str(path),
                calibration_sample_count=len(artifact.samples),
            )
        return artifact, computed, message

    def compute(self) -> tuple[bool, str]:
        with self._lock:
            settings = self._settings
            samples = [
                CalibrationSample(
                    item.sample_id,
                    item.base_from_tool.copy(),
                    item.camera_from_target.copy(),
                    item.joint_positions_rad,
                )
                for item in self._calibration_samples
            ]
            minimum_samples = self._minimum_samples
            calibration_mode = self._calibration_mode
            reference_frame = self._reference_frame
            previous_solution = None if self._solution is None else self._solution.copy()
            previous_quality = (
                None
                if not isinstance(self._diagnostics, AccuracyDiagnostics)
                else self._diagnostics.fit
            )
        if settings is None or calibration_mode is None:
            return False, "Apply valid settings first."
        if len(samples) < minimum_samples:
            return False, f"Need {minimum_samples} samples; currently have {len(samples)}."
        try:
            optical_transform = self._tf_buffer.lookup_transform(
                settings.camera_link_frame,
                settings.optical_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
            camera_link_from_optical = _transform_message_to_matrix(optical_transform.transform)
            solve_result = self._opencv_worker.solve(
                calibration_mode=calibration_mode,
                samples=samples,
                camera_link_from_optical=camera_link_from_optical,
                previous_solution=previous_solution,
                previous_quality=previous_quality,
            )
            reference_from_camera_link = solve_result.reference_from_camera_link
            diagnostics = solve_result.diagnostics
            quality = diagnostics.fit
            change = diagnostics.solution_change
            leave_one_out = diagnostics.leave_one_out
            coverage = diagnostics.pose_coverage
        except OpenCvWorkerFailure as exc:
            self._hard_fail_opencv_worker(exc)
            return False, str(exc)
        except OpenCvWorkerOperationError as exc:
            if exc.error_type in {
                "InvalidPoseError",
                "OpenCvRuntimeContractError",
            }:
                self._hard_fail_opencv_worker(exc)
                return False, str(exc)
            message = f"Calibration solve failed: {exc}"
            with self._lock:
                self._clear_solution_locked()
            self._event_logger.record(
                "ERROR",
                "calibration_failed",
                message,
                sample_count=len(samples),
            )
            return False, message
        except (ValueError, TransformException) as exc:
            message = f"Calibration solve failed: {exc}"
            with self._lock:
                self._clear_solution_locked()
            self._event_logger.record(
                "ERROR",
                "calibration_failed",
                message,
                sample_count=len(samples),
            )
            return False, message
        with self._lock:
            self._solution = reference_from_camera_link
            self._diagnostics = diagnostics
        self._publish_solution_preview(
            reference_frame,
            settings.camera_link_frame,
            reference_from_camera_link,
        )
        message = (
            f"Computed {reference_frame} <- {settings.camera_link_frame} "
            f"in {calibration_mode} mode from {quality.sample_count} samples; "
            f"FIT RMS={quality.translation_rms_mm:.3f}mm/"
            f"{quality.rotation_rms_deg:.3f}deg."
        )
        if leave_one_out.status == "failed":
            failed_text = ", ".join(leave_one_out.failed_sample_ids)
            message += (
                f" Leave-one-out failed for {failed_text}; YAML saving is disabled."
            )
            self._event_logger.record(
                "ERROR",
                "leave_one_out_failed",
                "Leave-one-out diagnostics failed; YAML saving disabled.",
                failed_sample_ids=list(leave_one_out.failed_sample_ids),
                sample_count=quality.sample_count,
            )
        self._event_logger.record(
            "INFO",
            "calibration_computed",
            message,
            calibration_mode=calibration_mode,
            reference_frame=reference_frame,
            sample_count=quality.sample_count,
            translation_rms_mm=quality.translation_rms_mm,
            rotation_rms_deg=quality.rotation_rms_deg,
            max_translation_residual_mm=quality.max_translation_residual_mm,
            max_translation_sample_id=quality.max_translation_sample_id,
            max_rotation_residual_deg=quality.max_rotation_residual_deg,
            max_rotation_sample_id=quality.max_rotation_sample_id,
            solution_change_available=change.available,
            camera_translation_delta_mm=change.camera_translation_delta_mm,
            camera_rotation_delta_deg=change.camera_rotation_delta_deg,
            leave_one_out_status=leave_one_out.status,
            leave_one_out_translation_rms_mm=leave_one_out.translation_rms_mm,
            leave_one_out_rotation_rms_deg=leave_one_out.rotation_rms_deg,
            pose_translation_span_mm=coverage.translation_span_mm,
            pose_rotation_span_deg=coverage.rotation_span_deg,
            ax_xb_pair_count=diagnostics.ax_xb.pair_count,
            ax_xb_translation_rms_mm=diagnostics.ax_xb.translation_rms_mm,
            ax_xb_rotation_rms_deg=diagnostics.ax_xb.rotation_rms_deg,
        )
        return True, message

    def _publish_solution_preview(
        self,
        reference_frame: str,
        camera_link_frame: str,
        reference_from_camera_link: np.ndarray,
    ) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = reference_frame
        transform.child_frame_id = camera_link_frame
        transform.transform.translation.x = float(reference_from_camera_link[0, 3])
        transform.transform.translation.y = float(reference_from_camera_link[1, 3])
        transform.transform.translation.z = float(reference_from_camera_link[2, 3])
        qx, qy, qz, qw = rotation_matrix_to_quaternion(reference_from_camera_link[:3, :3])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)

    def _rebroadcast_solution_preview(self) -> None:
        with self._lock:
            settings = self._settings
            solution = None if self._solution is None else self._solution.copy()
            reference_frame = self._reference_frame
        if settings is not None and solution is not None:
            self._publish_solution_preview(reference_frame, settings.camera_link_frame, solution)

    def save(self, filename=None, *, created_at=None) -> tuple[bool, str]:
        automatic = getattr(self, "automatic", None)
        if automatic is not None and (self._automatic_busy()
                                      or automatic.started and not automatic.complete):
            return False, "Complete automatic capture before saving a new calibration."
        with self._lock:
            settings = self._settings
            solution = None if self._solution is None else self._solution.copy()
            diagnostics = self._diagnostics
            calibration_mode = self._calibration_mode
            samples = [
                CalibrationSample(
                    item.sample_id,
                    item.base_from_tool.copy(),
                    item.camera_from_target.copy(),
                    item.joint_positions_rad,
                )
                for item in self._calibration_samples
            ]
        if (
            settings is None
            or calibration_mode is None
            or solution is None
            or not isinstance(diagnostics, AccuracyDiagnostics)
        ):
            return False, (
                f"Capture at least {MINIMUM_CALIBRATION_SAMPLES} accepted samples and "
                "obtain a valid automatic calibration before saving."
            )
        if not diagnostics.save_allowed:
            failed_ids = ", ".join(diagnostics.leave_one_out.failed_sample_ids)
            return False, (
                "YAML saving is disabled because leave-one-out diagnostics failed "
                f"for {failed_ids}. Improve or remove calibration samples first."
            )
        created_at = created_at or datetime.now(timezone.utc)
        try:
            path = output_path_for_mode(
                calibration_mode, created_at=created_at, filename=filename)
        except ValueError as exc:
            return False, str(exc)
        try:
            write_calibration_yaml(
                path,
                calibration_mode,
                settings,
                solution,
                diagnostics,
                samples,
                created_at=created_at,
            )
        except (OSError, ValueError) as exc:
            message = f"Failed to save calibration YAML: {exc}"
            self._event_logger.record(
                "ERROR",
                "calibration_save_failed",
                message,
                output_path=str(path),
            )
            return False, message
        message = f"Saved calibration YAML: {path}"
        self._event_logger.record(
            "INFO",
            "calibration_saved",
            message,
            output_path=str(path),
            camera_prefix=settings.camera_prefix,
            calibration_mode=calibration_mode,
            calibration_sample_count=diagnostics.fit.sample_count,
            fit_translation_rms_mm=diagnostics.fit.translation_rms_mm,
            fit_rotation_rms_deg=diagnostics.fit.rotation_rms_deg,
            leave_one_out_status=diagnostics.leave_one_out.status,
            ax_xb_translation_rms_mm=diagnostics.ax_xb.translation_rms_mm,
            ax_xb_rotation_rms_deg=diagnostics.ax_xb.rotation_rms_deg,
        )
        return True, message


class CalibrationWindow(QtWidgets.QWidget):
    def __init__(self, node: CalibrationNode) -> None:
        super().__init__()
        self._node = node
        self._configuration_applied = False
        self._sample_table_signature = None
        self._fatal_shutdown_requested = False
        self._retry_dialog = None
        self.setWindowTitle("DOBOT Camera Calibration - ChArUco")
        self.resize(1800, 980)
        self._build_ui()
        self._connect_dirty_signals()
        restored_state = self._node.load_last_session()
        if restored_state is not None:
            self._restore_last_session(restored_state)
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._refresh)
        self._timer.start()
        if restored_state is None:
            self._log(
                "No previous UI session exists. Select a mode, enter the camera prefix "
                "and ChArUco board measurements, then select Apply Settings."
            )
        else:
            self._log(
                f"Restored the last validated UI session from {self._node.ui_state_path} "
                f"(saved {restored_state.saved_at_utc}). Select Apply Settings to start."
            )

    def _restore_last_session(self, state: CalibrationUiState) -> None:
        mode_index = self.calibration_mode.findData(state.calibration_mode)
        dictionary_index = self.dictionary.findText(state.settings.dictionary_name)
        if mode_index < 0 or dictionary_index < 0:
            raise RuntimeError(
                "Last-session state references a mode or dictionary absent from the GUI"
            )
        self.calibration_mode.setCurrentIndex(mode_index)
        self.camera_prefix.setText(state.settings.camera_prefix)
        self.dictionary.setCurrentIndex(dictionary_index)
        self.squares_x.setValue(state.settings.squares_x)
        self.squares_y.setValue(state.settings.squares_y)
        self.square_length.setValue(state.settings.square_length_mm)
        self.marker_length.setValue(state.settings.marker_length_mm)
        self._configuration_applied = False
        self.output_label.setText("Output: restored settings require Apply Settings")

    def _build_ui(self) -> None:
        root = QtWidgets.QHBoxLayout(self)
        controls = QtWidgets.QWidget()
        controls.setMaximumWidth(470)
        controls_layout = QtWidgets.QVBoxLayout(controls)

        form = QtWidgets.QFormLayout()
        self.calibration_mode = QtWidgets.QComboBox()
        self.calibration_mode.addItem(
            "Camera to hand (fixed camera)",
            CAMERA_TO_HAND,
        )
        self.calibration_mode.addItem(
            "Camera on hand (wrist camera)",
            CAMERA_ON_HAND,
        )
        self.calibration_mode.setCurrentIndex(-1)
        self.camera_prefix = QtWidgets.QLineEdit()
        self.camera_prefix.setPlaceholderText("Enter camera prefix, for example bin_camera")
        self.reference_frame = QtWidgets.QLabel("Select a calibration mode")
        self.board_placement = QtWidgets.QLabel("Select a calibration mode")
        self.board_placement.setWordWrap(True)
        self.dictionary = QtWidgets.QComboBox()
        self.dictionary.addItems(list(CHARUCO_DICTIONARIES.keys()))
        self.squares_x = QtWidgets.QSpinBox()
        self.squares_x.setRange(3, 30)
        self.squares_x.setValue(5)
        self.squares_y = QtWidgets.QSpinBox()
        self.squares_y.setRange(3, 30)
        self.squares_y.setValue(7)
        self.square_length = QtWidgets.QDoubleSpinBox()
        self.square_length.setRange(0.1, 1000.0)
        self.square_length.setDecimals(3)
        self.square_length.setSuffix(" mm")
        self.square_length.setValue(30.0)
        self.marker_length = QtWidgets.QDoubleSpinBox()
        self.marker_length.setRange(0.1, 1000.0)
        self.marker_length.setDecimals(3)
        self.marker_length.setSuffix(" mm")
        self.marker_length.setValue(22.0)
        self.minimum_samples = QtWidgets.QLabel(
            f"{MINIMUM_CALIBRATION_SAMPLES} (fixed; automatic solve)"
        )
        form.addRow("Calibration mode", self.calibration_mode)
        form.addRow("Output reference", self.reference_frame)
        form.addRow("ChArUco placement", self.board_placement)
        form.addRow("Camera prefix", self.camera_prefix)
        form.addRow("ArUco dictionary", self.dictionary)
        form.addRow("Checker squares X", self.squares_x)
        form.addRow("Checker squares Y", self.squares_y)
        form.addRow("Checker square size", self.square_length)
        form.addRow("ArUco marker size", self.marker_length)
        form.addRow("Minimum samples", self.minimum_samples)
        guidance = QtWidgets.QLabel(
            "Hold stationary; rotate about multiple axes.\n"
            "Require ≥4 non-collinear ChArUco corners.\n"
            "Each new robot and board orientation must\n"
            "differ by ≥5° from every prior sample."
        )
        guidance.setWordWrap(True)
        guidance.setMinimumHeight(guidance.fontMetrics().lineSpacing() * 4)
        form.addRow(guidance)
        controls_layout.addLayout(form)

        self.apply_button = QtWidgets.QPushButton("Apply Settings")
        self.apply_button.clicked.connect(self._apply_settings)
        controls_layout.addWidget(self.apply_button)

        actions = QtWidgets.QGridLayout()
        self.capture_button = QtWidgets.QPushButton("Capture Calibration")
        self.load_button = QtWidgets.QPushButton("Load Calibration")
        self.undo_button = QtWidgets.QPushButton("Undo Last Calibration")
        self.remove_button = QtWidgets.QPushButton("Remove Selected Sample")
        self.reset_button = QtWidgets.QPushButton("Reset Samples")
        self.save_button = QtWidgets.QPushButton("Save YAML")
        self.capture_button.clicked.connect(self._capture)
        self.load_button.clicked.connect(self._load_calibration)
        self.undo_button.clicked.connect(self._undo)
        self.remove_button.clicked.connect(self._remove_selected)
        self.reset_button.clicked.connect(self._reset)
        self.save_button.clicked.connect(self._save)
        actions.addWidget(self.capture_button, 0, 0)
        actions.addWidget(self.load_button, 0, 1)
        actions.addWidget(self.undo_button, 1, 0)
        actions.addWidget(self.remove_button, 1, 1)
        actions.addWidget(self.reset_button, 2, 0)
        actions.addWidget(self.save_button, 2, 1)
        self.automatic_button = QtWidgets.QPushButton("Start Automatic Capture")
        self.automatic_stop_button = QtWidgets.QPushButton("Stop Automatic Capture")
        self.save_new_button = QtWidgets.QPushButton("Save as New Calibration")
        self.automatic_button.clicked.connect(self._start_automatic)
        self.automatic_stop_button.clicked.connect(self._node.automatic.stop)
        self.save_new_button.clicked.connect(self._save_as)
        actions.addWidget(self.automatic_button, 3, 0)
        actions.addWidget(self.automatic_stop_button, 3, 1)
        actions.addWidget(self.save_new_button, 4, 0, 1, 2)
        controls_layout.addLayout(actions)

        self.automatic_label = QtWidgets.QLabel(self._node.automatic.message)
        self.automatic_label.setWordWrap(True)
        controls_layout.addWidget(self.automatic_label)

        self.gate_label = QtWidgets.QLabel("Configuration not applied")
        self.gate_label.setWordWrap(True)
        self.gate_label.setMinimumHeight(58)
        controls_layout.addWidget(self.gate_label)

        self.output_label = QtWidgets.QLabel("Output: waiting for camera prefix")
        self.output_label.setWordWrap(True)
        controls_layout.addWidget(self.output_label)

        self.status = QtWidgets.QPlainTextEdit()
        self.status.setReadOnly(True)
        controls_layout.addWidget(self.status, 1)

        overlay_group = QtWidgets.QGroupBox(
            "Live RGB ChArUco and Calibrated Camera Pose"
        )
        overlay_layout = QtWidgets.QVBoxLayout(overlay_group)
        video_layout = QtWidgets.QHBoxLayout()
        rgb_group = QtWidgets.QGroupBox("RGB board pose")
        rgb_layout = QtWidgets.QVBoxLayout(rgb_group)
        self.overlay = QtWidgets.QLabel(
            "Apply settings to subscribe to the selected camera."
        )
        self.overlay.setAlignment(QtCore.Qt.AlignCenter)
        self.overlay.setMinimumSize(480, 360)
        self.overlay.setStyleSheet(
            "QLabel { background-color: #101010; color: #d0d0d0; border: 1px solid #444; }"
        )
        rgb_layout.addWidget(self.overlay)
        video_layout.addWidget(rgb_group, 1)

        overlay_layout.addLayout(video_layout, 3)

        diagnostics_group = QtWidgets.QGroupBox("Quality Diagnostics")
        diagnostics_layout = QtWidgets.QVBoxLayout(diagnostics_group)
        self.diagnostics_summary = QtWidgets.QLabel(
            _diagnostics_summary_text(None, 0)
        )
        self.diagnostics_summary.setWordWrap(True)
        self.diagnostics_summary.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        diagnostics_layout.addWidget(self.diagnostics_summary)

        self.sample_table = QtWidgets.QTableWidget(0, 6)
        self.sample_table.setHorizontalHeaderLabels(
            (
                "ID",
                "Residual T [mm]",
                "Residual R [deg]",
                "LOO TF T [mm]",
                "LOO TF R [deg]",
                "Flags",
            )
        )
        self.sample_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.sample_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.sample_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.sample_table.verticalHeader().setVisible(False)
        header = self.sample_table.horizontalHeader()
        header.setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(5, QtWidgets.QHeaderView.Stretch)
        self.sample_table.itemSelectionChanged.connect(self._sample_selection_changed)
        diagnostics_layout.addWidget(self.sample_table, 1)
        overlay_layout.addWidget(diagnostics_group, 2)

        root.addWidget(controls)
        root.addWidget(overlay_group, 1)
        self._set_action_states(False, False, 0, False, False)

    def _connect_dirty_signals(self) -> None:
        self.calibration_mode.currentIndexChanged.connect(self._mode_changed)
        self.camera_prefix.textChanged.connect(self._mark_dirty)
        self.dictionary.currentTextChanged.connect(self._mark_dirty)
        self.squares_x.valueChanged.connect(self._mark_dirty)
        self.squares_y.valueChanged.connect(self._mark_dirty)
        self.square_length.valueChanged.connect(self._mark_dirty)
        self.marker_length.valueChanged.connect(self._mark_dirty)

    def _mode_changed(self, *_args) -> None:
        calibration_mode = self.calibration_mode.currentData()
        if calibration_mode == CAMERA_TO_HAND:
            self.reference_frame.setText(BASE_FRAME)
            self.board_placement.setText(f"Rigidly attach the board to {TOOL_FRAME}.")
        elif calibration_mode == CAMERA_ON_HAND:
            self.reference_frame.setText(TOOL_FRAME)
            self.board_placement.setText(f"Keep the board fixed relative to {BASE_FRAME}.")
        else:
            self.reference_frame.setText("Select a calibration mode")
            self.board_placement.setText("Select a calibration mode")
        self._mark_dirty()

    def _mark_dirty(self, *_args) -> None:
        if not self._configuration_applied:
            return
        self._configuration_applied = False
        self._log("Settings changed. Select Apply Settings; existing samples will be cleared.")

    def _settings(self) -> CharucoSettings:
        return CharucoSettings(
            camera_prefix=self.camera_prefix.text().strip(),
            dictionary_name=self.dictionary.currentText(),
            squares_x=int(self.squares_x.value()),
            squares_y=int(self.squares_y.value()),
            square_length_mm=float(self.square_length.value()),
            marker_length_mm=float(self.marker_length.value()),
        )

    def _selected_mode(self) -> str:
        calibration_mode = self.calibration_mode.currentData()
        return validate_calibration_mode(calibration_mode)

    def _apply_settings(self) -> None:
        try:
            settings = self._settings()
            calibration_mode = self._selected_mode()
            message = self._node.configure(
                settings,
                calibration_mode,
            )
            state = self._node.save_last_session(
                settings,
                calibration_mode,
            )
        except (ValueError, RuntimeError) as exc:
            self._configuration_applied = False
            self._log(f"ERROR: {exc}")
            QtWidgets.QMessageBox.critical(self, "Invalid calibration settings", str(exc))
            return
        self._configuration_applied = True
        self.output_label.setText(
            f"Output pattern: {output_pattern_for_mode(calibration_mode)}"
        )
        self._log(message)
        self._log(
            f"Saved last-session UI state at {state.saved_at_utc} to "
            f"{self._node.ui_state_path}."
        )
        self._log(
            f"Subscribed to {settings.color_topic} and "
            f"{settings.camera_info_topic}; both must use "
            f"{settings.optical_frame}."
        )

    def _capture(self) -> None:
        success, message = self._node.capture_sample()
        self._log(("OK: " if success else "ERROR: ") + message)

    def _start_automatic(self) -> None:
        recipe = self._node.automatic.recipe
        if recipe is None:
            return
        if QtWidgets.QMessageBox.question(
            self, "Start automatic robot capture",
            f"Move through {len(recipe.samples)} saved joint positions in order at "
            "20% speed and acceleration (global speed also applies)?\n\n"
            "Confirm the starting position and all connecting joint-motion paths are clear. "
            "Close robot_controller, Motion Debug and Gripper Diagnostics first. "
            "Release the robot after hand guiding. Calibration will exit drag mode if active "
            "and enable the robot if disabled, then confirm it is stationary and idle. "
            "Fresh joint/robot feedback, an empty queue, user/tool 0 and DI1 LOW are required. "
            "Gripper outputs stay unchanged. Current samples will be replaced by fresh captures. "
            "Each position gets three automatic attempts, with a one-second stationary hold "
            "before every capture. Only three failed attempts prompt Continue or Stop. "
            "A movement-timeout retry requires confirmed Stop and fresh robot checks. "
            "The robot will remain at the final position. The source file stays unchanged.",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        ) != QtWidgets.QMessageBox.Yes:
            return
        success, message = self._node.automatic.start()
        self._log(("OK: " if success else "ERROR: ") + message)

    def _undo(self) -> None:
        success, message = self._node.undo_sample()
        self._log(("OK: " if success else "ERROR: ") + message)

    def _load_calibration(self) -> None:
        initial_directory = str(output_pattern_for_mode(CAMERA_TO_HAND).parent)
        filename, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Load Calibration",
            initial_directory,
            "Calibration YAML (*.yaml)",
        )
        if not filename:
            return
        snapshot = self._node.status_snapshot()
        if int(snapshot["sample_count"]) > 0 and QtWidgets.QMessageBox.question(
            self,
            "Replace current calibration",
            "Loading this file will replace all current calibration samples and "
            "the calculated solution. Continue?",
        ) != QtWidgets.QMessageBox.Yes:
            return
        try:
            artifact, computed, message = self._node.load_calibration(Path(filename))
            self._restore_loaded_calibration(artifact)
            state = self._node.save_last_session(
                artifact.settings,
                artifact.calibration_mode,
            )
        except (OSError, ValueError, RuntimeError) as exc:
            self._log(f"ERROR: {exc}")
            QtWidgets.QMessageBox.critical(self, "Calibration load failed", str(exc))
            return
        self.output_label.setText(
            f"Output pattern: {output_pattern_for_mode(artifact.calibration_mode)}"
        )
        self._log(("OK: " if computed else "ERROR: ") + message)
        self._log(
            f"Saved loaded settings as last-session UI state at {state.saved_at_utc}."
        )
        if computed:
            QtWidgets.QMessageBox.information(self, "Calibration loaded", message)
        else:
            QtWidgets.QMessageBox.warning(
                self,
                "Calibration loaded without a solution",
                message,
            )

    def _restore_loaded_calibration(self, artifact: CalibrationArtifact) -> None:
        self._configuration_applied = False
        mode_index = self.calibration_mode.findData(artifact.calibration_mode)
        dictionary_index = self.dictionary.findText(artifact.settings.dictionary_name)
        if mode_index < 0 or dictionary_index < 0:
            raise RuntimeError(
                "Loaded calibration references a mode or dictionary absent from the GUI"
            )
        self.calibration_mode.setCurrentIndex(mode_index)
        self.camera_prefix.setText(artifact.settings.camera_prefix)
        self.dictionary.setCurrentIndex(dictionary_index)
        self.squares_x.setValue(artifact.settings.squares_x)
        self.squares_y.setValue(artifact.settings.squares_y)
        self.square_length.setValue(artifact.settings.square_length_mm)
        self.marker_length.setValue(artifact.settings.marker_length_mm)
        self._configuration_applied = True

    def _selected_sample_id(self) -> str | None:
        selected_rows = self.sample_table.selectionModel().selectedRows()
        if len(selected_rows) != 1:
            return None
        item = self.sample_table.item(selected_rows[0].row(), 0)
        return None if item is None else item.text()

    def _remove_selected(self) -> None:
        sample_id = self._selected_sample_id()
        if sample_id is None:
            self._log("ERROR: Select exactly one sample row to remove.")
            return
        if QtWidgets.QMessageBox.question(
            self,
            "Remove selected sample",
            f"Remove {sample_id}? This action never removes any other sample.",
        ) != QtWidgets.QMessageBox.Yes:
            return
        success, message = self._node.remove_sample(sample_id)
        self._log(("OK: " if success else "ERROR: ") + message)

    def _sample_selection_changed(self) -> None:
        self.remove_button.setEnabled(
            self._configuration_applied and not self._node._automatic_busy()
            and self._selected_sample_id() is not None
        )

    def _reset(self) -> None:
        if QtWidgets.QMessageBox.question(
            self,
            "Reset calibration samples",
            "Clear all calibration samples and the computed solution?",
        ) != QtWidgets.QMessageBox.Yes:
            return
        self._log(self._node.reset_samples())

    def _save(self) -> None:
        success, message = self._node.save()
        self._show_save_result(success, message)

    def _save_as(self) -> None:
        created_at = datetime.now(timezone.utc)
        default = output_path_for_mode(self._node._calibration_mode, created_at=created_at)
        filename, accepted = QtWidgets.QInputDialog.getText(
            self, "Save as New Calibration",
            "Filename in calibration/ (existing files cannot be overwritten).\n"
            "Keep the default name for automatic station discovery.",
            QtWidgets.QLineEdit.Normal, default.name)
        if not accepted:
            return
        success, message = self._node.save(filename, created_at=created_at)
        self._show_save_result(success, message)

    def _show_save_result(self, success, message) -> None:
        self._log(("OK: " if success else "ERROR: ") + message)
        if success:
            QtWidgets.QMessageBox.information(self, "Calibration saved", message)
        else:
            QtWidgets.QMessageBox.critical(self, "Calibration save failed", message)

    def _log(self, message: str) -> None:
        self.status.appendPlainText(str(message))

    def _set_action_states(
        self,
        configured: bool,
        target_ready: bool,
        sample_count: int,
        has_solution: bool,
        save_allowed: bool,
    ) -> None:
        automatic = self._node.automatic
        busy = self._node._automatic_busy()
        active = configured and self._configuration_applied and not busy
        self.capture_button.setEnabled(active and target_ready)
        self.undo_button.setEnabled(active and sample_count > 0)
        self.remove_button.setEnabled(active and self._selected_sample_id() is not None)
        self.reset_button.setEnabled(active and sample_count > 0)
        can_save = (active and has_solution and save_allowed
                    and (not automatic.started or automatic.complete))
        self.save_button.setEnabled(can_save)
        self.save_button.setVisible(automatic.recipe is None)
        self.save_new_button.setVisible(automatic.recipe is not None)
        self.save_new_button.setEnabled(can_save)
        self.automatic_button.setEnabled(active and automatic.recipe is not None)
        self.automatic_stop_button.setEnabled(automatic.active or automatic.stopping)
        self.load_button.setEnabled(not busy)
        self.apply_button.setEnabled(not busy)
        for widget in (self.calibration_mode, self.camera_prefix, self.dictionary,
                       self.squares_x, self.squares_y, self.square_length, self.marker_length):
            widget.setEnabled(not busy)

    def _refresh_sample_table(self, rows: tuple[dict, ...]) -> None:
        signature = tuple(
            (
                row["sample_id"],
                row["translation_residual_mm"],
                row["rotation_residual_deg"],
                row["leave_one_out_translation_mm"],
                row["leave_one_out_rotation_deg"],
                row["flags"],
            )
            for row in rows
        )
        if signature == self._sample_table_signature:
            return
        selected_id = self._selected_sample_id()
        self.sample_table.setRowCount(len(rows))
        selected_row = None
        for row_index, row in enumerate(rows):
            values = (
                row["sample_id"],
                _metric_text(row["translation_residual_mm"], ""),
                _metric_text(row["rotation_residual_deg"], ""),
                _metric_text(row["leave_one_out_translation_mm"], ""),
                _metric_text(row["leave_one_out_rotation_deg"], ""),
                ", ".join(row["flags"]),
            )
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(str(value))
                if row["flags"]:
                    font = item.font()
                    font.setBold(True)
                    item.setFont(font)
                self.sample_table.setItem(row_index, column, item)
            if row["sample_id"] == selected_id:
                selected_row = row_index
        if selected_row is not None:
            self.sample_table.selectRow(selected_row)
        self._sample_table_signature = signature

    @staticmethod
    def _render_overlay(
        label: QtWidgets.QLabel,
        image: np.ndarray | None,
        waiting_text: str,
    ) -> None:
        if image is None:
            label.setPixmap(QtGui.QPixmap())
            label.setText(waiting_text)
            return
        height, width, channels = image.shape
        qimage = QtGui.QImage(
            image.data,
            width,
            height,
            channels * width,
            QtGui.QImage.Format_RGB888,
        ).copy()
        pixmap = QtGui.QPixmap.fromImage(qimage).scaled(
            label.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )
        label.setText("")
        label.setPixmap(pixmap)

    def _refresh_retry_prompt(self) -> None:
        automatic = self._node.automatic
        prompt = getattr(automatic, "retry_prompt", None)
        if prompt is not None and prompt["event"].is_set():
            prompt = None
        token = None if prompt is None else prompt["token"]
        if self._retry_dialog is not None and self._retry_dialog[0] != token:
            dialog = self._retry_dialog[1]
            self._retry_dialog = None
            dialog.blockSignals(True)
            dialog.close()
            dialog.deleteLater()
        if prompt is None or self._retry_dialog is not None:
            return
        if prompt["phase"] == "motion":
            detail = ("Robot Stop is confirmed. Check the robot and the path to this position. "
                      "Continue may command movement again.")
        elif prompt["phase"] == "camera":
            detail = ("Check the RGB stream and CameraInfo. The robot is stationary. "
                      "Continue waits for valid camera data before moving to the saved joints.")
        else:
            detail = ("Check whether the camera view or ChArUco board is obstructed. "
                      "The robot remains at this position for another capture attempt.")
        dialog = QtWidgets.QMessageBox(self)
        dialog.setWindowTitle("Calibration: three attempts exhausted")
        dialog.setIcon(QtWidgets.QMessageBox.Warning)
        dialog.setText(
            f"Position {prompt['index']}: all three attempts failed.\n\n"
            f"{prompt['reason']}\n\n{detail}\n\n"
            "Continue starts another three attempts here, with a one-second stationary "
            "hold before each capture. Earlier samples are kept. Stop ends the run.")
        dialog.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        dialog.button(QtWidgets.QMessageBox.Yes).setText("Continue")
        dialog.button(QtWidgets.QMessageBox.No).setText("Stop")
        dialog.setDefaultButton(QtWidgets.QMessageBox.No)
        dialog.setEscapeButton(QtWidgets.QMessageBox.No)
        dialog.setWindowModality(QtCore.Qt.NonModal)
        self._retry_dialog = token, dialog
        dialog.finished.connect(lambda result: self._answer_retry_prompt(token, result))
        dialog.show()

    def _answer_retry_prompt(self, token, result) -> None:
        if self._retry_dialog is not None and self._retry_dialog[0] == token:
            self._retry_dialog[1].deleteLater()
            self._retry_dialog = None
        self._node.automatic.respond_retry(token, result == QtWidgets.QMessageBox.Yes)

    def _refresh(self) -> None:
        snapshot = self._node.status_snapshot()
        self.automatic_label.setText(self._node.automatic.message)
        self._refresh_retry_prompt()
        fatal_error = snapshot.get("fatal_error")
        if fatal_error is not None:
            if not self._fatal_shutdown_requested:
                self._fatal_shutdown_requested = True
                self._configuration_applied = False
                self._set_action_states(False, False, 0, False, False)
                self._log(f"FATAL: {fatal_error}")
                self.gate_label.setText(f"Target gate: FATAL\n{fatal_error}")
                self.gate_label.setStyleSheet(
                    "QLabel { background-color: #f5c2c7; padding: 8px; }"
                )
                QtCore.QCoreApplication.exit(1)
            return
        configured = bool(snapshot["configured"] and self._configuration_applied)
        target_ready = bool(snapshot["target_ready"] and configured)
        gate_text = snapshot["gate"] if configured else "Settings changed or not yet applied."
        prefix = "READY" if target_ready else "BLOCKED"
        color = "#b7e4c7" if target_ready else "#f5c2c7"
        self.gate_label.setText(
            f"Target gate: {prefix}\n{gate_text}\n"
            f"Calibration: {snapshot['sample_count']} "
            f"(automatic solve at {snapshot['minimum_samples']})"
        )
        self.gate_label.setStyleSheet(f"QLabel {{ background-color: {color}; padding: 8px; }}")
        self._set_action_states(
            configured,
            target_ready,
            int(snapshot["sample_count"]),
            bool(snapshot["has_solution"]),
            bool(snapshot["save_allowed"]),
        )
        self.diagnostics_summary.setText(
            _diagnostics_summary_text(
                snapshot["diagnostics"],
                int(snapshot["sample_count"]),
            )
        )
        self._refresh_sample_table(snapshot["sample_rows"])

        self._render_overlay(
            self.overlay, self._node.latest_overlay(), snapshot["detection"]
        )


def main(args=None) -> None:
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "camera_calibration requires ROS_LOCALHOST_ONLY=1; "
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
        node = CalibrationNode()
        executor = _create_calibration_executor(node)
        spin_thread = threading.Thread(
            target=executor.spin,
            daemon=True,
        )
        spin_thread.start()
        window = CalibrationWindow(node)
        window.show()
        application_exit_code = application.exec_()
        fatal_error = node.fatal_error()
        if fatal_error is not None:
            raise RuntimeError(fatal_error)
        if application_exit_code != 0:
            raise RuntimeError(
                f"Camera calibration GUI exited with code {application_exit_code}"
            )
    finally:
        if node is not None:
            node.automatic.close()
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
        if spin_thread is not None:
            spin_thread.join(timeout=1.0)
        if node is not None:
            node.close_opencv_worker()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    main()
