import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import cv2
import numpy as np
import pytest
import yaml
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.clock import ClockType
from rclpy.context import Context
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from camera_calibration_gui import main as calibration_main
from camera_calibration_gui import calibration_core
from camera_calibration_gui.calibration_core import (
    CAMERA_ON_HAND,
    CAMERA_TO_HAND,
    MINIMUM_CALIBRATION_SAMPLES,
    REQUIRED_CHARUCO_LEGACY_PATTERN,
    REQUIRED_HAND_EYE_METHOD,
    REQUIRED_OPENCV_API,
    REQUIRED_OPENCV_THREAD_COUNT,
    REQUIRED_OPENCV_VERSION,
    REQUIRED_OPENCV_WHEEL_SHA256,
    AccuracyDiagnostics,
    CalibrationSample,
    CalibrationUiState,
    CalibrationQuality,
    CharucoSettings,
    AxXbDiagnostics,
    ax_xb_diagnostics,
    sample_rotation_conflict,
    InvalidPoseError,
    LeaveOneOutDiagnostics,
    PackageEventLogger,
    PoseCoverageDiagnostics,
    SampleResidual,
    SolutionChangeDiagnostics,
    calibration_quality_from_transforms,
    calibration_ui_state_path,
    invert_transform,
    leave_one_out_diagnostics,
    load_calibration_yaml,
    load_calibration_ui_state,
    output_path_for_mode,
    pose_coverage_diagnostics,
    reference_frame_for_mode,
    rotation_angle_deg,
    rotation_matrix_to_rpy_deg,
    solution_change_diagnostics,
    solve_camera_on_hand,
    solve_camera_to_hand,
    write_calibration_yaml,
    write_calibration_ui_state,
)
from camera_calibration_gui.opencv_worker import (
    ArucoMarkerSettings,
    DetectionResult,
    OpenCvWorkerClient,
    OpenCvWorkerFailure,
    OpenCvWorkerOperationError,
    REQUIRED_ARUCO_API,
    SolveResult,
    _validate_detection_result,
)
from camera_calibration_gui import opencv_worker_runtime


def _opencv_runtime_dir() -> Path:
    value = os.environ.get("CAMERA_CALIBRATION_TEST_OPENCV_RUNTIME")
    if not value:
        raise RuntimeError(
            "CAMERA_CALIBRATION_TEST_OPENCV_RUNTIME must identify the extracted "
            "package-private OpenCV runtime"
        )
    return Path(value).resolve()


def _opencv_worker() -> OpenCvWorkerClient:
    return OpenCvWorkerClient(runtime_dir=_opencv_runtime_dir())


def _transform(rotation_vector, translation):
    rotation, _jacobian = cv2.Rodrigues(np.asarray(rotation_vector, dtype=np.float64))
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.asarray(translation, dtype=np.float64)
    return matrix


def _sample(sample_id, base_from_tool=None, camera_from_target=None, joint_positions_rad=None):
    return CalibrationSample(
        sample_id,
        np.eye(4, dtype=np.float64) if base_from_tool is None else base_from_tool,
        np.eye(4, dtype=np.float64) if camera_from_target is None else camera_from_target,
        (
            tuple(float(index) / 10.0 for index in range(6))
            if joint_positions_rad is None
            else tuple(float(position) for position in joint_positions_rad)
        ),
    )


def _quality(sample_count=MINIMUM_CALIBRATION_SAMPLES, translation=0.25, rotation=0.10):
    residuals = tuple(
        SampleResidual(f"C{index}", 0.0, 0.0)
        for index in range(1, sample_count + 1)
    )
    return CalibrationQuality(
        sample_count,
        translation,
        rotation,
        translation,
        "C1",
        rotation,
        "C1",
        residuals,
        np.eye(4, dtype=np.float64),
    )


def _diagnostics(quality=None, leave_one_out=None):
    quality = quality or _quality()
    leave_one_out = leave_one_out or LeaveOneOutDiagnostics(
        "not_available", (), (), None, None, None, None, None, None
    )
    return AccuracyDiagnostics(
        quality,
        SolutionChangeDiagnostics(False, None, None, None, None),
        leave_one_out,
        PoseCoverageDiagnostics(100.0, 20.0),
        AxXbDiagnostics(quality.sample_count - 1, 0.0, 0.0),
    )


_ROBOT_POSES = (
    ([0.00, 0.00, 0.00], [0.45, -0.25, 0.40]),
    ([0.20, -0.10, 0.10], [0.52, -0.10, 0.55]),
    ([-0.15, 0.25, -0.05], [0.35, 0.05, 0.62]),
    ([0.30, 0.15, -0.20], [0.58, 0.15, 0.48]),
    ([-0.25, -0.20, 0.18], [0.42, -0.18, 0.72]),
    ([0.12, 0.32, 0.25], [0.62, 0.02, 0.66]),
    ([-0.30, 0.10, -0.22], [0.30, -0.02, 0.52]),
    ([0.18, -0.28, 0.30], [0.55, -0.30, 0.60]),
)


def _synthetic_problem(calibration_mode, pose_indices, sample_prefix="C"):
    samples = []
    if calibration_mode == CAMERA_TO_HAND:
        expected = _transform([0.15, -0.25, 0.10], [0.65, -0.30, 1.20])
        constant = _transform([-0.05, 0.03, 0.12], [0.02, 0.01, 0.18])
        for sample_number, pose_index in enumerate(pose_indices, start=1):
            base_from_tool = _transform(*_ROBOT_POSES[pose_index])
            camera_from_target = (
                invert_transform(expected) @ base_from_tool @ constant
            )
            samples.append(
                _sample(
                    f"{sample_prefix}{sample_number}",
                    base_from_tool,
                    camera_from_target,
                )
            )
    else:
        expected = _transform([0.08, -0.12, 0.04], [0.04, 0.01, 0.09])
        constant = _transform([-0.10, 0.05, 0.20], [0.70, 0.15, 0.35])
        for sample_number, pose_index in enumerate(pose_indices, start=1):
            base_from_tool = _transform(*_ROBOT_POSES[pose_index])
            camera_from_target = (
                invert_transform(expected)
                @ invert_transform(base_from_tool)
                @ constant
            )
            samples.append(
                _sample(
                    f"{sample_prefix}{sample_number}",
                    base_from_tool,
                    camera_from_target,
                )
            )
    return samples, expected


def test_opencv_runtime_is_package_private_absolute_serial_and_opencl_free():
    runtime_dir = _opencv_runtime_dir()
    runtime = opencv_worker_runtime.configure_opencv_runtime(runtime_dir)
    assert runtime == {
        "opencv_version": REQUIRED_OPENCV_VERSION,
        "opencv_module_path": str(Path(cv2.__file__).resolve()),
        "opencv_thread_count": REQUIRED_OPENCV_THREAD_COUNT,
        "opencv_opencl_enabled": False,
        "opencv_api": REQUIRED_OPENCV_API,
        "charuco_legacy_pattern": REQUIRED_CHARUCO_LEGACY_PATTERN,
        "hand_eye_method": REQUIRED_HAND_EYE_METHOD,
        "opencv_wheel_sha256": REQUIRED_OPENCV_WHEEL_SHA256,
        "moveit_reference_commit": calibration_core.MOVEIT_REFERENCE_COMMIT,
        "marker_corner_refinement": "CORNER_REFINE_NONE",
        "adjacent_marker_minimum": 2,
        "marker_recovery": False,
        "bin_teach_aruco_api": REQUIRED_ARUCO_API,
    }
    assert Path(runtime["opencv_module_path"]).is_relative_to(runtime_dir)
    assert cv2.getNumThreads() == 1
    assert not cv2.ocl.useOpenCL()

    with patch.object(opencv_worker_runtime.cv2, "__version__", "4.10.1"):
        try:
            opencv_worker_runtime.configure_opencv_runtime(runtime_dir)
        except RuntimeError as exc:
            assert "requires package-private OpenCV version exactly 4.10.0" in str(exc)
        else:
            raise AssertionError("A noncanonical OpenCV version was accepted")


def test_opencv_runtime_rejects_missing_files_and_system_module_leakage(tmp_path):
    try:
        OpenCvWorkerClient(runtime_dir=tmp_path)
    except OpenCvWorkerFailure as exc:
        assert exc.operation == "startup"
        assert exc.worker_pid is None
        assert "runtime is incomplete" in str(exc)
    else:
        raise AssertionError("An incomplete package-private runtime was accepted")

    opencv_worker_runtime.configure_opencv_runtime(_opencv_runtime_dir())
    try:
        opencv_worker_runtime._verify_runtime(tmp_path)
    except RuntimeError as exc:
        assert "escaped the package-private runtime" in str(exc)
    else:
        raise AssertionError("An OpenCV module outside the selected runtime was accepted")


def test_opencv_runtime_rejects_api_thread_and_opencl_drift():
    runtime_dir = _opencv_runtime_dir()
    opencv_worker_runtime.configure_opencv_runtime(runtime_dir)
    with patch.object(opencv_worker_runtime.cv2.aruco, "CharucoDetector", None):
        try:
            opencv_worker_runtime._verify_runtime(runtime_dir)
        except RuntimeError as exc:
            assert "Required OpenCV 4.10 ChArUco API is missing" in str(exc)
        else:
            raise AssertionError("A missing modern ChArUco API was accepted")

    with patch.object(opencv_worker_runtime.cv2, "undistortPoints", None):
        with pytest.raises(RuntimeError, match="undistortPoints"):
            opencv_worker_runtime._verify_runtime(runtime_dir)

    cv2.setNumThreads(2)
    try:
        opencv_worker_runtime._verify_runtime(runtime_dir)
    except RuntimeError as exc:
        assert "OpenCV thread count changed" in str(exc)
    else:
        raise AssertionError("OpenCV thread drift was accepted")
    finally:
        cv2.setNumThreads(REQUIRED_OPENCV_THREAD_COUNT)

    cv2.ocl.setUseOpenCL(True)
    try:
        opencv_worker_runtime._verify_runtime(runtime_dir)
    except RuntimeError as exc:
        assert "OpenCV OpenCL was enabled" in str(exc)
    else:
        raise AssertionError("OpenCV OpenCL drift was accepted")
    finally:
        cv2.ocl.setUseOpenCL(False)


def test_ros_qt_parent_imports_no_opencv_module():
    environment = os.environ.copy()
    runtime_dir = str(_opencv_runtime_dir())
    python_paths = [
        path
        for path in environment.get("PYTHONPATH", "").split(os.pathsep)
        if path and str(Path(path).resolve()) != runtime_dir
    ]
    source_python = Path(__file__).resolve().parents[1] / "python"
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(source_python), *python_paths)
    )
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import sys; "
                "import camera_calibration_gui.main; "
                "raise SystemExit(1 if 'cv2' in sys.modules else 0)"
            ),
        ],
        check=False,
        env=environment,
        capture_output=True,
        text=True,
        timeout=20.0,
    )
    assert result.returncode == 0, result.stderr


def test_tf_listener_runs_while_sensor_callback_is_blocked():
    context = Context()
    rclpy.init(context=context)
    listener_node = Node("camera_calibration_tf_executor_test", context=context)
    publisher_node = Node("camera_calibration_tf_publisher_test", context=context)
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, listener_node)
    tf_broadcaster = TransformBroadcaster(publisher_node)
    callback_entered = threading.Event()
    release_callback = threading.Event()

    def blocked_sensor_callback(_message):
        callback_entered.set()
        release_callback.wait(timeout=3.0)

    listener_node.create_subscription(
        Empty,
        "/camera_calibration_test/blocked_sensor",
        blocked_sensor_callback,
        1,
    )
    blocked_sensor_publisher = publisher_node.create_publisher(
        Empty,
        "/camera_calibration_test/blocked_sensor",
        1,
    )
    executor = calibration_main._create_calibration_executor(listener_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        discovery_deadline = time.monotonic() + 2.0
        while (
            blocked_sensor_publisher.get_subscription_count() != 1
            and time.monotonic() < discovery_deadline
        ):
            time.sleep(0.01)
        assert blocked_sensor_publisher.get_subscription_count() == 1

        blocked_sensor_publisher.publish(Empty())
        assert callback_entered.wait(timeout=2.0)

        transform = TransformStamped()
        transform.header.frame_id = "calibration_test_base"
        transform.child_frame_id = "calibration_test_tool"
        transform.transform.rotation.w = 1.0
        transform_deadline = time.monotonic() + 2.0
        while time.monotonic() < transform_deadline:
            transform.header.stamp = publisher_node.get_clock().now().to_msg()
            tf_broadcaster.sendTransform(transform)
            if tf_buffer.can_transform(
                transform.header.frame_id,
                transform.child_frame_id,
                Time(),
            ):
                break
            time.sleep(0.01)

        assert tf_buffer.can_transform(
            transform.header.frame_id,
            transform.child_frame_id,
            Time(),
        )
        assert not release_callback.is_set()
        assert calibration_main.CALIBRATION_EXECUTOR_THREAD_COUNT == 2
    finally:
        release_callback.set()
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        tf_listener.unregister()
        listener_node.destroy_node()
        publisher_node.destroy_node()
        rclpy.shutdown(context=context)

    assert not spin_thread.is_alive()


def test_spawned_opencv_worker_has_absolute_runtime_and_clean_shutdown():
    worker = _opencv_worker()
    worker_pid = worker.pid
    try:
        assert worker_pid is not None
        assert worker.runtime["opencv_version"] == REQUIRED_OPENCV_VERSION
        assert worker.runtime["opencv_api"] == REQUIRED_OPENCV_API
        assert worker.runtime["opencv_wheel_sha256"] == REQUIRED_OPENCV_WHEEL_SHA256
        assert worker.runtime["charuco_legacy_pattern"] is True
        assert worker.runtime["hand_eye_method"] == "CALIB_HAND_EYE_TSAI"
        assert worker.runtime["opencv_thread_count"] == 1
        assert worker.runtime["opencv_opencl_enabled"] is False
        assert Path(worker.runtime["opencv_module_path"]).is_relative_to(
            _opencv_runtime_dir()
        )
        worker.check_health()
    finally:
        worker.close()

    assert worker.exit_code == 0


def test_spawned_opencv_worker_sigsegv_is_terminal_and_never_restarted():
    worker = _opencv_worker()
    worker_pid = worker.pid
    os.kill(worker_pid, signal.SIGSEGV)
    worker._process.join(timeout=5.0)
    try:
        worker.check_health()
    except OpenCvWorkerFailure as exc:
        assert exc.worker_pid == worker_pid
        assert exc.exit_code == -signal.SIGSEGV
        assert exc.signal_name == "SIGSEGV"
        assert "no retry was attempted" in str(exc)
    else:
        raise AssertionError("A dead OpenCV worker was accepted as healthy")
    finally:
        worker.close()

    assert worker.pid == worker_pid


def test_opencv_worker_timeout_is_terminal_and_terminates_same_process():
    worker = OpenCvWorkerClient.__new__(OpenCvWorkerClient)
    worker._connection = MagicMock()
    worker._connection.poll.return_value = False
    worker._process = MagicMock()
    worker._process.pid = 654
    worker._process.exitcode = -signal.SIGTERM
    worker._process.is_alive.return_value = True
    worker._failed = False
    worker._closed = False

    try:
        worker._receive_response("detect", 1, 0.0)
    except OpenCvWorkerFailure as exc:
        assert exc.worker_pid == 654
        assert "no response within 0.0 seconds" in str(exc)
        assert "no retry was attempted" in str(exc)
    else:
        raise AssertionError("An OpenCV worker response timeout was accepted")

    worker._process.terminate.assert_called_once_with()
    assert worker._failed


def test_node_records_native_worker_failure_and_clears_tf_solution():
    node = calibration_main.CalibrationNode.__new__(calibration_main.CalibrationNode)
    node._lock = threading.RLock()
    node._event_logger = MagicMock()
    node._opencv_worker = MagicMock(pid=321, exit_code=-11)
    node._fatal_error = None
    node._detection_status = "ready"
    node._latest_pose = np.eye(4, dtype=np.float64)
    node._latest_pose_time = 10.0
    node._solution = np.eye(4, dtype=np.float64)
    node._diagnostics = _diagnostics()
    node._latest_overlay = np.ones((2, 2, 3), dtype=np.uint8)
    node._last_frame_metadata = {
        "sequence": 44,
        "width": 848,
        "height": 480,
        "encoding": "rgb8",
        "step": 2544,
    }
    node.get_logger = MagicMock()
    node.get_logger.return_value.fatal = MagicMock()
    failure = OpenCvWorkerFailure(
        "worker exited",
        operation="detect",
        worker_pid=321,
        exit_code=-11,
    )

    node._hard_fail_opencv_worker(failure)

    assert "worker exited" in node._fatal_error
    assert node._solution is None
    assert node._diagnostics is None
    assert node._latest_pose is None
    record = node._event_logger.record.call_args
    assert record.args[:3] == (
        "ERROR",
        "opencv_worker_failed",
        node._fatal_error,
    )
    assert record.kwargs["operation"] == "detect"
    assert record.kwargs["worker_pid"] == 321
    assert record.kwargs["exit_code"] == -11
    assert record.kwargs["signal"] == "SIGSEGV"
    assert record.kwargs["retry_attempted"] is False
    assert record.kwargs["last_frame"]["sequence"] == 44
    node.get_logger.return_value.fatal.assert_called_once_with(node._fatal_error)


def test_single_sensor_frame_skip_clears_readiness_but_preserves_solution():
    node = calibration_main.CalibrationNode.__new__(calibration_main.CalibrationNode)
    node._lock = threading.RLock()
    node._event_logger = MagicMock()
    node.get_logger = MagicMock()
    node._last_detection_event = "ready"
    node._detection_status = "ready"
    node._camera_matrix = np.eye(3, dtype=np.float64)
    node._distortion = np.zeros(5, dtype=np.float64)
    node._camera_info_size = (4, 6)
    node._latest_pose = np.eye(4, dtype=np.float64)
    node._latest_pose_time = 10.0
    node._solution = np.eye(4, dtype=np.float64)
    node._diagnostics = _diagnostics()
    node._fatal_error = None

    node._skip_sensor_frame(
        "color_camera_info_pair_skipped",
        "one mismatched pair",
        reason="dimension_mismatch",
    )

    assert node._fatal_error is None
    assert node._solution is not None
    assert node._diagnostics is not None
    assert node._latest_pose is None
    assert node._latest_pose_time is None
    record = node._event_logger.record.call_args
    assert record.args == (
        "WARNING",
        "color_camera_info_pair_skipped",
        "one mismatched pair",
    )
    assert record.kwargs["frame_skipped"] is True
    assert record.kwargs["next_valid_frame_required"] is True
    node.get_logger.return_value.warning.assert_called_once_with("one mismatched pair")


def test_color_callback_detects_without_any_depth_publisher_or_cache():
    node = calibration_main.CalibrationNode.__new__(calibration_main.CalibrationNode)
    node._lock = threading.RLock()
    node._settings = CharucoSettings("bin_camera", "DICT_4X4_50", 3, 3, 28.0, 21.0)
    node._camera_matrix = np.eye(3, dtype=np.float64)
    node._distortion = np.zeros(5, dtype=np.float64)
    node._camera_info_size = (4, 6)
    node._solution = None
    node._diagnostics = None
    node._reference_frame = "base_link"
    node._fatal_error = None
    node._frame_sequence = 0
    node._opencv_worker = MagicMock()
    node._opencv_worker.detect.return_value = DetectionResult(
        1, "ready", "four corners", 4, np.eye(4), np.zeros((4, 6, 3), dtype=np.uint8)
    )
    node._set_detection_status = MagicMock()
    node._skip_sensor_frame = MagicMock()
    message = calibration_main.Image()
    message.header.frame_id = "bin_camera_color_optical_frame"
    message.header.stamp.sec = 1
    message.height, message.width = 4, 6
    message.encoding, message.is_bigendian, message.step = "rgb8", 0, 18
    message.data = bytes(72)
    node._on_color_image(message)
    node._opencv_worker.detect.assert_called_once()
    assert set(node._opencv_worker.detect.call_args.kwargs) == {
        "frame_sequence", "image_rgb", "camera_matrix", "distortion", "solution_overlay"
    }
    assert node._target_gate_locked()[0]
    assert node._latest_corner_count == 4
    assert not hasattr(node, "_depth_subscription")
    assert not hasattr(calibration_main.CalibrationNode, "_on_depth_image")
    assert np.array_equal(node.latest_overlay(), np.zeros((4, 6, 3), dtype=np.uint8))
    message.width = 5
    message.step = 15
    message.data = bytes(60)
    node._on_color_image(message)
    assert node._opencv_worker.detect.call_count == 1
    assert node._skip_sensor_frame.call_args.kwargs["reason"] == "dimension_mismatch"


def test_rgb8_ros_image_decode_is_strict_and_owned():
    message = calibration_main.Image()
    message.height = 2
    message.width = 3
    message.encoding = "rgb8"
    message.is_bigendian = 0
    message.step = 9
    message.data = bytes(range(18))

    decoded = calibration_main._rgb8_image_from_message(message)

    assert decoded.shape == (2, 3, 3)
    assert decoded.dtype == np.uint8
    assert decoded.flags.c_contiguous
    assert decoded.flags.owndata
    message.encoding = "bgr8"
    try:
        calibration_main._rgb8_image_from_message(message)
    except ValueError as exc:
        assert "required exact encoding 'rgb8'" in str(exc)
    else:
        raise AssertionError("A noncanonical camera encoding was accepted")


def test_malformed_detector_result_is_rejected_as_terminal_protocol_drift():
    malformed = DetectionResult(
        frame_sequence=8,
        state="ready",
        message="invalid synthetic result",
        corner_count=4,
        camera_from_target=None,
        overlay_rgb=np.zeros((2, 3, 3), dtype=np.uint8),
    )
    try:
        _validate_detection_result(
            malformed,
            frame_sequence=8,
            image_shape=(2, 3, 3),
        )
    except ValueError as exc:
        assert "four corners and a rigid RGB pose" in str(exc)
    else:
        raise AssertionError("A malformed ready detector result was accepted")


def test_invalid_native_calibration_pose_has_terminal_error_type():
    invalid = np.eye(4, dtype=np.float64)
    invalid[0, 0] = 2.0
    try:
        calibration_core._validate_solution(invalid)
    except InvalidPoseError as exc:
        response = opencv_worker_runtime._error_response("solve", 1, exc)
        assert response["error_type"] == "InvalidPoseError"
        operation_error = OpenCvWorkerOperationError(
            "solve",
            response["error_type"],
            response["error"],
        )
        assert operation_error.error_type == "InvalidPoseError"
    else:
        raise AssertionError("An invalid native calibration pose was accepted")


def test_worker_native_detector_rejects_mismatched_corner_ids():
    settings = CharucoSettings(
        "bin_camera",
        "DICT_4X4_50",
        3,
        3,
        28.0,
        21.0,
    )

    class MalformedDetector:
        @staticmethod
        def setCharucoParameters(_parameters):
            return None

        @staticmethod
        def detectBoard(_gray):
            corners = [np.zeros((1, 1, 2), dtype=np.float32)]
            identifiers = np.array([[0], [1]], dtype=np.int32)
            return corners, identifiers, (), None

    context = opencv_worker_runtime._DetectorContext(
        settings,
        None,
        MalformedDetector(),
    )
    try:
        opencv_worker_runtime.detect_frame(
            {
                "frame_sequence": 1,
                "image_rgb": np.zeros((8, 8, 3), dtype=np.uint8),
                "camera_matrix": np.eye(3, dtype=np.float64),
                "distortion": np.zeros(5, dtype=np.float64),
                "solution_overlay": None,
            },
            context,
        )
    except ValueError as exc:
        assert "mismatched ChArUco corners and IDs" in str(exc)
    else:
        raise AssertionError("Mismatched native detector arrays were accepted")


def test_spawned_worker_alternates_full_partial_and_empty_charuco_frames():
    settings = CharucoSettings(
        "bin_camera",
        "DICT_4X4_50",
        3,
        3,
        28.0,
        21.0,
    )
    _dictionary, board = opencv_worker_runtime.create_legacy_charuco_board(settings)
    board_image = board.generateImage((600, 600), marginSize=40)
    board_rgb = np.repeat(board_image[:, :, None], 3, axis=2)
    full = np.full((1080, 1920, 3), 255, dtype=np.uint8)
    full[240:840, 660:1260] = board_rgb
    partial = full.copy()
    partial[:, 960:] = 255
    empty = np.full((1080, 1920, 3), 255, dtype=np.uint8)
    blurred = cv2.GaussianBlur(full, (5, 5), 0.8)
    shifted = np.full_like(full, 255)
    shifted[260:860, 700:1300] = board_rgb
    edge_clipped = np.full_like(full, 255)
    edge_clipped[240:840, 0:300] = board_rgb[:, 300:]
    camera_matrix = np.array(
        [[1500.0, 0.0, 960.0], [0.0, 1500.0, 540.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    distortion = np.zeros(5, dtype=np.float64)
    worker = _opencv_worker()
    try:
        worker.configure(settings)
        states = []
        frames = (full, partial, empty, blurred, shifted, edge_clipped)
        for sequence in range(1, 61):
            image = frames[(sequence - 1) % len(frames)]
            result = worker.detect(
                frame_sequence=sequence,
                image_rgb=image,
                camera_matrix=camera_matrix,
                distortion=distortion,
                solution_overlay=None,
            )
            states.append(result.state)
            assert result.frame_sequence == sequence
            assert result.overlay_rgb.shape == image.shape
            assert result.overlay_rgb.flags.owndata
        assert "ready" in states
        assert "not_visible" in states
        assert set(states).issubset(
            {
                "ready",
                "insufficient_corners",
                "not_visible",
            }
        )
    finally:
        worker.close()

    assert worker.exit_code == 0


def test_spawned_worker_detects_exact_four_5x5_bin_markers():
    settings = ArucoMarkerSettings("DICT_5X5_50", 40.0)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    image = np.full((900, 1200), 255, dtype=np.uint8)
    placements = {
        0: (100, 100),
        1: (900, 100),
        2: (900, 600),
        3: (100, 600),
    }
    for marker_id, (left, top) in placements.items():
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 180)
        image[top:top + 180, left:left + 180] = marker
    image_rgb = np.repeat(image[:, :, None], 3, axis=2)
    camera_matrix = np.array(
        [[1000.0, 0.0, 600.0], [0.0, 1000.0, 450.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    worker = _opencv_worker()
    try:
        worker.configure_aruco(settings)
        result = worker.detect_aruco(
            frame_sequence=1,
            image_rgb=image_rgb,
            camera_matrix=camera_matrix,
            distortion=np.zeros(5, dtype=np.float64),
        )
        assert result.state == "ready"
        assert result.detected_ids == (0, 1, 2, 3)
        assert tuple(item.marker_id for item in result.observations) == (0, 1, 2, 3)
        assert all(item.image_corners_px.shape == (4, 2) for item in result.observations)
        assert all(item.optical_corners_m.shape == (4, 3) for item in result.observations)
        assert result.overlay_rgb.flags.owndata
    finally:
        worker.close()


def test_modern_charuco_pose_and_identifiers_match_known_board_geometry():
    settings = CharucoSettings("bin_camera", "DICT_4X4_50", 3, 3, 28.0, 21.0)
    _dictionary, board = opencv_worker_runtime.create_legacy_charuco_board(settings)
    board_image = board.generateImage((800, 800), marginSize=50)
    context = opencv_worker_runtime.create_detector_context(settings)
    _, charuco_ids, _, marker_ids = context.detector.detectBoard(board_image)
    assert sorted(charuco_ids.reshape(-1).tolist()) == [0, 1, 2, 3]
    assert sorted(marker_ids.reshape(-1).tolist()) == [0, 1, 2, 3]
    assert (
        context.detector.getDetectorParameters().cornerRefinementMethod
        == cv2.aruco.CORNER_REFINE_NONE
    )
    params = context.detector.getCharucoParameters()
    assert params.minMarkers == 2 and not params.tryRefineMarkers

    image_rgb = np.repeat(board_image[:, :, None], 3, axis=2)
    camera_matrix = np.array(
        [[2000.0, 0.0, 400.0], [0.0, 2000.0, 400.0], [0.0, 0.0, 1.0]], dtype=np.float64
    )
    with patch.object(cv2, "drawFrameAxes", wraps=cv2.drawFrameAxes) as axes:
        result = opencv_worker_runtime.detect_frame(
            dict(frame_sequence=1, image_rgb=image_rgb, camera_matrix=camera_matrix,
                 distortion=np.zeros(5, dtype=np.float64), solution_overlay=None),
            context,
        )
    assert result.state == "ready" and result.corner_count == 4
    assert np.linalg.norm(result.camera_from_target[:3, 3] - [-0.042, -0.042, 0.240]) < 0.002
    assert rotation_angle_deg(result.camera_from_target[:3, :3]) < 0.2  # Raster quantization.
    assert np.any(result.overlay_rgb != image_rgb)
    axes.assert_called_once()
    assert np.allclose(axes.call_args.args[4].reshape(3), result.camera_from_target[:3, 3])
    assert np.allclose(cv2.Rodrigues(axes.call_args.args[3])[0], result.camera_from_target[:3, :3])
    assert abs(axes.call_args.args[5] - 0.042) < 1e-8


def test_spawned_worker_owns_hand_eye_solve_and_diagnostics():
    samples, expected = _synthetic_problem(CAMERA_TO_HAND, range(5))
    worker = _opencv_worker()
    try:
        result = worker.solve(
            calibration_mode=CAMERA_TO_HAND,
            samples=samples,
            camera_link_from_optical=np.eye(4, dtype=np.float64),
            previous_solution=None,
            previous_quality=None,
        )
        translation_error = np.linalg.norm(
            result.reference_from_camera_link[:3, 3] - expected[:3, 3]
        )
        rotation_error = rotation_angle_deg(
            result.reference_from_camera_link[:3, :3].T @ expected[:3, :3]
        )
        assert translation_error < 1e-6
        assert rotation_error < 1e-5
        assert result.diagnostics.fit.sample_count == 5
        assert result.diagnostics.leave_one_out.status == "not_available"
    finally:
        worker.close()

    assert worker.exit_code == 0


def test_worker_runtime_is_independent_from_the_ros_executor_thread():
    worker = _opencv_worker()
    try:
        assert worker.runtime["opencv_thread_count"] == REQUIRED_OPENCV_THREAD_COUNT
        assert worker.runtime["opencv_version"] == REQUIRED_OPENCV_VERSION
        assert worker.runtime["opencv_opencl_enabled"] is False
    finally:
        worker.close()


def test_modern_charuco_detector_survives_repeated_frames():
    opencv_worker_runtime.configure_opencv_runtime(_opencv_runtime_dir())
    settings = CharucoSettings(
        "bin_camera",
        "DICT_4X4_50",
        3,
        3,
        28.0,
        21.0,
    )
    _dictionary, board = opencv_worker_runtime.create_legacy_charuco_board(settings)
    board_image = board.generateImage((300, 300), marginSize=20)
    context = opencv_worker_runtime.create_detector_context(settings)
    camera_matrix = np.array(
        [[750.0, 0.0, 320.0], [0.0, 750.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    distortion = np.zeros(5, dtype=np.float64)
    detected_frames = 0
    for index in range(500):
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        x_offset = 165 + ((index % 7) - 3)
        y_offset = 85 + (((index // 7) % 7) - 3)
        frame[y_offset:y_offset + 300, x_offset:x_offset + 300] = np.repeat(
            board_image[:, :, None],
            3,
            axis=2,
        )
        if index % 3 == 0:
            frame = cv2.GaussianBlur(frame, (3, 3), 0.4)
        result = opencv_worker_runtime.detect_frame(
            {
                "frame_sequence": index + 1,
                "image_rgb": frame,
                "camera_matrix": camera_matrix,
                "distortion": distortion,
                "solution_overlay": None,
            },
            context,
        )
        if result.state == "ready":
            detected_frames += 1
    assert detected_frames == 500


def test_target_gate_accepts_latest_valid_pose_immediately_without_stability_wait():
    node = calibration_main.CalibrationNode.__new__(calibration_main.CalibrationNode)
    node._settings = object()
    node._latest_pose_time = 100.0
    node._latest_corner_count = 4
    node._detection_status = "ready"
    node._latest_pose = np.eye(4, dtype=np.float64)
    assert not hasattr(node, "_pose_history")
    with patch.object(calibration_main.time, "monotonic", return_value=100.0):
        ready, status = node._target_gate_locked()
    assert ready and "stationary" in status
    with patch.object(calibration_main.time, "monotonic", return_value=100.5):
        assert node._target_gate_locked()[0]
    with patch.object(calibration_main.time, "monotonic", return_value=100.501):
        assert not node._target_gate_locked()[0]


def test_manual_prefix_and_charuco_geometry_are_strict():
    settings = CharucoSettings(
        camera_prefix="camera_one",
        dictionary_name="DICT_4X4_50",
        squares_x=5,
        squares_y=7,
        square_length_mm=30.0,
        marker_length_mm=22.0,
    )
    settings.validate()
    assert settings.color_topic == "/camera_one/color/image_raw"
    assert not hasattr(settings, "depth_topic")
    assert settings.camera_info_topic == "/camera_one/color/camera_info"
    assert settings.optical_frame == "camera_one_color_optical_frame"
    assert settings.camera_link_frame == "camera_one_link"
    _dictionary, board = opencv_worker_runtime.create_legacy_charuco_board(settings)
    board_image = board.generateImage((600, 840))
    detector = opencv_worker_runtime.create_detector_context(settings).detector
    _charuco_corners, _charuco_ids, marker_corners, marker_ids = (
        detector.detectBoard(board_image)
    )
    assert marker_corners
    assert marker_ids is not None

    invalid = CharucoSettings("", "DICT_4X4_50", 5, 7, 30.0, 22.0)
    try:
        invalid.validate()
    except ValueError as exc:
        assert "Camera prefix is required" in str(exc)
    else:
        raise AssertionError("Empty manual prefix was accepted")

    leading_slash = CharucoSettings("/camera_one", "DICT_4X4_50", 5, 7, 30.0, 22.0)
    try:
        leading_slash.validate()
    except ValueError as exc:
        assert "Camera prefix is required" in str(exc)
    else:
        raise AssertionError("Camera prefix was silently normalized")


def test_last_session_ui_state_is_atomic_strict_and_overwritten(tmp_path):
    path = calibration_ui_state_path(tmp_path)
    assert load_calibration_ui_state(path) is None
    settings = CharucoSettings(
        camera_prefix="bin_camera",
        dictionary_name="DICT_4X4_50",
        squares_x=3,
        squares_y=3,
        square_length_mm=30.0,
        marker_length_mm=22.0,
    )
    saved_at = datetime(2026, 9, 3, 8, 0, 0, tzinfo=timezone.utc)
    written = write_calibration_ui_state(
        path,
        CAMERA_TO_HAND,
        settings,
        saved_at=saved_at,
    )
    assert path == tmp_path / "logs" / "camera_calibration" / "last_session.json"
    assert written.saved_at_utc == "2026-09-03T08:00:00.000000Z"
    assert written.minimum_samples == MINIMUM_CALIBRATION_SAMPLES
    assert load_calibration_ui_state(path) == written

    replacement = write_calibration_ui_state(
        path,
        CAMERA_ON_HAND,
        CharucoSettings(
            camera_prefix="robot_camera",
            dictionary_name="DICT_5X5_100",
            squares_x=6,
            squares_y=8,
            square_length_mm=35.0,
            marker_length_mm=25.0,
        ),
        saved_at=datetime(2026, 9, 3, 9, 0, 0, tzinfo=timezone.utc),
    )
    assert load_calibration_ui_state(path) == replacement
    assert replacement.settings.camera_prefix == "robot_camera"
    assert replacement.minimum_samples == MINIMUM_CALIBRATION_SAMPLES

    wrong_minimum = json.loads(path.read_text(encoding="utf-8"))
    wrong_minimum["minimum_samples"] = 3
    path.write_text(json.dumps(wrong_minimum), encoding="utf-8")
    try:
        load_calibration_ui_state(path)
    except ValueError as exc:
        assert "minimum_samples must be exactly 5" in str(exc)
    else:
        raise AssertionError("A noncanonical calibration minimum was accepted")

    write_calibration_ui_state(
        path,
        CAMERA_ON_HAND,
        replacement.settings,
        saved_at=datetime(2026, 9, 3, 9, 30, 0, tzinfo=timezone.utc),
    )

    malformed = json.loads(path.read_text(encoding="utf-8"))
    malformed["unsupported"] = True
    path.write_text(json.dumps(malformed), encoding="utf-8")
    try:
        load_calibration_ui_state(path)
    except ValueError as exc:
        assert "canonical keys" in str(exc)
    else:
        raise AssertionError("Malformed last-session UI state was accepted")


def test_gui_restores_last_session_as_unapplied_prefill(tmp_path):
    settings = CharucoSettings(
        camera_prefix="bin_camera",
        dictionary_name="DICT_4X4_100",
        squares_x=4,
        squares_y=5,
        square_length_mm=28.0,
        marker_length_mm=21.0,
    )
    state = CalibrationUiState(
        saved_at_utc="2026-09-03T08:00:00.000000Z",
        calibration_mode=CAMERA_TO_HAND,
        settings=settings,
        minimum_samples=MINIMUM_CALIBRATION_SAMPLES,
    )

    class FakeNode:
        ui_state_path = tmp_path / "logs" / "camera_calibration" / "last_session.json"
        configure_calls = 0
        automatic = SimpleNamespace(
            active=False, capturing=False, started=False, complete=False, recipe=None,
            stopping=False, message="Load a calibration", stop=MagicMock())

        @staticmethod
        def _automatic_busy():
            return False

        @staticmethod
        def load_last_session():
            return state

        @staticmethod
        def save():
            return True, "Saved calibration YAML: /tmp/calibration.yaml"

    with patch.dict(os.environ, {"QT_QPA_PLATFORM": "offscreen"}):
        application = (
            calibration_main.QtWidgets.QApplication.instance()
            or calibration_main.QtWidgets.QApplication([])
        )
        window = calibration_main.CalibrationWindow(FakeNode())
        try:
            assert window.calibration_mode.currentData() == CAMERA_TO_HAND
            assert window.camera_prefix.text() == "bin_camera"
            assert window.dictionary.currentText() == "DICT_4X4_100"
            assert window.squares_x.value() == 4
            assert window.squares_y.value() == 5
            assert window.square_length.value() == 28.0
            assert window.marker_length.value() == 21.0
            assert not hasattr(window, "minimum_corners")
            assert window.minimum_samples.text() == "5 (fixed; automatic solve)"
            assert not hasattr(window, "compute_button")
            assert not window._configuration_applied
            assert not window.capture_button.isEnabled()
            assert FakeNode.configure_calls == 0
            window._configuration_applied = True
            window._set_action_states(True, True, 3, True, True)
            assert window.capture_button.isEnabled()
            assert window.load_button.text() == "Load Calibration"
            assert window.load_button.isEnabled()
            assert window.save_button.isEnabled()
            assert window.overlay.minimumWidth() == 480
            assert not hasattr(window, "depth_overlay")
            assert window.sample_table.columnCount() == 6
            window._refresh_sample_table(
                (
                    {
                        "sample_id": "C1",
                        "translation_residual_mm": 1.25,
                        "rotation_residual_deg": 0.5,
                        "leave_one_out_translation_mm": None,
                        "leave_one_out_rotation_deg": None,
                        "flags": ("MAX FIT T",),
                    },
                )
            )
            assert window.sample_table.rowCount() == 1
            assert window.sample_table.item(0, 0).text() == "C1"
            assert window.sample_table.item(0, 0).font().bold()
            window.sample_table.selectRow(0)
            application.processEvents()
            assert window.remove_button.isEnabled()
            with patch.object(
                calibration_main.QtWidgets.QMessageBox,
                "information",
            ) as confirmation:
                window._save()
            confirmation.assert_called_once_with(
                window,
                "Calibration saved",
                "Saved calibration YAML: /tmp/calibration.yaml",
            )
        finally:
            window._timer.stop()
            window.close()
            application.processEvents()


def test_synthetic_camera_to_hand_solution_recovers_base_from_camera():
    base_from_camera = _transform([0.15, -0.25, 0.10], [0.65, -0.30, 1.20])
    tool_from_target = _transform([-0.05, 0.03, 0.12], [0.02, 0.01, 0.18])
    poses = [
        ([0.00, 0.00, 0.00], [0.45, -0.25, 0.40]),
        ([0.20, -0.10, 0.10], [0.52, -0.10, 0.55]),
        ([-0.15, 0.25, -0.05], [0.35, 0.05, 0.62]),
        ([0.30, 0.15, -0.20], [0.58, 0.15, 0.48]),
        ([-0.25, -0.20, 0.18], [0.42, -0.18, 0.72]),
        ([0.12, 0.32, 0.25], [0.62, 0.02, 0.66]),
        ([-0.30, 0.10, -0.22], [0.30, -0.02, 0.52]),
        ([0.18, -0.28, 0.30], [0.55, -0.30, 0.60]),
    ]
    samples = []
    for index, (rotation_vector, translation) in enumerate(
        poses[:MINIMUM_CALIBRATION_SAMPLES],
        start=1,
    ):
        base_from_tool = _transform(rotation_vector, translation)
        base_from_target = base_from_tool @ tool_from_target
        camera_from_target = invert_transform(base_from_camera) @ base_from_target
        samples.append(_sample(f"C{index}", base_from_tool, camera_from_target))

    solved, quality = solve_camera_to_hand(samples)
    translation_error = np.linalg.norm(solved[:3, 3] - base_from_camera[:3, 3])
    rotation_error = rotation_angle_deg(solved[:3, :3].T @ base_from_camera[:3, :3])
    assert translation_error < 1e-6
    assert rotation_error < 1e-5
    assert quality.sample_count == len(samples)


def test_synthetic_camera_on_hand_solution_recovers_tool_from_camera():
    tool_from_camera = _transform([0.08, -0.12, 0.04], [0.04, 0.01, 0.09])
    base_from_target = _transform([-0.10, 0.05, 0.20], [0.70, 0.15, 0.35])
    poses = [
        ([0.00, 0.00, 0.00], [0.45, -0.25, 0.40]),
        ([0.20, -0.10, 0.10], [0.52, -0.10, 0.55]),
        ([-0.15, 0.25, -0.05], [0.35, 0.05, 0.62]),
        ([0.30, 0.15, -0.20], [0.58, 0.15, 0.48]),
        ([-0.25, -0.20, 0.18], [0.42, -0.18, 0.72]),
        ([0.12, 0.32, 0.25], [0.62, 0.02, 0.66]),
        ([-0.30, 0.10, -0.22], [0.30, -0.02, 0.52]),
        ([0.18, -0.28, 0.30], [0.55, -0.30, 0.60]),
    ]
    samples = []
    for index, (rotation_vector, translation) in enumerate(
        poses[:MINIMUM_CALIBRATION_SAMPLES],
        start=1,
    ):
        base_from_tool = _transform(rotation_vector, translation)
        camera_from_target = (
            invert_transform(tool_from_camera)
            @ invert_transform(base_from_tool)
            @ base_from_target
        )
        samples.append(_sample(f"C{index}", base_from_tool, camera_from_target))

    solved, quality = solve_camera_on_hand(samples)
    translation_error = np.linalg.norm(solved[:3, 3] - tool_from_camera[:3, 3])
    rotation_error = rotation_angle_deg(solved[:3, :3].T @ tool_from_camera[:3, :3])
    assert translation_error < 1e-6
    assert rotation_error < 1e-5
    assert quality.sample_count == len(samples)


def test_fit_diagnostics_identify_worst_translation_and_rotation_samples():
    transforms = [np.eye(4, dtype=np.float64) for _index in range(5)]
    transforms[2] = _transform([0.0, 0.0, 0.0], [0.012, 0.0, 0.0])
    transforms[3] = _transform([0.0, 0.0, math.radians(8.0)], [0.0, 0.0, 0.0])
    quality = calibration_quality_from_transforms(
        ["C1", "C2", "C3", "C4", "C5"],
        transforms,
    )

    assert quality.max_translation_sample_id == "C3"
    assert quality.max_rotation_sample_id == "C4"
    assert quality.translation_rms_mm > 0.0
    assert quality.rotation_rms_deg > 0.0


def test_solution_change_and_pose_coverage_use_physical_units():
    previous = np.eye(4, dtype=np.float64)
    current = _transform([0.0, 0.0, math.radians(2.0)], [0.003, 0.004, 0.0])
    change = solution_change_diagnostics(
        previous,
        _quality(3, 1.0, 0.5),
        current,
        _quality(4, 0.8, 0.3),
    )
    assert change.available
    assert abs(change.translation_rms_delta_mm + 0.2) < 1e-12
    assert abs(change.rotation_rms_delta_deg + 0.2) < 1e-12
    assert abs(change.camera_translation_delta_mm - 5.0) < 1e-9
    assert abs(change.camera_rotation_delta_deg - 2.0) < 1e-9

    samples = [
        _sample("C1"),
        _sample(
            "C2",
            _transform([0.0, 0.0, math.radians(10.0)], [0.03, 0.04, 0.0]),
        ),
    ]
    coverage = pose_coverage_diagnostics(samples)
    assert abs(coverage.translation_span_mm - 50.0) < 1e-9
    assert abs(coverage.rotation_span_deg - 10.0) < 1e-9


def test_leave_one_out_for_both_modes():
    for calibration_mode in (CAMERA_TO_HAND, CAMERA_ON_HAND):
        calibration_samples, expected = _synthetic_problem(
            calibration_mode,
            range(6),
        )
        solved, _quality_result = (
            solve_camera_to_hand(calibration_samples)
            if calibration_mode == CAMERA_TO_HAND
            else solve_camera_on_hand(calibration_samples)
        )
        leave_one_out = leave_one_out_diagnostics(
            calibration_mode,
            calibration_samples,
            np.eye(4, dtype=np.float64),
            solved,
        )
        assert leave_one_out.status == "valid"
        assert len(leave_one_out.entries) == len(calibration_samples)
        assert leave_one_out.translation_rms_mm < 1e-3
        assert leave_one_out.rotation_rms_deg < 1e-3

        solved_again, _quality_again = (
            solve_camera_to_hand(calibration_samples)
            if calibration_mode == CAMERA_TO_HAND
            else solve_camera_on_hand(calibration_samples)
        )
        assert np.allclose(solved_again, solved)
        assert np.allclose(solved, expected, atol=1e-6)


def test_leave_one_out_failure_blocks_save_and_identifies_omission(tmp_path):
    samples, _expected = _synthetic_problem(CAMERA_TO_HAND, range(6))

    def solve_subset(_mode, subset):
        if all(sample.sample_id != "C2" for sample in subset):
            raise ValueError("synthetic weak geometry")
        return np.eye(4, dtype=np.float64), _quality(5, 0.0, 0.0)

    with patch.object(calibration_core, "solve_calibration", side_effect=solve_subset):
        result = leave_one_out_diagnostics(
            CAMERA_TO_HAND,
            samples,
            np.eye(4, dtype=np.float64),
            np.eye(4, dtype=np.float64),
        )
    assert result.status == "failed"
    assert result.failed_sample_ids == ("C2",)
    blocked_diagnostics = _diagnostics(leave_one_out=result)
    assert not blocked_diagnostics.save_allowed
    settings = CharucoSettings(
        "bin_camera", "DICT_4X4_50", 5, 7, 30.0, 22.0
    )
    try:
        write_calibration_yaml(
            tmp_path / "blocked.yaml",
            CAMERA_TO_HAND,
            settings,
            np.eye(4, dtype=np.float64),
            blocked_diagnostics,
            samples,
        )
    except ValueError as exc:
        assert "leave-one-out diagnostics fail" in str(exc)
    else:
        raise AssertionError("YAML was written despite failed leave-one-out diagnostics")


def test_manual_calibration_removal_below_five_clears_solution():
    node = calibration_main.CalibrationNode.__new__(calibration_main.CalibrationNode)
    node._lock = __import__("threading").RLock()
    node._event_logger = MagicMock()
    node._minimum_samples = MINIMUM_CALIBRATION_SAMPLES
    node._calibration_samples = [_sample(f"C{index}") for index in range(1, 6)]
    node._next_calibration_sample_number = 6
    node._settings = CharucoSettings("camera", "DICT_4X4_50", 3, 3, 28.0, 21.0)
    node._calibration_mode = CAMERA_TO_HAND
    node._reference_frame = "base_link"
    node._publish_solution_preview = MagicMock()
    node._solution = np.eye(4, dtype=np.float64)
    node._diagnostics = _diagnostics()
    node._latest_overlay = np.ones((2, 2, 3), dtype=np.uint8)

    success, message = node.remove_sample("C2")

    assert success
    assert "Removed C2" in message
    assert [sample.sample_id for sample in node._calibration_samples] == ["C1", "C3", "C4", "C5"]
    assert node._solution is None
    assert node._diagnostics is None
    assert node._latest_overlay is None
    node._rebroadcast_solution_preview()
    node._publish_solution_preview.assert_not_called()
    assert not node.save()[0]
    assert node._next_calibration_sample_number == 6


def test_pose_diversity_and_calibration_ids_are_strict_and_non_reused():
    class FakeClock:
        @staticmethod
        def now():
            return calibration_main.Time(seconds=10.0, clock_type=ClockType.ROS_TIME)

    class FakeTfBuffer:
        translation_x = 0.0
        rotation_deg = 0.0

        @classmethod
        def lookup_transform(cls, *_args, **_kwargs):
            transform = calibration_main.TransformStamped()
            transform.header.stamp = calibration_main.Time(
                seconds=9.5,
                clock_type=ClockType.ROS_TIME,
            ).to_msg()
            transform.transform.translation.x = cls.translation_x
            transform.transform.rotation.w = math.cos(math.radians(cls.rotation_deg) / 2)
            transform.transform.rotation.z = math.sin(math.radians(cls.rotation_deg) / 2)
            return transform

    node = calibration_main.CalibrationNode.__new__(calibration_main.CalibrationNode)
    node._lock = __import__("threading").RLock()
    node._event_logger = MagicMock()
    node._latest_pose = np.eye(4, dtype=np.float64)
    node._latest_corner_count = 8
    node._latest_joint_positions = tuple(float(index) / 10.0 for index in range(6))
    node._latest_joint_state_stamp = calibration_main.Time(
        seconds=9.5,
        clock_type=ClockType.ROS_TIME,
    )
    node._target_gate_locked = MagicMock(return_value=(True, "ready"))
    node._tf_buffer = FakeTfBuffer()
    node.get_clock = MagicMock(return_value=FakeClock())
    node._calibration_samples = [_sample("C1")]
    node._next_calibration_sample_number = 2
    node._solution = np.eye(4, dtype=np.float64)

    success, message, sample, _age, _corners = node._capture_observation()
    assert not success
    assert "Robot orientation is too similar to C1" in message
    assert sample is None

    FakeTfBuffer.translation_x = 0.1
    assert not node._capture_observation()[0]  # Translation alone must not qualify.
    FakeTfBuffer.rotation_deg = 10.0
    assert not node._capture_observation()[0]  # Board rotation is independently required.
    node._latest_pose = _transform([0.0, 0.0, math.radians(10)], [0.0, 0.0, 1.0])
    success, _message, sample, _age, _corners = node._capture_observation()
    assert success
    assert sample.sample_id == "C2"
    assert sample.joint_positions_rad == tuple(
        float(index) / 10.0 for index in range(6)
    )
    node._calibration_samples.append(sample)

    FakeTfBuffer.translation_x = 0.2
    FakeTfBuffer.rotation_deg = 20.0
    node._latest_pose = _transform([0.0, 0.0, math.radians(20)], [0.0, 0.0, 1.0])
    success, _message, sample, _age, _corners = node._capture_observation()
    assert success
    assert sample.sample_id == "C3"
    node._calibration_samples.append(sample)
    node._calibration_samples.pop(1)

    FakeTfBuffer.translation_x = 0.3
    FakeTfBuffer.rotation_deg = 30.0
    node._latest_pose = _transform([0.0, 0.0, math.radians(30)], [0.0, 0.0, 1.0])
    success, _message, sample, _age, _corners = node._capture_observation()
    assert success
    assert sample.sample_id == "C4"


def test_joint_state_capture_contract_is_canonical_fresh_and_finite():
    node = calibration_main.CalibrationNode.__new__(calibration_main.CalibrationNode)
    node._lock = threading.RLock()
    node._event_logger = MagicMock()
    node._latest_joint_positions = None
    node._latest_joint_state_stamp = None

    valid = calibration_main.JointState()
    valid.header.stamp = calibration_main.Time(
        seconds=9.5,
        clock_type=ClockType.ROS_TIME,
    ).to_msg()
    valid.name = list(calibration_core.JOINT_NAMES)
    valid.position = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    node._on_joint_state(valid)
    assert node._latest_joint_positions == (
        0.1,
        -0.2,
        0.3,
        -0.4,
        0.5,
        -0.6,
    )
    assert node._latest_joint_state_stamp.nanoseconds > 0

    invalid = calibration_main.JointState()
    invalid.header.stamp = valid.header.stamp
    invalid.name = list(reversed(calibration_core.JOINT_NAMES))
    invalid.position = valid.position
    node._on_joint_state(invalid)
    assert node._latest_joint_positions is None
    assert node._latest_joint_state_stamp is None
    assert node._event_logger.record.call_args.args[1] == "joint_state_rejected"


def test_calibrated_pose_overlay_reports_reference_pose_and_quality():
    pose = _transform([0.0, 0.0, math.radians(90.0)], [0.1, -0.2, 0.3])
    quality = _quality()
    roll, pitch, yaw = rotation_matrix_to_rpy_deg(pose[:3, :3])
    assert abs(roll) < 1e-9
    assert abs(pitch) < 1e-9
    assert abs(yaw - 90.0) < 1e-9

    lines = calibration_main._solution_overlay_lines(
        "base_link",
        "bin_camera_link",
        pose,
        quality,
    )
    assert lines == (
        "CALIBRATED TF | 5 samples",
        "base_link <- bin_camera_link",
        "XYZ [m]: +0.1000, -0.2000, +0.3000",
        "RPY [deg]: +0.00, -0.00, +90.00",
        "FIT RMS: 0.250 mm / 0.100 deg",
    )
    assert calibration_main._solution_overlay_font_scale(640) == 0.8
    assert calibration_main._solution_overlay_font_scale(848) > 0.99
    assert calibration_main._solution_overlay_font_scale(1920) == 1.2
    assert calibration_main.SOLUTION_OVERLAY_TEXT_THICKNESS == 2
    image = np.zeros((480, 848, 3), dtype=np.uint8)
    opencv_worker_runtime._draw_solution_overlay(
        image,
        "base_link",
        "bin_camera_link",
        pose,
        quality,
    )
    assert np.any(image != 0)


def test_only_calculated_camera_tf_is_broadcast_for_rviz():
    node = calibration_main.CalibrationNode.__new__(calibration_main.CalibrationNode)
    node.get_clock = MagicMock()
    node.get_clock.return_value.now.return_value = calibration_main.Time(seconds=10.0)
    node._tf_broadcaster = MagicMock()
    pose = _transform([0.1, -0.2, 0.3], [0.4, 0.5, 0.6])

    node._publish_solution_preview("base_link", "bin_camera_link", pose)

    published = node._tf_broadcaster.sendTransform.call_args.args[0]
    assert published.header.frame_id == "base_link"
    assert published.child_frame_id == "bin_camera_link"
    assert published.transform.translation.x == 0.4
    assert published.transform.translation.y == 0.5
    assert published.transform.translation.z == 0.6
    assert not hasattr(calibration_main.CalibrationNode, "_publish_target_transform")


def test_node_compute_populates_diagnostics_and_broadcasts_preview():
    class IdentityTfBuffer:
        @staticmethod
        def lookup_transform(*_args, **_kwargs):
            transform = calibration_main.TransformStamped()
            transform.transform.rotation.w = 1.0
            return transform

    samples, _expected = _synthetic_problem(CAMERA_TO_HAND, range(5))
    node = calibration_main.CalibrationNode.__new__(calibration_main.CalibrationNode)
    node._lock = __import__("threading").RLock()
    node._settings = CharucoSettings(
        "bin_camera", "DICT_4X4_50", 5, 7, 30.0, 22.0
    )
    node._calibration_mode = CAMERA_TO_HAND
    node._reference_frame = "base_link"
    node._minimum_samples = MINIMUM_CALIBRATION_SAMPLES
    node._calibration_samples = samples
    node._tf_buffer = IdentityTfBuffer()
    node._event_logger = MagicMock()
    node._solution = None
    node._diagnostics = None
    node._latest_overlay = None
    node._publish_solution_preview = MagicMock()
    solved, quality = calibration_core.solve_calibration(CAMERA_TO_HAND, samples)
    diagnostics = AccuracyDiagnostics(
        quality,
        solution_change_diagnostics(None, None, solved, quality),
        leave_one_out_diagnostics(
            CAMERA_TO_HAND,
            samples,
            np.eye(4, dtype=np.float64),
            solved,
        ),
        pose_coverage_diagnostics(samples),
        ax_xb_diagnostics(CAMERA_TO_HAND, samples, solved),
    )
    node._opencv_worker = MagicMock()
    node._opencv_worker.solve.return_value = SolveResult(solved, diagnostics)

    success, message = node.compute()

    assert success
    assert "FIT RMS" in message
    assert node._solution is not None
    assert isinstance(node._diagnostics, AccuracyDiagnostics)
    assert node._diagnostics.leave_one_out.status == "not_available"
    assert node._diagnostics.save_allowed
    node._opencv_worker.solve.assert_called_once()
    node._publish_solution_preview.assert_called_once()


def test_node_treats_invalid_worker_solve_pose_as_terminal():
    class IdentityTfBuffer:
        @staticmethod
        def lookup_transform(*_args, **_kwargs):
            transform = calibration_main.TransformStamped()
            transform.transform.rotation.w = 1.0
            return transform

    samples, _expected = _synthetic_problem(
        CAMERA_TO_HAND,
        range(MINIMUM_CALIBRATION_SAMPLES),
    )
    node = calibration_main.CalibrationNode.__new__(calibration_main.CalibrationNode)
    node._lock = threading.RLock()
    node._settings = CharucoSettings(
        "bin_camera", "DICT_4X4_50", 5, 7, 30.0, 22.0
    )
    node._calibration_mode = CAMERA_TO_HAND
    node._reference_frame = "base_link"
    node._minimum_samples = MINIMUM_CALIBRATION_SAMPLES
    node._calibration_samples = samples
    node._tf_buffer = IdentityTfBuffer()
    node._event_logger = MagicMock()
    node._solution = np.eye(4, dtype=np.float64)
    node._diagnostics = _diagnostics()
    node._latest_overlay = np.ones((2, 2, 3), dtype=np.uint8)
    node._fatal_error = None
    node._detection_status = "ready"
    node._latest_pose = np.eye(4, dtype=np.float64)
    node._latest_pose_time = 10.0
    node._last_frame_metadata = {"sequence": 12}
    node.get_logger = MagicMock()
    node._opencv_worker = MagicMock(pid=321, exit_code=None)
    node._opencv_worker.solve.side_effect = OpenCvWorkerOperationError(
        "solve",
        "InvalidPoseError",
        "invalid native pose",
    )

    success, message = node.compute()

    assert not success
    assert "InvalidPoseError" in message
    assert node._fatal_error is not None
    assert node._solution is None
    assert node._diagnostics is None
    assert node._event_logger.record.call_args.args[1] == "opencv_worker_failed"
    node.get_logger.return_value.fatal.assert_called_once_with(node._fatal_error)


def test_node_treats_worker_runtime_drift_during_solve_as_terminal():
    class IdentityTfBuffer:
        @staticmethod
        def lookup_transform(*_args, **_kwargs):
            transform = calibration_main.TransformStamped()
            transform.transform.rotation.w = 1.0
            return transform

    samples, _expected = _synthetic_problem(
        CAMERA_TO_HAND,
        range(MINIMUM_CALIBRATION_SAMPLES),
    )
    node = calibration_main.CalibrationNode.__new__(calibration_main.CalibrationNode)
    node._lock = threading.RLock()
    node._settings = CharucoSettings(
        "bin_camera", "DICT_4X4_50", 5, 7, 30.0, 22.0
    )
    node._calibration_mode = CAMERA_TO_HAND
    node._reference_frame = "base_link"
    node._minimum_samples = MINIMUM_CALIBRATION_SAMPLES
    node._calibration_samples = samples
    node._tf_buffer = IdentityTfBuffer()
    node._event_logger = MagicMock()
    node._solution = np.eye(4, dtype=np.float64)
    node._diagnostics = _diagnostics()
    node._latest_overlay = np.ones((2, 2, 3), dtype=np.uint8)
    node._fatal_error = None
    node._detection_status = "ready"
    node._latest_pose = np.eye(4, dtype=np.float64)
    node._latest_pose_time = 10.0
    node._last_frame_metadata = {"sequence": 13}
    node.get_logger = MagicMock()
    node._opencv_worker = MagicMock(pid=321, exit_code=None)
    node._opencv_worker.solve.side_effect = OpenCvWorkerOperationError(
        "solve",
        "OpenCvRuntimeContractError",
        "OpenCV thread count changed after initialization",
    )

    success, message = node.compute()

    assert not success
    assert "OpenCvRuntimeContractError" in message
    assert node._fatal_error is not None
    assert node._solution is None
    assert node._diagnostics is None
    assert node._event_logger.record.call_args.args[1] == "opencv_worker_failed"
    node.get_logger.return_value.fatal.assert_called_once_with(node._fatal_error)


def test_capture_at_minimum_and_later_samples_automatically_recomputes():
    for existing_count in (3, 4, 5):
        node = calibration_main.CalibrationNode.__new__(calibration_main.CalibrationNode)
        node._lock = __import__("threading").RLock()
        node._minimum_samples = MINIMUM_CALIBRATION_SAMPLES
        node._event_logger = MagicMock()
        node._calibration_samples = []
        node._latest_overlay = None
        for index in range(existing_count):
            base_from_tool = np.eye(4, dtype=np.float64)
            base_from_tool[0, 3] = float(index + 1)
            node._calibration_samples.append(
                _sample(f"C{index + 1}", base_from_tool)
            )
        node._solution = np.eye(4, dtype=np.float64)
        node._diagnostics = _diagnostics(_quality(existing_count, 0.0, 0.0))
        new_sample = _sample(f"C{existing_count + 1}")
        node._capture_observation = MagicMock(
            return_value=(True, "", new_sample, 0.5, 8)
        )
        node.compute = MagicMock(return_value=(True, "Automatic calibration complete."))

        success, message = node.capture_sample()

        assert success
        assert len(node._calibration_samples) == existing_count + 1
        if existing_count < 4:
            node.compute.assert_not_called()
            assert node._solution is None and node._diagnostics is None
        else:
            node.compute.assert_called_once_with()
            assert "Automatic calibration complete." in message


def test_schema_seven_yaml_round_trip_restores_rgb_and_joint_observations(tmp_path):
    settings = CharucoSettings("camera_two", "DICT_4X4_100", 6, 8, 35.0, 25.0)
    created_at = datetime(2026, 9, 2, 12, 34, 56, 789012, tzinfo=timezone.utc)
    path = output_path_for_mode(CAMERA_ON_HAND, tmp_path, created_at)
    assert path == (
        tmp_path
        / "calibration"
        / "camera_on_hand_calibration_20260902T123456_789012Z.yaml"
    )
    samples, solved = _synthetic_problem(CAMERA_ON_HAND, range(5))
    _solution, quality = solve_camera_on_hand(samples)
    leave_one_out = leave_one_out_diagnostics(
        CAMERA_ON_HAND,
        samples,
        np.eye(4, dtype=np.float64),
        solved,
    )
    write_calibration_yaml(
        path,
        CAMERA_ON_HAND,
        settings,
        solved,
        _diagnostics(quality, leave_one_out=leave_one_out),
        samples,
        created_at=created_at,
    )
    content = path.read_text(encoding="utf-8")
    assert "schema_version: 7" in content
    assert "calibration_mode: camera_on_hand" in content
    assert "prefix: camera_two" in content
    assert "dictionary: DICT_4X4_100" in content
    assert "target_frame: Link6" in content
    assert "source_frame: camera_two_link" in content
    assert "convention: target_from_source" in content
    assert "calibration_fit:" in content
    assert "status: not_available" in content
    assert "captured_samples:" in content
    assert "joint_state_topic: /joint_states" in content
    assert "joint_position_unit: radian" in content
    assert "joint_positions_rad:" in content
    assert "depth" not in content.lower()
    assert "pipeline:" in content and "ax_xb:" in content
    assert calibration_core.MOVEIT_REFERENCE_COMMIT in content
    assert "validation" not in content.lower()

    loaded = load_calibration_yaml(path)
    assert loaded.created_at_utc == "2026-09-02T12:34:56.789012Z"
    assert loaded.calibration_mode == CAMERA_ON_HAND
    assert loaded.settings == settings
    assert np.allclose(loaded.reference_from_camera_link, solved)
    assert [sample.sample_id for sample in loaded.samples] == [
        "C1",
        "C2",
        "C3",
        "C4",
        "C5",
    ]
    for expected_sample, loaded_sample in zip(samples, loaded.samples):
        assert np.allclose(loaded_sample.base_from_tool, expected_sample.base_from_tool)
        assert np.allclose(
            loaded_sample.camera_from_target,
            expected_sample.camera_from_target,
        )
        assert loaded_sample.joint_positions_rad == expected_sample.joint_positions_rad

    try:
        write_calibration_yaml(
            path,
            CAMERA_ON_HAND,
            settings,
            solved,
            _diagnostics(quality, leave_one_out=leave_one_out),
            samples,
            created_at=created_at,
        )
    except FileExistsError:
        pass
    else:
        raise AssertionError("An existing timestamped calibration was overwritten")

    fixed_path = output_path_for_mode(CAMERA_TO_HAND, tmp_path, created_at)
    assert fixed_path.name == "camera_to_hand_calibration_20260902T123456_789012Z.yaml"
    assert reference_frame_for_mode(CAMERA_TO_HAND) == "base_link"
    assert reference_frame_for_mode(CAMERA_ON_HAND) == "Link6"

    old_schema_path = tmp_path / "schema_five.yaml"
    old_payload = yaml.safe_load(content)
    old_payload["schema_version"] = 5
    old_schema_path.write_text(yaml.safe_dump(old_payload), encoding="utf-8")
    try:
        load_calibration_yaml(old_schema_path)
    except ValueError as exc:
        assert "schema_version must be exactly 7" in str(exc)
    else:
        raise AssertionError("A schema-5 calibration artifact was accepted")

    malformed_path = tmp_path / "malformed.yaml"
    malformed_payload = yaml.safe_load(content)
    malformed_payload["captured_samples"][0]["base_from_tool"][3][3] = 0.0
    malformed_path.write_text(yaml.safe_dump(malformed_payload), encoding="utf-8")
    try:
        load_calibration_yaml(malformed_path)
    except ValueError as exc:
        assert "homogeneous bottom row" in str(exc)
    else:
        raise AssertionError("A malformed raw calibration transform was accepted")

    missing_joints_path = tmp_path / "missing_joint_positions.yaml"
    missing_joints_payload = yaml.safe_load(content)
    del missing_joints_payload["captured_samples"][0]["joint_positions_rad"]
    missing_joints_path.write_text(
        yaml.safe_dump(missing_joints_payload),
        encoding="utf-8",
    )
    try:
        load_calibration_yaml(missing_joints_path)
    except ValueError as exc:
        assert "captured_samples[1] must contain exactly" in str(exc)
    else:
        raise AssertionError("A sample without joint positions was accepted")


def test_node_load_calibration_restores_ids_and_recomputes(tmp_path):
    settings = CharucoSettings("camera_two", "DICT_4X4_100", 6, 8, 35.0, 25.0)
    samples, solved = _synthetic_problem(CAMERA_ON_HAND, range(5))
    _solution, quality = solve_camera_on_hand(samples)
    leave_one_out = leave_one_out_diagnostics(
        CAMERA_ON_HAND,
        samples,
        np.eye(4, dtype=np.float64),
        solved,
    )
    path = tmp_path / "load.yaml"
    write_calibration_yaml(
        path,
        CAMERA_ON_HAND,
        settings,
        solved,
        _diagnostics(quality, leave_one_out=leave_one_out),
        samples,
        created_at=datetime(2026, 9, 3, 8, 0, 0, tzinfo=timezone.utc),
    )
    node = calibration_main.CalibrationNode.__new__(calibration_main.CalibrationNode)
    node._lock = __import__("threading").RLock()
    node._event_logger = MagicMock()
    node._calibration_samples = []
    node._next_calibration_sample_number = 1
    node.configure = MagicMock(return_value="configured")
    node.compute = MagicMock(return_value=(True, "recomputed"))
    node.automatic = MagicMock()

    artifact, computed, message = node.load_calibration(path)

    assert computed
    assert artifact.settings == settings
    assert [sample.sample_id for sample in node._calibration_samples] == [
        "C1",
        "C2",
        "C3",
        "C4",
        "C5",
    ]
    assert node._next_calibration_sample_number == 6
    node.configure.assert_called_once_with(settings, CAMERA_ON_HAND)
    node.compute.assert_called_once_with()
    node.automatic.set_recipe.assert_called_once_with(artifact, path)
    assert "Loaded 5 calibration samples" in message


def test_package_log_overwrites_before_event_1001(tmp_path):
    logger = PackageEventLogger(Path(tmp_path))
    for index in range(1001):
        logger.record("INFO", "test", "bounded", index=index)
    lines = logger.path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 1
    assert '"index":1000' in lines[0]


@pytest.mark.parametrize("mode", (CAMERA_ON_HAND, CAMERA_TO_HAND))
def test_tsai_and_adjacent_ax_xb_match_pinned_moveit_conventions(mode):
    samples, expected_optical = _synthetic_problem(mode, range(6))
    robot_inputs = [
        sample.base_from_tool if mode == CAMERA_ON_HAND
        else np.linalg.inv(sample.base_from_tool)
        for sample in samples
    ]
    # Independent reproduction of the pinned upstream OpenCV argument order.
    rotation, translation = cv2.calibrateHandEye(
        [value[:3, :3] for value in robot_inputs],
        [value[:3, 3] for value in robot_inputs],
        [sample.camera_from_target[:3, :3] for sample in samples],
        [sample.camera_from_target[:3, 3] for sample in samples],
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    upstream = np.eye(4)
    upstream[:3, :3], upstream[:3, 3] = rotation, translation.reshape(3)
    assert np.allclose(upstream, expected_optical, atol=1e-6)
    internal = _transform([0.2, -0.1, 0.3], [0.012, 0.024, 0.036])
    result = opencv_worker_runtime.solve(dict(
        calibration_mode=mode, samples=samples, camera_link_from_optical=internal,
        previous_solution=None, previous_quality=None,
    ))
    assert np.allclose(result.reference_from_camera_link, upstream @ np.linalg.inv(internal))
    assert result.diagnostics.ax_xb.translation_rms_mm < 1e-6
    assert result.diagnostics.ax_xb.rotation_rms_deg < 1e-5
    assert result.diagnostics.leave_one_out.status == "valid"

    noisy_x = _transform([0.05, 0.0, 0.0], [0.01, 0.0, 0.0]) @ upstream
    translation_errors, rotation_errors = [], []
    for index in range(len(samples) - 1):
        # The upstream method calls this reprojection error, but it is a
        # transform equation residual, not an image-pixel reprojection error.
        a = np.linalg.inv(robot_inputs[index]) @ robot_inputs[index + 1]
        b = samples[index].camera_from_target @ np.linalg.inv(
            samples[index + 1].camera_from_target
        )
        ax, xb = a @ noisy_x, noisy_x @ b
        translation_errors.append(0.5 * (
            np.linalg.norm(ax[:3, 3] - xb[:3, 3])
            + np.linalg.norm(np.linalg.inv(ax)[:3, 3] - np.linalg.inv(xb)[:3, 3])
        ))
        rotation_errors.append(np.linalg.norm(cv2.Rodrigues(ax[:3, :3].T @ xb[:3, :3])[0]))
    actual = ax_xb_diagnostics(mode, samples, noisy_x)
    assert actual.pair_count == 5
    assert actual.translation_rms_mm == pytest.approx(
        np.sqrt(np.mean(np.square(translation_errors))) * 1000, abs=1e-8
    )
    assert actual.rotation_rms_deg == pytest.approx(
        math.degrees(np.sqrt(np.mean(np.square(rotation_errors)))), abs=1e-8
    )


def test_angular_diversity_checks_every_prior_robot_and_board_pose():
    def pose(deg, x=0.0):
        return _transform([0.0, 0.0, math.radians(deg)], [x, 0.0, 0.0])

    earlier = [_sample("C1", pose(0), pose(0)), _sample("C9", pose(30), pose(30))]
    assert sample_rotation_conflict(pose(5), pose(5), earlier) is None
    assert sample_rotation_conflict(pose(4.999), pose(10), earlier)[:2] == ("C1", "Robot")
    assert sample_rotation_conflict(pose(10), pose(4.999), earlier)[:2] == ("C1", "Board")
    assert sample_rotation_conflict(pose(0, 10), pose(0, 10), earlier)[:2] == ("C1", "Robot")
    assert sample_rotation_conflict(pose(10), pose(32), earlier)[:2] == ("C9", "Board")


@pytest.mark.parametrize(
    "state", ("collinear_corners", "insufficient_corners", "pose_unavailable")
)
def test_ordinary_corner_and_no_pose_frames_block_without_native_failure(state):
    settings = CharucoSettings("camera", "DICT_4X4_50", 5, 7, 30.0, 22.0)
    context = opencv_worker_runtime.create_detector_context(settings)
    board_image = context.board.generateImage((500, 700), marginSize=20)
    corners, ids, markers, marker_ids = context.detector.detectBoard(board_image)
    chosen = np.arange(4 if state != "insufficient_corners" else 3)
    # IDs 0..3 are the first collinear row on a five-square-wide board.
    selected = [int(np.flatnonzero(ids.reshape(-1) == identifier)[0]) for identifier in chosen]
    if state == "pose_unavailable":
        selected = list(range(len(ids)))
    detector = MagicMock(wraps=context.detector)
    detector.detectBoard.return_value = (corners[selected], ids[selected], markers, marker_ids)
    context = opencv_worker_runtime._DetectorContext(settings, context.board, detector)
    image = np.repeat(board_image[:, :, None], 3, axis=2)
    with patch.object(cv2, "solvePnP", return_value=(False, None, None)) as solve:
        result = opencv_worker_runtime.detect_frame(dict(
            frame_sequence=1, image_rgb=image, camera_matrix=np.eye(3),
            distortion=np.zeros(5), solution_overlay=None,
        ), context)
    assert result.state == state and result.camera_from_target is None
    assert solve.call_count == (1 if state == "pose_unavailable" else 0)
    assert detector.detectBoard.call_args.args[0].ndim == 2
    assert _validate_detection_result(
        result, frame_sequence=1, image_shape=image.shape
    ).state == state


def test_distorted_exact_rgb_correspondences_recover_measured_board_pose_and_axes():
    settings = CharucoSettings("camera", "DICT_4X4_50", 3, 3, 28.0, 21.0)
    context = opencv_worker_runtime.create_detector_context(settings)
    expected = _transform([0.1, -0.15, 0.03], [-0.02, 0.01, 0.65])
    k = np.array([[1100.0, 0.0, 960.0], [0.0, 1090.0, 540.0], [0.0, 0.0, 1.0]])
    distortion = np.array([0.10, -0.03, 0.001, -0.002, 0.01])
    rv = cv2.Rodrigues(expected[:3, :3])[0]
    corners = cv2.projectPoints(
        context.board.getChessboardCorners(), rv, expected[:3, 3], k, distortion
    )[0]
    ids = np.arange(4, dtype=np.int32).reshape(-1, 1)
    markers = tuple(cv2.projectPoints(obj, rv, expected[:3, 3], k, distortion)[0].reshape(1, 4, 2)
                    for obj in context.board.getObjPoints())
    detector = MagicMock(wraps=context.detector)
    detector.detectBoard.return_value = (corners, ids, markers, ids)
    context = opencv_worker_runtime._DetectorContext(settings, context.board, detector)
    with patch.object(cv2, "solvePnP", wraps=cv2.solvePnP) as solve:
        result = opencv_worker_runtime.detect_frame(dict(
            frame_sequence=1, image_rgb=np.full((1080, 1920, 3), 255, dtype=np.uint8),
            camera_matrix=k, distortion=distortion, solution_overlay=None,
        ), context)
    assert result.state == "ready"
    assert np.allclose(result.camera_from_target, expected, atol=1e-4)
    assert solve.call_args.kwargs == dict(useExtrinsicGuess=False, flags=cv2.SOLVEPNP_ITERATIVE)
    params = detector.setCharucoParameters.call_args.args[0]
    assert np.array_equal(params.cameraMatrix, k)
    assert np.array_equal(params.distCoeffs.reshape(-1), distortion)


@pytest.mark.parametrize("count", (5, 6))
@pytest.mark.parametrize("mode", (CAMERA_ON_HAND, CAMERA_TO_HAND))
def test_schema_seven_strict_round_trip_and_rejections(tmp_path, count, mode):
    samples, _expected = _synthetic_problem(mode, range(count))
    # Preserve nonconsecutive, non-sorted IDs as well as capture order.
    samples = [_sample(f"C{20 - index}", sample.base_from_tool, sample.camera_from_target,
                       sample.joint_positions_rad) for index, sample in enumerate(samples)]
    result = opencv_worker_runtime.solve(dict(
        calibration_mode=mode, samples=samples, camera_link_from_optical=np.eye(4),
        previous_solution=None, previous_quality=None,
    ))
    path = tmp_path / "new.yaml"
    settings = CharucoSettings("camera", "DICT_4X4_50", 3, 3, 28.0, 21.0)
    write_calibration_yaml(path, mode, settings, result.reference_from_camera_link,
                           result.diagnostics, samples)
    loaded = load_calibration_yaml(path)
    assert [sample.sample_id for sample in loaded.samples] == [
        sample.sample_id for sample in samples
    ]
    payload = yaml.safe_load(path.read_text())
    assert payload["quality"]["ax_xb"]["pair_count"] == count - 1
    assert payload["quality"]["leave_one_out"]["status"] == (
        "not_available" if count == 5 else "valid"
    )
    bad_path = tmp_path / "bad.yaml"
    for version in range(1, 7):
        bad_path.write_text(yaml.safe_dump(dict(payload, schema_version=version)))
        with pytest.raises(ValueError, match="schema_version must be exactly 7"):
            load_calibration_yaml(bad_path)
    for field in ("base_from_tool", "camera_from_target"):
        broken = yaml.safe_load(path.read_text())
        broken["captured_samples"][-1][field] = broken["captured_samples"][0][field]
        bad_path.write_text(yaml.safe_dump(broken))
        with pytest.raises(ValueError, match="orientation is too similar to C20"):
            load_calibration_yaml(bad_path)
    broken = yaml.safe_load(path.read_text())
    broken["captured_samples"] = broken["captured_samples"][:4]
    bad_path.write_text(yaml.safe_dump(broken))
    with pytest.raises(ValueError, match="At least 5 calibration samples"):
        load_calibration_yaml(bad_path)
    broken = yaml.safe_load(path.read_text())
    broken["pipeline"]["reference_commit"] = "not-the-pinned-commit"
    bad_path.write_text(yaml.safe_dump(broken))
    with pytest.raises(ValueError, match="pipeline.reference_commit"):
        load_calibration_yaml(bad_path)


def test_prefill_schema_four_contains_no_samples_or_corner_selector(tmp_path):
    path = tmp_path / "last_session.json"
    settings = CharucoSettings("camera", "DICT_4X4_50", 3, 3, 28.0, 21.0)
    write_calibration_ui_state(path, CAMERA_TO_HAND, settings)
    payload = json.loads(path.read_text())
    assert payload["schema_version"] == 4 and payload["minimum_samples"] == 5
    assert "minimum_corners" not in payload
    assert "samples" not in payload and "solution" not in payload
    for version in range(1, 4):
        path.write_text(json.dumps(dict(payload, schema_version=version)))
        with pytest.raises(ValueError, match="schema_version must be exactly 4"):
            load_calibration_ui_state(path)
