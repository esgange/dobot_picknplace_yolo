import multiprocessing
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_prefix

from .calibration_core import (
    AccuracyDiagnostics,
    CalibrationQuality,
    CalibrationSample,
    CharucoSettings,
    REQUIRED_CHARUCO_LEGACY_PATTERN,
    REQUIRED_HAND_EYE_METHOD,
    REQUIRED_OPENCV_API,
    REQUIRED_OPENCV_THREAD_COUNT,
    REQUIRED_OPENCV_VERSION,
    REQUIRED_OPENCV_WHEEL_SHA256,
    rotation_matrix_to_rpy_deg,
)


OPENCV_WORKER_STARTUP_TIMEOUT_SEC = 5.0
OPENCV_WORKER_RESPONSE_TIMEOUT_SEC = 5.0
OPENCV_WORKER_SHUTDOWN_TIMEOUT_SEC = 2.0
SOLUTION_OVERLAY_MIN_FONT_SCALE = 0.8
SOLUTION_OVERLAY_MAX_FONT_SCALE = 1.2
SOLUTION_OVERLAY_FONT_WIDTH_REFERENCE_PX = 850.0
SOLUTION_OVERLAY_TEXT_THICKNESS = 2
OPENCV_RUNTIME_RELATIVE_PATH = Path("lib/camera_calibration/opencv_runtime")
ARUCO_5X5_DICTIONARIES = (
    "DICT_5X5_50",
    "DICT_5X5_100",
    "DICT_5X5_250",
    "DICT_5X5_1000",
)
REQUIRED_ARUCO_API = "ArucoDetector.detectMarkers+solvePnP(SOLVEPNP_IPPE_SQUARE)"
REQUIRED_BIN_MARKER_IDS = (0, 1, 2, 3)


@dataclass(frozen=True)
class DetectionResult:
    frame_sequence: int
    state: str
    message: str
    corner_count: int
    camera_from_target: np.ndarray | None
    overlay_rgb: np.ndarray


@dataclass(frozen=True)
class ArucoMarkerSettings:
    dictionary_name: str
    marker_size_mm: float

    def validate(self) -> None:
        if self.dictionary_name not in ARUCO_5X5_DICTIONARIES:
            raise ValueError("ArUco dictionary must be an exact 5x5 dictionary")
        if (
            type(self.marker_size_mm) not in {int, float}
            or not np.isfinite(float(self.marker_size_mm))
            or float(self.marker_size_mm) <= 0.0
        ):
            raise ValueError("ArUco marker size must be greater than zero millimetres")


@dataclass(frozen=True)
class ArucoMarkerObservation:
    marker_id: int
    image_corners_px: np.ndarray
    optical_corners_m: np.ndarray
    optical_from_marker: np.ndarray


@dataclass(frozen=True)
class ArucoDetectionResult:
    frame_sequence: int
    state: str
    message: str
    detected_ids: tuple[int, ...]
    observations: tuple[ArucoMarkerObservation, ...]
    overlay_rgb: np.ndarray


@dataclass(frozen=True)
class SolveResult:
    reference_from_camera_link: np.ndarray
    diagnostics: AccuracyDiagnostics


def _validate_detection_result(
    result,
    *,
    frame_sequence: int,
    image_shape: tuple[int, ...],
) -> DetectionResult:
    if not isinstance(result, DetectionResult):
        raise ValueError("response is not a DetectionResult")
    if type(result.frame_sequence) is not int or result.frame_sequence != frame_sequence:
        raise ValueError("frame sequence does not match the request")
    if result.state not in {
        "not_visible", "insufficient_corners", "collinear_corners",
        "pose_unavailable", "ready",
    }:
        raise ValueError(f"unsupported detection state {result.state!r}")
    if type(result.message) is not str or not result.message:
        raise ValueError("detection message is empty or not a string")
    if type(result.corner_count) is not int or result.corner_count < 0:
        raise ValueError("corner count is not a non-negative integer")
    overlay = result.overlay_rgb
    if (
        not isinstance(overlay, np.ndarray)
        or overlay.dtype != np.uint8
        or overlay.shape != image_shape
        or not overlay.flags.c_contiguous
    ):
        raise ValueError("RGB overlay is not a contiguous uint8 image of requested shape")
    pose = result.camera_from_target
    if result.state == "ready":
        if (
            result.corner_count < 4
            or not isinstance(pose, np.ndarray)
            or pose.shape != (4, 4)
            or not np.all(np.isfinite(pose))
            or not np.allclose(pose[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12)
            or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-3)
            or abs(float(np.linalg.det(pose[:3, :3])) - 1.0) > 1e-3
        ):
            raise ValueError("ready result does not contain four corners and a rigid RGB pose")
    elif pose is not None:
        raise ValueError("non-ready result unexpectedly contains a pose")
    return DetectionResult(
        frame_sequence=result.frame_sequence,
        state=result.state,
        message=result.message,
        corner_count=result.corner_count,
        camera_from_target=(
            None if pose is None else np.array(pose, dtype=np.float64, order="C", copy=True)
        ),
        overlay_rgb=np.array(overlay, dtype=np.uint8, order="C", copy=True),
    )


def _validate_aruco_detection_result(
    result,
    *,
    frame_sequence: int,
    image_shape: tuple[int, ...],
) -> ArucoDetectionResult:
    if not isinstance(result, ArucoDetectionResult):
        raise ValueError("response is not an ArucoDetectionResult")
    if type(result.frame_sequence) is not int or result.frame_sequence != frame_sequence:
        raise ValueError("frame sequence does not match the request")
    if result.state not in {
        "not_visible",
        "required_ids_missing",
        "unexpected_ids",
        "pose_unavailable",
        "ready",
    }:
        raise ValueError(f"unsupported ArUco detection state {result.state!r}")
    if type(result.message) is not str or not result.message:
        raise ValueError("ArUco detection message is empty or not a string")
    if (
        not isinstance(result.detected_ids, tuple)
        or any(type(marker_id) is not int for marker_id in result.detected_ids)
        or tuple(sorted(set(result.detected_ids))) != result.detected_ids
    ):
        raise ValueError("detected ArUco IDs must be a sorted unique integer tuple")
    overlay = result.overlay_rgb
    if (
        not isinstance(overlay, np.ndarray)
        or overlay.dtype != np.uint8
        or overlay.shape != image_shape
        or not overlay.flags.c_contiguous
    ):
        raise ValueError("ArUco RGB overlay is not a contiguous requested-shape image")
    if result.state == "ready":
        if result.detected_ids != REQUIRED_BIN_MARKER_IDS:
            raise ValueError("ready ArUco result does not contain exact IDs 0 through 3")
        if (
            not isinstance(result.observations, tuple)
            or len(result.observations) != len(REQUIRED_BIN_MARKER_IDS)
        ):
            raise ValueError("ready ArUco result does not contain four observations")
        observation_ids = tuple(item.marker_id for item in result.observations)
        if observation_ids != REQUIRED_BIN_MARKER_IDS:
            raise ValueError("ArUco observations are not in exact marker-ID order")
        for observation in result.observations:
            if (
                not isinstance(observation, ArucoMarkerObservation)
                or not isinstance(observation.image_corners_px, np.ndarray)
                or observation.image_corners_px.shape != (4, 2)
                or not np.all(np.isfinite(observation.image_corners_px))
                or not isinstance(observation.optical_corners_m, np.ndarray)
                or observation.optical_corners_m.shape != (4, 3)
                or not np.all(np.isfinite(observation.optical_corners_m))
                or not isinstance(observation.optical_from_marker, np.ndarray)
                or observation.optical_from_marker.shape != (4, 4)
                or not np.all(np.isfinite(observation.optical_from_marker))
                or not np.allclose(
                    observation.optical_from_marker[3],
                    [0.0, 0.0, 0.0, 1.0],
                    atol=1e-12,
                )
                or not np.allclose(
                    observation.optical_from_marker[:3, :3].T
                    @ observation.optical_from_marker[:3, :3],
                    np.eye(3),
                    atol=1e-3,
                )
                or abs(
                    float(np.linalg.det(observation.optical_from_marker[:3, :3]))
                    - 1.0
                ) > 1e-3
            ):
                raise ValueError("ready ArUco result contains malformed marker geometry")
    elif result.observations:
        raise ValueError("blocked ArUco result unexpectedly contains observations")
    observations = tuple(
        ArucoMarkerObservation(
            marker_id=item.marker_id,
            image_corners_px=np.array(
                item.image_corners_px, dtype=np.float64, order="C", copy=True
            ),
            optical_corners_m=np.array(
                item.optical_corners_m, dtype=np.float64, order="C", copy=True
            ),
            optical_from_marker=np.array(
                item.optical_from_marker, dtype=np.float64, order="C", copy=True
            ),
        )
        for item in result.observations
    )
    return ArucoDetectionResult(
        frame_sequence=result.frame_sequence,
        state=result.state,
        message=result.message,
        detected_ids=result.detected_ids,
        observations=observations,
        overlay_rgb=np.array(overlay, dtype=np.uint8, order="C", copy=True),
    )


class OpenCvWorkerFailure(RuntimeError):
    def __init__(
        self,
        message: str,
        *,
        operation: str,
        worker_pid: int | None,
        exit_code: int | None,
    ) -> None:
        super().__init__(message)
        self.operation = operation
        self.worker_pid = worker_pid
        self.exit_code = exit_code

    @property
    def signal_name(self) -> str | None:
        if self.exit_code is None or self.exit_code >= 0:
            return None
        try:
            return signal.Signals(-self.exit_code).name
        except ValueError:
            return f"SIGNAL_{-self.exit_code}"


class OpenCvWorkerOperationError(RuntimeError):
    def __init__(self, operation: str, error_type: str, message: str) -> None:
        super().__init__(f"OpenCV worker {operation} failed ({error_type}): {message}")
        self.operation = operation
        self.error_type = error_type


def _solution_overlay_lines(
    reference_frame: str,
    camera_link_frame: str,
    reference_from_camera_link: np.ndarray,
    quality: CalibrationQuality,
) -> tuple[str, ...]:
    translation = reference_from_camera_link[:3, 3]
    roll, pitch, yaw = rotation_matrix_to_rpy_deg(reference_from_camera_link[:3, :3])
    return (
        f"CALIBRATED TF | {quality.sample_count} samples",
        f"{reference_frame} <- {camera_link_frame}",
        f"XYZ [m]: {translation[0]:+.4f}, {translation[1]:+.4f}, {translation[2]:+.4f}",
        f"RPY [deg]: {roll:+.2f}, {pitch:+.2f}, {yaw:+.2f}",
        f"FIT RMS: {quality.translation_rms_mm:.3f} mm / {quality.rotation_rms_deg:.3f} deg",
    )


def _solution_overlay_font_scale(image_width: int) -> float:
    return max(
        SOLUTION_OVERLAY_MIN_FONT_SCALE,
        min(
            SOLUTION_OVERLAY_MAX_FONT_SCALE,
            image_width / SOLUTION_OVERLAY_FONT_WIDTH_REFERENCE_PX,
        ),
    )


def installed_opencv_runtime_dir() -> Path:
    prefix = Path(get_package_prefix("camera_calibration")).resolve()
    return prefix / OPENCV_RUNTIME_RELATIVE_PATH


def _validate_runtime_directory(runtime_dir: Path) -> Path:
    candidate = Path(runtime_dir).resolve()
    required_files = (
        candidate / "cv2" / "__init__.py",
        candidate / "cv2" / "cv2.abi3.so",
        candidate / "opencv_python-4.10.0.84.dist-info" / "LICENSE.txt",
    )
    missing = [str(path) for path in required_files if not path.is_file()]
    if missing:
        raise OpenCvWorkerFailure(
            "Package-private OpenCV runtime is incomplete; "
            f"missing={missing}",
            operation="startup",
            worker_pid=None,
            exit_code=None,
        )
    return candidate


def _worker_entry(connection, runtime_dir_text: str) -> None:
    runtime_dir = Path(runtime_dir_text).resolve()
    sys.path.insert(0, str(runtime_dir))
    from .opencv_worker_runtime import worker_main

    worker_main(connection, runtime_dir)


class OpenCvWorkerClient:
    def __init__(self, runtime_dir: Path | None = None) -> None:
        selected_runtime = _validate_runtime_directory(
            installed_opencv_runtime_dir() if runtime_dir is None else runtime_dir
        )
        context = multiprocessing.get_context("spawn")
        parent_connection, child_connection = context.Pipe(duplex=True)
        self._connection = parent_connection
        self._lock = threading.Lock()
        self._request_number = 0
        self._closed = False
        self._failed = False
        self._process = context.Process(
            target=_worker_entry,
            args=(child_connection, str(selected_runtime)),
            name="camera_calibration_opencv_worker",
            daemon=True,
        )
        self._process.start()
        child_connection.close()
        try:
            response = self._receive_response(
                "startup",
                None,
                OPENCV_WORKER_STARTUP_TIMEOUT_SEC,
            )
        except OpenCvWorkerFailure:
            self._failed = True
            self.close()
            raise
        if response.get("operation") != "startup":
            self._failed = True
            self.close()
            raise OpenCvWorkerFailure(
                "OpenCV worker returned an invalid startup response",
                operation="startup",
                worker_pid=self.pid,
                exit_code=self.exit_code,
            )
        if not response.get("ok"):
            self._failed = True
            error_type = str(response.get("error_type", "unknown"))
            error = str(response.get("error", "no error detail"))
            self.close()
            raise OpenCvWorkerFailure(
                f"OpenCV worker startup failed ({error_type}): {error}",
                operation="startup",
                worker_pid=self.pid,
                exit_code=self.exit_code,
            )
        self.runtime = response["payload"]
        required_runtime = {
            "opencv_version": REQUIRED_OPENCV_VERSION,
            "opencv_thread_count": REQUIRED_OPENCV_THREAD_COUNT,
            "opencv_opencl_enabled": False,
            "opencv_api": REQUIRED_OPENCV_API,
            "charuco_legacy_pattern": REQUIRED_CHARUCO_LEGACY_PATTERN,
            "hand_eye_method": REQUIRED_HAND_EYE_METHOD,
            "opencv_wheel_sha256": REQUIRED_OPENCV_WHEEL_SHA256,
        }
        for key, expected in required_runtime.items():
            if self.runtime.get(key) != expected:
                failure = self._failure(
                    "startup",
                    f"runtime field {key} must be {expected!r}; "
                    f"received {self.runtime.get(key)!r}",
                )
                self.close()
                raise failure

    @property
    def pid(self) -> int | None:
        return self._process.pid

    @property
    def exit_code(self) -> int | None:
        return self._process.exitcode

    def _failure(self, operation: str, reason: str) -> OpenCvWorkerFailure:
        self._process.join(timeout=0.05)
        exit_code = self._process.exitcode
        signal_text = ""
        if exit_code is not None and exit_code < 0:
            try:
                signal_text = f" ({signal.Signals(-exit_code).name})"
            except ValueError:
                signal_text = f" (signal {-exit_code})"
        self._failed = True
        return OpenCvWorkerFailure(
            (
                f"OpenCV worker failed during {operation}: {reason}; "
                f"pid={self.pid}, exit_code={exit_code}{signal_text}; no retry was attempted"
            ),
            operation=operation,
            worker_pid=self.pid,
            exit_code=exit_code,
        )

    def _receive_response(
        self,
        operation: str,
        request_id: int | None,
        timeout_sec: float,
    ) -> dict:
        deadline = time.monotonic() + timeout_sec
        while True:
            wait_time = min(0.05, max(0.0, deadline - time.monotonic()))
            if self._connection.poll(wait_time):
                try:
                    response = self._connection.recv()
                except (EOFError, OSError) as exc:
                    raise self._failure(operation, f"response pipe closed: {exc}") from exc
                if not isinstance(response, dict):
                    raise self._failure(operation, "response was not a dictionary")
                if request_id is not None and response.get("request_id") != request_id:
                    raise self._failure(operation, "response request ID did not match")
                return response
            if not self._process.is_alive():
                raise self._failure(operation, "process exited before replying")
            if time.monotonic() >= deadline:
                self._process.terminate()
                self._process.join(timeout=OPENCV_WORKER_SHUTDOWN_TIMEOUT_SEC)
                raise self._failure(
                    operation,
                    f"no response within {timeout_sec:.1f} seconds; worker was terminated",
                )

    def _request(self, operation: str, payload):
        with self._lock:
            if self._closed:
                raise OpenCvWorkerFailure(
                    "OpenCV worker is closed",
                    operation=operation,
                    worker_pid=self.pid,
                    exit_code=self.exit_code,
                )
            if self._failed or not self._process.is_alive():
                raise self._failure(operation, "process is not alive")
            self._request_number += 1
            request_id = self._request_number
            try:
                self._connection.send(
                    {
                        "request_id": request_id,
                        "operation": operation,
                        "payload": payload,
                    }
                )
            except (BrokenPipeError, EOFError, OSError) as exc:
                raise self._failure(operation, f"request pipe failed: {exc}") from exc
            response = self._receive_response(
                operation,
                request_id,
                OPENCV_WORKER_RESPONSE_TIMEOUT_SEC,
            )
            if response.get("operation") != operation:
                raise self._failure(operation, "response operation did not match")
            if not response.get("ok"):
                raise OpenCvWorkerOperationError(
                    operation,
                    str(response.get("error_type", "unknown")),
                    str(response.get("error", "no error detail")),
                )
            return response["payload"]

    def check_health(self) -> None:
        if self._closed:
            return
        if self._failed or not self._process.is_alive():
            raise self._failure("health_check", "process is not alive")

    def configure(self, settings: CharucoSettings) -> None:
        self._request("configure", settings)

    def configure_aruco(self, settings: ArucoMarkerSettings) -> None:
        settings.validate()
        self._request("configure_aruco", settings)

    def detect(
        self,
        *,
        frame_sequence: int,
        image_rgb: np.ndarray,
        camera_matrix: np.ndarray,
        distortion: np.ndarray,
        solution_overlay: dict | None,
    ) -> DetectionResult:
        result = self._request(
            "detect",
            {
                "frame_sequence": frame_sequence,
                "image_rgb": image_rgb,
                "camera_matrix": camera_matrix,
                "distortion": distortion,
                "solution_overlay": solution_overlay,
            },
        )
        try:
            return _validate_detection_result(
                result,
                frame_sequence=frame_sequence,
                image_shape=image_rgb.shape,
            )
        except ValueError as exc:
            raise self._failure("detect", f"malformed detector result: {exc}") from exc

    def detect_aruco(
        self,
        *,
        frame_sequence: int,
        image_rgb: np.ndarray,
        camera_matrix: np.ndarray,
        distortion: np.ndarray,
    ) -> ArucoDetectionResult:
        result = self._request(
            "detect_aruco",
            {
                "frame_sequence": frame_sequence,
                "image_rgb": image_rgb,
                "camera_matrix": camera_matrix,
                "distortion": distortion,
            },
        )
        try:
            return _validate_aruco_detection_result(
                result,
                frame_sequence=frame_sequence,
                image_shape=image_rgb.shape,
            )
        except ValueError as exc:
            raise self._failure(
                "detect_aruco",
                f"malformed detector result: {exc}",
            ) from exc

    def image_rays(
        self,
        *,
        image_points: np.ndarray,
        camera_matrix: np.ndarray,
        distortion: np.ndarray,
    ) -> np.ndarray:
        """Undistorted optical rays (x, y, 1); cv2 remains in the lifetime child."""
        result = self._request(
            "image_rays",
            {"image_points": image_points, "camera_matrix": camera_matrix,
             "distortion": distortion},
        )
        if (
            not isinstance(result, np.ndarray)
            or result.dtype != np.float64
            or result.shape != (len(image_points), 3)
            or not np.all(np.isfinite(result))
            or not np.all(result[:, 2] == 1.0)
        ):
            raise self._failure("image_rays", "malformed undistorted ray result")
        return result

    def project_points(
        self,
        *,
        optical_points: np.ndarray,
        camera_matrix: np.ndarray,
        distortion: np.ndarray,
    ) -> np.ndarray:
        result = self._request(
            "project_points",
            {
                "optical_points": optical_points,
                "camera_matrix": camera_matrix,
                "distortion": distortion,
            },
        )
        if (
            not isinstance(result, np.ndarray)
            or result.dtype != np.float64
            or result.shape != (len(optical_points), 2)
            or not np.all(np.isfinite(result))
        ):
            raise self._failure("project_points", "malformed projected point result")
        return result

    def solve(
        self,
        *,
        calibration_mode: str,
        samples: list[CalibrationSample],
        camera_link_from_optical: np.ndarray,
        previous_solution: np.ndarray | None,
        previous_quality: CalibrationQuality | None,
    ) -> SolveResult:
        return self._request(
            "solve",
            {
                "calibration_mode": calibration_mode,
                "samples": samples,
                "camera_link_from_optical": camera_link_from_optical,
                "previous_solution": previous_solution,
                "previous_quality": previous_quality,
            },
        )

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            if self._process.is_alive() and not self._failed:
                self._request_number += 1
                request_id = self._request_number
                try:
                    self._connection.send(
                        {
                            "request_id": request_id,
                            "operation": "shutdown",
                            "payload": None,
                        }
                    )
                    self._receive_response(
                        "shutdown",
                        request_id,
                        OPENCV_WORKER_SHUTDOWN_TIMEOUT_SEC,
                    )
                except (OpenCvWorkerFailure, BrokenPipeError, EOFError, OSError):
                    pass
            self._process.join(timeout=OPENCV_WORKER_SHUTDOWN_TIMEOUT_SEC)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=OPENCV_WORKER_SHUTDOWN_TIMEOUT_SEC)
            self._connection.close()
