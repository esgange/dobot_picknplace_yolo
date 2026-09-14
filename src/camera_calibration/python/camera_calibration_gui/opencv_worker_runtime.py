import signal
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .calibration_core import (
    AccuracyDiagnostics,
    CalibrationSample,
    CharucoSettings,
    InvalidPoseError,
    MINIMUM_CHARUCO_CORNERS,
    MOVEIT_REFERENCE_COMMIT,
    ax_xb_diagnostics,
    REQUIRED_CHARUCO_LEGACY_PATTERN,
    REQUIRED_HAND_EYE_METHOD,
    REQUIRED_OPENCV_API,
    REQUIRED_OPENCV_THREAD_COUNT,
    REQUIRED_OPENCV_VERSION,
    REQUIRED_OPENCV_WHEEL_SHA256,
    invert_transform,
    leave_one_out_diagnostics,
    pose_coverage_diagnostics,
    solution_change_diagnostics,
    solve_calibration,
)
from .opencv_worker import (
    ArucoDetectionResult,
    ArucoMarkerObservation,
    ArucoMarkerSettings,
    DetectionResult,
    REQUIRED_ARUCO_API,
    REQUIRED_BIN_MARKER_IDS,
    SOLUTION_OVERLAY_TEXT_THICKNESS,
    SolveResult,
    _solution_overlay_font_scale,
    _solution_overlay_lines,
)


@dataclass
class _DetectorContext:
    settings: CharucoSettings
    board: object
    detector: object


@dataclass
class _ArucoDetectorContext:
    settings: ArucoMarkerSettings
    detector: object
    object_corners_m: np.ndarray


class OpenCvRuntimeContractError(RuntimeError):
    """Signal terminal drift from the package-private OpenCV contract."""


def _is_below(path: Path, directory: Path) -> bool:
    try:
        path.relative_to(directory)
    except ValueError:
        return False
    return True


def _verify_runtime(runtime_dir: Path) -> dict:
    module_path = Path(cv2.__file__).resolve()
    if not _is_below(module_path, runtime_dir):
        raise OpenCvRuntimeContractError(
            "OpenCV module escaped the package-private runtime: "
            f"module={module_path}, runtime={runtime_dir}"
        )
    if cv2.__version__ != REQUIRED_OPENCV_VERSION:
        raise OpenCvRuntimeContractError(
            "camera_calibration requires package-private OpenCV version exactly "
            f"{REQUIRED_OPENCV_VERSION}; loaded {cv2.__version__}"
        )
    required_api = {
        "getPredefinedDictionary": cv2.aruco,
        "DetectorParameters": cv2.aruco,
        "CharucoBoard": cv2.aruco,
        "setLegacyPattern": cv2.aruco.CharucoBoard,
        "CharucoDetector": cv2.aruco,
        "detectBoard": cv2.aruco.CharucoDetector,
        "matchImagePoints": cv2.aruco.CharucoBoard,
        "solvePnP": cv2,
        "projectPoints": cv2,
        "undistortPoints": cv2,
        "Rodrigues": cv2,
        "drawFrameAxes": cv2,
        "checkCharucoCornersCollinear": cv2.aruco.CharucoBoard,
        "ArucoDetector": cv2.aruco,
        "detectMarkers": cv2.aruco.ArucoDetector,
        "drawDetectedMarkers": cv2.aruco,
    }
    missing = [
        name
        for name, owner in required_api.items()
        if not callable(getattr(owner, name, None))
    ]
    if missing:
        raise OpenCvRuntimeContractError(
            f"Required OpenCV 4.10 ChArUco API is missing: {missing}"
        )
    if cv2.getNumThreads() != REQUIRED_OPENCV_THREAD_COUNT:
        raise OpenCvRuntimeContractError(
            "OpenCV thread count changed after initialization; "
            f"required={REQUIRED_OPENCV_THREAD_COUNT} actual={cv2.getNumThreads()}"
        )
    if cv2.ocl.useOpenCL():
        raise OpenCvRuntimeContractError(
            "OpenCV OpenCL was enabled after initialization"
        )
    return {
        "opencv_version": cv2.__version__,
        "opencv_module_path": str(module_path),
        "opencv_thread_count": cv2.getNumThreads(),
        "opencv_opencl_enabled": False,
        "opencv_api": REQUIRED_OPENCV_API,
        "charuco_legacy_pattern": REQUIRED_CHARUCO_LEGACY_PATTERN,
        "hand_eye_method": REQUIRED_HAND_EYE_METHOD,
        "moveit_reference_commit": MOVEIT_REFERENCE_COMMIT,
        "marker_corner_refinement": "CORNER_REFINE_NONE",
        "adjacent_marker_minimum": 2,
        "marker_recovery": False,
        "opencv_wheel_sha256": REQUIRED_OPENCV_WHEEL_SHA256,
        "bin_teach_aruco_api": REQUIRED_ARUCO_API,
    }


def configure_opencv_runtime(runtime_dir: Path) -> dict:
    if cv2.__version__ != REQUIRED_OPENCV_VERSION:
        raise OpenCvRuntimeContractError(
            "camera_calibration requires package-private OpenCV version exactly "
            f"{REQUIRED_OPENCV_VERSION}; loaded {cv2.__version__}"
        )
    cv2.setNumThreads(REQUIRED_OPENCV_THREAD_COUNT)
    cv2.ocl.setUseOpenCL(False)
    return _verify_runtime(runtime_dir)


def create_legacy_charuco_board(settings: CharucoSettings):
    settings.validate()
    dictionary_identifier = getattr(cv2.aruco, settings.dictionary_name, None)
    if type(dictionary_identifier) is not int:
        raise OpenCvRuntimeContractError(
            f"OpenCV lacks required dictionary constant {settings.dictionary_name}"
        )
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_identifier)
    board = cv2.aruco.CharucoBoard(
        (settings.squares_x, settings.squares_y),
        settings.square_length_mm / 1000.0,
        settings.marker_length_mm / 1000.0,
        dictionary,
    )
    board.setLegacyPattern(REQUIRED_CHARUCO_LEGACY_PATTERN)
    if board.getLegacyPattern() is not REQUIRED_CHARUCO_LEGACY_PATTERN:
        raise OpenCvRuntimeContractError(
            "ChArUco board did not retain the required legacy pattern"
        )
    return dictionary, board


def create_detector_context(settings: CharucoSettings) -> _DetectorContext:
    dictionary, board = create_legacy_charuco_board(settings)
    charuco_parameters = cv2.aruco.CharucoParameters()
    charuco_parameters.minMarkers = 2
    charuco_parameters.tryRefineMarkers = False
    detector_parameters = cv2.aruco.DetectorParameters()
    detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    detector = cv2.aruco.CharucoDetector(
        board,
        charuco_parameters,
        detector_parameters,
    )
    if detector.getBoard().getDictionary().bytesList.shape != dictionary.bytesList.shape:
        raise OpenCvRuntimeContractError(
            "ChArUco detector did not retain the configured dictionary"
        )
    return _DetectorContext(settings, board, detector)


def create_aruco_detector_context(
    settings: ArucoMarkerSettings,
) -> _ArucoDetectorContext:
    settings.validate()
    dictionary_identifier = getattr(cv2.aruco, settings.dictionary_name, None)
    if type(dictionary_identifier) is not int:
        raise OpenCvRuntimeContractError(
            f"OpenCV lacks required dictionary constant {settings.dictionary_name}"
        )
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_identifier)
    detector_parameters = cv2.aruco.DetectorParameters()
    detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    detector = cv2.aruco.ArucoDetector(dictionary, detector_parameters)
    half_size_m = (float(settings.marker_size_mm) / 1000.0) / 2.0
    object_corners_m = np.asarray(
        [
            [-half_size_m, half_size_m, 0.0],
            [half_size_m, half_size_m, 0.0],
            [half_size_m, -half_size_m, 0.0],
            [-half_size_m, -half_size_m, 0.0],
        ],
        dtype=np.float32,
    )
    return _ArucoDetectorContext(settings, detector, object_corners_m)


def _validate_camera_parameters(payload: dict) -> tuple[np.ndarray, np.ndarray]:
    camera_matrix = np.asarray(payload["camera_matrix"], dtype=np.float64)
    distortion = np.asarray(payload["distortion"], dtype=np.float64).reshape(-1)
    if camera_matrix.shape != (3, 3) or not np.all(np.isfinite(camera_matrix)):
        raise ValueError("Detector camera matrix must be a finite 3x3 matrix")
    if distortion.size < 4 or not np.all(np.isfinite(distortion)):
        raise ValueError("Detector distortion must have at least four finite values")
    return (
        np.array(camera_matrix, dtype=np.float64, order="C", copy=True),
        np.array(distortion, dtype=np.float64, order="C", copy=True),
    )


def _validate_detected_points(
    corners,
    identifiers,
    *,
    points_per_item: int,
    label: str,
) -> int:
    if identifiers is None:
        if corners is not None and len(corners) != 0:
            raise ValueError(f"OpenCV returned {label} corners without IDs")
        return 0
    ids = np.asarray(identifiers)
    if ids.dtype.kind not in {"i", "u"} or ids.ndim not in {1, 2}:
        raise ValueError(f"OpenCV returned malformed {label} IDs")
    count = int(ids.size)
    if corners is None or len(corners) != count:
        raise ValueError(f"OpenCV returned mismatched {label} corners and IDs")
    for index, item in enumerate(corners):
        points = np.asarray(item)
        if points.size != points_per_item * 2 or not np.all(np.isfinite(points)):
            raise ValueError(
                f"OpenCV returned malformed {label} corners at index {index}"
            )
    return count


def _draw_solution_overlay(
    image: np.ndarray,
    reference_frame: str,
    camera_link_frame: str,
    reference_from_camera_link: np.ndarray,
    quality,
) -> None:
    lines = _solution_overlay_lines(
        reference_frame,
        camera_link_frame,
        reference_from_camera_link,
        quality,
    )
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = _solution_overlay_font_scale(image.shape[1])
    thickness = SOLUTION_OVERLAY_TEXT_THICKNESS
    padding = 8
    line_height = max(
        cv2.getTextSize(line, font, scale, thickness)[0][1] for line in lines
    ) + 7
    text_width = max(
        cv2.getTextSize(line, font, scale, thickness)[0][0] for line in lines
    )
    panel_width = min(image.shape[1], text_width + (padding * 2))
    panel_height = min(image.shape[0], (line_height * len(lines)) + (padding * 2))
    shade = image.copy()
    cv2.rectangle(shade, (0, 0), (panel_width, panel_height), (0, 0, 0), -1)
    cv2.addWeighted(shade, 0.72, image, 0.28, 0.0, image)
    for index, line in enumerate(lines):
        color = (80, 255, 80) if index == 0 else (255, 255, 255)
        y_position = padding + ((index + 1) * line_height) - 4
        cv2.putText(
            image,
            line,
            (padding, y_position),
            font,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )


def _draw_measurement_overlay(image: np.ndarray, lines: tuple[str, ...]) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = max(0.52, min(0.78, image.shape[1] / 1600.0))
    thickness = 2
    padding = 7
    line_height = max(
        cv2.getTextSize(line, font, scale, thickness)[0][1] for line in lines
    ) + 6
    text_width = max(
        cv2.getTextSize(line, font, scale, thickness)[0][0] for line in lines
    )
    panel_width = min(image.shape[1], text_width + (padding * 2))
    panel_height = min(image.shape[0], (line_height * len(lines)) + (padding * 2))
    top = max(0, image.shape[0] - panel_height)
    shade = image.copy()
    cv2.rectangle(
        shade,
        (0, top),
        (panel_width, image.shape[0]),
        (0, 0, 0),
        -1,
    )
    cv2.addWeighted(shade, 0.72, image, 0.28, 0.0, image)
    for index, line in enumerate(lines):
        cv2.putText(
            image,
            line,
            (padding, top + padding + ((index + 1) * line_height) - 4),
            font,
            scale,
            (80, 255, 80) if index == 0 else (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )


def _finish_detection(
    *,
    frame_sequence: int,
    state: str,
    message: str,
    corner_count: int,
    camera_from_target: np.ndarray | None,
    overlay_bgr: np.ndarray,
    solution_overlay: dict | None,
) -> DetectionResult:
    if solution_overlay is not None:
        _draw_solution_overlay(
            overlay_bgr,
            solution_overlay["reference_frame"],
            solution_overlay["camera_link_frame"],
            solution_overlay["reference_from_camera_link"],
            solution_overlay["quality"],
        )
    overlay_rgb = cv2.cvtColor(overlay_bgr, cv2.COLOR_BGR2RGB)
    return DetectionResult(
        frame_sequence,
        state,
        message,
        corner_count,
        camera_from_target,
        np.array(overlay_rgb, dtype=np.uint8, order="C", copy=True),
    )


def detect_frame(payload: dict, context: _DetectorContext) -> DetectionResult:
    frame_sequence = payload["frame_sequence"]
    candidate = np.asarray(payload["image_rgb"])
    if (
        candidate.dtype != np.uint8
        or candidate.ndim != 3
        or candidate.shape[2] != 3
        or candidate.size == 0
        or not candidate.flags.c_contiguous
    ):
        raise ValueError(
            "Detector input must be a non-empty contiguous HxWx3 uint8 RGB image"
        )
    image_rgb = np.array(candidate, dtype=np.uint8, order="C", copy=True)
    camera_matrix, distortion = _validate_camera_parameters(payload)
    overlay_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    gray = np.array(
        cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY),
        dtype=np.uint8, order="C", copy=True,
    )
    charuco_parameters = cv2.aruco.CharucoParameters()
    charuco_parameters.cameraMatrix = camera_matrix
    charuco_parameters.distCoeffs = distortion
    charuco_parameters.minMarkers = 2
    charuco_parameters.tryRefineMarkers = False
    context.detector.setCharucoParameters(charuco_parameters)
    charuco_corners, charuco_ids, marker_corners, marker_ids = (
        context.detector.detectBoard(gray)
    )
    marker_count = _validate_detected_points(
        marker_corners, marker_ids, points_per_item=4, label="marker",
    )
    corner_count = _validate_detected_points(
        charuco_corners, charuco_ids, points_per_item=1, label="ChArUco",
    )
    solution_overlay = payload["solution_overlay"]

    def blocked(state: str, message: str) -> DetectionResult:
        _draw_measurement_overlay(overlay_bgr, (message,))
        return _finish_detection(
            frame_sequence=frame_sequence, state=state, message=message,
            corner_count=corner_count, camera_from_target=None,
            overlay_bgr=overlay_bgr, solution_overlay=solution_overlay,
        )

    if marker_count == 0:
        return blocked("not_visible", "No ChArUco board markers detected.")
    cv2.aruco.drawDetectedMarkers(overlay_bgr, marker_corners, marker_ids)
    if corner_count:
        ids = np.asarray(charuco_ids).reshape(-1)
        board_corner_count = len(context.board.getChessboardCorners())
        if (
            len(np.unique(ids)) != corner_count
            or np.any(ids < 0)
            or np.any(ids >= board_corner_count)
        ):
            raise ValueError("OpenCV returned duplicate or out-of-board ChArUco IDs")
        cv2.aruco.drawDetectedCornersCharuco(overlay_bgr, charuco_corners, charuco_ids)
    if corner_count < MINIMUM_CHARUCO_CORNERS:
        return blocked(
            "insufficient_corners",
            f"Detected {corner_count}/{MINIMUM_CHARUCO_CORNERS} required ChArUco corners.",
        )
    if context.board.checkCharucoCornersCollinear(charuco_ids):
        return blocked("collinear_corners", "Detected ChArUco corners lie on one line.")
    object_points, image_points = context.board.matchImagePoints(charuco_corners, charuco_ids)
    object_points = np.asarray(object_points, dtype=np.float32)
    image_points = np.asarray(image_points, dtype=np.float32)
    if (
        object_points.size != corner_count * 3
        or image_points.size != corner_count * 2
        or not np.all(np.isfinite(object_points))
        or not np.all(np.isfinite(image_points))
    ):
        raise ValueError("ChArUco board returned malformed pose correspondences")
    pose_ok, rotation_vector, translation_vector = cv2.solvePnP(
        object_points, image_points, camera_matrix, distortion,
        useExtrinsicGuess=False, flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not pose_ok:
        return blocked("pose_unavailable", "No valid RGB ChArUco board pose in this frame.")
    rotation_vector = np.asarray(rotation_vector, dtype=np.float64).reshape(3)
    translation_vector = np.asarray(translation_vector, dtype=np.float64).reshape(3)
    if not (
        np.all(np.isfinite(rotation_vector))
        and np.all(np.isfinite(translation_vector))
    ):
        raise InvalidPoseError("OpenCV returned a non-finite ChArUco board pose")
    rotation_matrix, _jacobian = cv2.Rodrigues(rotation_vector)
    camera_from_target = np.eye(4, dtype=np.float64)
    camera_from_target[:3, :3] = rotation_matrix
    camera_from_target[:3, 3] = translation_vector
    axis_length = 0.5 * min(context.settings.squares_x, context.settings.squares_y)
    axis_length *= context.settings.square_length_mm / 1000.0
    cv2.drawFrameAxes(
        overlay_bgr, camera_matrix, distortion, rotation_vector,
        translation_vector, axis_length, 2,
    )
    return _finish_detection(
        frame_sequence=frame_sequence, state="ready",
        message="RGB ChArUco pose ready. Hold the robot stationary for capture.",
        corner_count=corner_count, camera_from_target=camera_from_target,
        overlay_bgr=overlay_bgr, solution_overlay=solution_overlay,
    )


def detect_aruco_frame(
    payload: dict,
    context: _ArucoDetectorContext,
) -> ArucoDetectionResult:
    frame_sequence = payload["frame_sequence"]
    candidate = np.asarray(payload["image_rgb"])
    if (
        candidate.dtype != np.uint8
        or candidate.ndim != 3
        or candidate.shape[2] != 3
        or candidate.size == 0
        or not candidate.flags.c_contiguous
    ):
        raise ValueError(
            "ArUco input must be a non-empty contiguous HxWx3 uint8 RGB image"
        )
    image_rgb = np.array(candidate, dtype=np.uint8, order="C", copy=True)
    camera_matrix, distortion = _validate_camera_parameters(payload)
    overlay_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    gray = np.array(
        cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY),
        dtype=np.uint8,
        order="C",
        copy=True,
    )
    marker_corners, marker_ids, _rejected = context.detector.detectMarkers(gray)
    marker_count = _validate_detected_points(
        marker_corners,
        marker_ids,
        points_per_item=4,
        label="ArUco marker",
    )
    if marker_count:
        cv2.aruco.drawDetectedMarkers(overlay_bgr, marker_corners, marker_ids)
        ids_array = np.asarray(marker_ids, dtype=np.int64).reshape(-1)
        if len(np.unique(ids_array)) != marker_count or np.any(ids_array < 0):
            raise ValueError("OpenCV returned duplicate or negative ArUco IDs")
        detected_ids = tuple(sorted(int(value) for value in ids_array))
    else:
        ids_array = np.empty((0,), dtype=np.int64)
        detected_ids = ()

    def finish(
        state: str,
        message: str,
        observations: tuple[ArucoMarkerObservation, ...] = (),
    ) -> ArucoDetectionResult:
        _draw_measurement_overlay(overlay_bgr, (message,))
        overlay_rgb = cv2.cvtColor(overlay_bgr, cv2.COLOR_BGR2RGB)
        return ArucoDetectionResult(
            frame_sequence=frame_sequence,
            state=state,
            message=message,
            detected_ids=detected_ids,
            observations=observations,
            overlay_rgb=np.array(
                overlay_rgb,
                dtype=np.uint8,
                order="C",
                copy=True,
            ),
        )

    if marker_count == 0:
        return finish("not_visible", "No ArUco markers detected; require IDs 0,1,2,3.")
    detected_set = set(detected_ids)
    required_set = set(REQUIRED_BIN_MARKER_IDS)
    unexpected = sorted(detected_set - required_set)
    if unexpected:
        return finish(
            "unexpected_ids",
            f"Unexpected ArUco IDs {unexpected}; visible set must be exactly 0,1,2,3.",
        )
    missing = sorted(required_set - detected_set)
    if missing:
        return finish(
            "required_ids_missing",
            f"Missing required ArUco IDs {missing}; detected {list(detected_ids)}.",
        )

    index_by_id = {
        int(marker_id): index for index, marker_id in enumerate(ids_array.tolist())
    }
    observations = []
    object_corners = np.asarray(context.object_corners_m, dtype=np.float64)
    for marker_id in REQUIRED_BIN_MARKER_IDS:
        image_corners = np.asarray(
            marker_corners[index_by_id[marker_id]],
            dtype=np.float64,
        ).reshape(4, 2)
        pose_ok, rotation_vector, translation_vector = cv2.solvePnP(
            context.object_corners_m,
            image_corners,
            camera_matrix,
            distortion,
            useExtrinsicGuess=False,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not pose_ok:
            return finish(
                "pose_unavailable",
                f"No valid pose for required ArUco marker ID {marker_id}.",
            )
        rotation_vector = np.asarray(rotation_vector, dtype=np.float64).reshape(3)
        translation_vector = np.asarray(translation_vector, dtype=np.float64).reshape(3)
        if not (
            np.all(np.isfinite(rotation_vector))
            and np.all(np.isfinite(translation_vector))
        ):
            raise InvalidPoseError(
                f"OpenCV returned a non-finite pose for ArUco marker ID {marker_id}"
            )
        if translation_vector[2] <= 0.0:
            return finish(
                "pose_unavailable",
                f"ArUco marker ID {marker_id} pose is not in front of the camera.",
            )
        rotation_matrix, _jacobian = cv2.Rodrigues(rotation_vector)
        optical_from_marker = np.eye(4, dtype=np.float64)
        optical_from_marker[:3, :3] = rotation_matrix
        optical_from_marker[:3, 3] = translation_vector
        optical_corners = (
            (rotation_matrix @ object_corners.T).T + translation_vector.reshape(1, 3)
        )
        if not np.all(np.isfinite(optical_corners)):
            raise InvalidPoseError(
                f"OpenCV returned non-finite corners for ArUco marker ID {marker_id}"
            )
        cv2.drawFrameAxes(
            overlay_bgr,
            camera_matrix,
            distortion,
            rotation_vector,
            translation_vector,
            (float(context.settings.marker_size_mm) / 1000.0) * 0.5,
            2,
        )
        observations.append(
            ArucoMarkerObservation(
                marker_id=marker_id,
                image_corners_px=np.array(
                    image_corners, dtype=np.float64, order="C", copy=True
                ),
                optical_corners_m=np.array(
                    optical_corners, dtype=np.float64, order="C", copy=True
                ),
                optical_from_marker=np.array(
                    optical_from_marker, dtype=np.float64, order="C", copy=True
                ),
            )
        )
    return finish(
        "ready",
        "ArUco IDs 0,1,2,3 ready; outside-corner ROI is available.",
        tuple(observations),
    )


def solve(payload: dict) -> SolveResult:
    calibration_mode = payload["calibration_mode"]
    samples: list[CalibrationSample] = payload["samples"]
    camera_link_from_optical = np.asarray(
        payload["camera_link_from_optical"],
        dtype=np.float64,
    )
    previous_solution = payload["previous_solution"]
    previous_quality = payload["previous_quality"]
    reference_from_optical, quality = solve_calibration(calibration_mode, samples)
    reference_from_camera_link = reference_from_optical @ invert_transform(
        camera_link_from_optical
    )
    change = solution_change_diagnostics(
        previous_solution,
        previous_quality,
        reference_from_camera_link,
        quality,
    )
    leave_one_out = leave_one_out_diagnostics(
        calibration_mode,
        samples,
        camera_link_from_optical,
        reference_from_camera_link,
    )
    diagnostics = AccuracyDiagnostics(
        quality,
        change,
        leave_one_out,
        pose_coverage_diagnostics(samples),
        ax_xb_diagnostics(calibration_mode, samples, reference_from_optical),
    )
    return SolveResult(reference_from_camera_link, diagnostics)


def _error_response(operation: str, request_id, exc: Exception) -> dict:
    return {
        "ok": False,
        "operation": operation,
        "request_id": request_id,
        "error_type": type(exc).__name__,
        "error": str(exc),
    }


def worker_main(connection, runtime_dir: Path) -> None:
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        try:
            runtime = configure_opencv_runtime(runtime_dir)
        except Exception as exc:
            connection.send(_error_response("startup", None, exc))
            return
        connection.send({"ok": True, "operation": "startup", "payload": runtime})
        context = None
        aruco_context = None
        while True:
            request = connection.recv()
            request_id = request["request_id"]
            operation = request["operation"]
            payload = request["payload"]
            if operation == "shutdown":
                connection.send(
                    {
                        "ok": True,
                        "operation": operation,
                        "request_id": request_id,
                        "payload": None,
                    }
                )
                return
            try:
                _verify_runtime(runtime_dir)
                if operation == "configure":
                    context = create_detector_context(payload)
                    result = None
                elif operation == "configure_aruco":
                    aruco_context = create_aruco_detector_context(payload)
                    result = None
                elif operation == "detect":
                    if context is None:
                        raise RuntimeError("Detector settings have not been configured")
                    result = detect_frame(payload, context)
                elif operation == "detect_aruco":
                    if aruco_context is None:
                        raise RuntimeError("ArUco detector settings have not been configured")
                    result = detect_aruco_frame(payload, aruco_context)
                elif operation == "solve":
                    result = solve(payload)
                elif operation == "image_rays":
                    pixels = np.asarray(payload["image_points"], dtype=np.float64)
                    if (pixels.ndim != 2 or pixels.shape[1] != 2 or len(pixels) == 0
                            or not np.all(np.isfinite(pixels))):
                        raise ValueError("Image rays require finite Nx2 pixels")
                    normalized = cv2.undistortPoints(
                        pixels.reshape(-1, 1, 2),
                        payload["camera_matrix"], payload["distortion"],
                    ).reshape(-1, 2)
                    result = np.ascontiguousarray(
                        np.column_stack((normalized, np.ones(len(pixels)))), dtype=np.float64,
                    )
                elif operation == "project_points":
                    points = np.asarray(payload["optical_points"], dtype=np.float64)
                    if (
                        points.ndim != 2 or points.shape[1] != 3
                        or not np.all(np.isfinite(points))
                        or np.any(points[:, 2] <= 0.0)
                    ):
                        raise ValueError(
                            "Projection requires finite points in front of the camera"
                        )
                    pixels, _jacobian = cv2.projectPoints(
                        points, np.zeros(3), np.zeros(3),
                        payload["camera_matrix"], payload["distortion"],
                    )
                    result = np.ascontiguousarray(pixels.reshape(-1, 2), dtype=np.float64)
                else:
                    raise RuntimeError(
                        f"Unsupported OpenCV worker operation: {operation}"
                    )
            except Exception as exc:
                connection.send(_error_response(operation, request_id, exc))
            else:
                connection.send(
                    {
                        "ok": True,
                        "operation": operation,
                        "request_id": request_id,
                        "payload": result,
                    }
                )
    except (EOFError, BrokenPipeError):
        return
    finally:
        connection.close()
