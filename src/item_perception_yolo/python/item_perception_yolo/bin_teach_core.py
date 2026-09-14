import hashlib
import ipaddress
import math
import os
import re
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

from camera_calibration_gui.calibration_core import (
    BASE_FRAME,
    CALIBRATION_ARTIFACT_SCHEMA_VERSION,
    CAMERA_PREFIX_PATTERN,
    CAMERA_ON_HAND,
    CAMERA_TO_HAND,
    REQUIRED_OPENCV_VERSION,
    REQUIRED_OPENCV_WHEEL_SHA256,
    invert_transform,
    workspace_root,
)
from camera_calibration_gui.opencv_worker import (
    REQUIRED_ARUCO_API,
    REQUIRED_BIN_MARKER_IDS,
)

from .platform_teach_core import (
    PLATFORM_ARTIFACT_SCHEMA_VERSION,
    PLATFORM_FRAME,
    PLATFORM_REFERENCE_CONVENTION,
    PLATFORM_REFERENCE_DEFINITION,
    ROBOT_TF_MAX_AGE_SEC,
    TOOL_FRAME,
    AppliedCameraCalibration,
    PlatformCalibrationArtifact,
    _canonical_utc_value,
    _transform_from_payload,
    _transform_payload,
    _validate_nonzero_stamp,
    _validate_rigid_transform,
    calibration_directory,
    load_camera_calibration,
    load_platform_calibration,
    load_robot_lan1_ip,
    platform_output_path,
    resolve_base_from_camera_link,
)
from .ui_state import (
    ARUCO_5X5_DICTIONARIES,
    PACKAGE_UI_STATE_SCHEMA_VERSION,
    load_package_ui_state,
    write_bin_ui_state,
)
from . import planar_bin_roi


BIN_ARTIFACT_SCHEMA_VERSION = 3
BIN_UI_STATE_SCHEMA_VERSION = PACKAGE_UI_STATE_SCHEMA_VERSION
TARGET_MAX_AGE_SEC = 0.5
ROI_CORNER_SELECTION = "farthest_from_four_marker_center_centroid"
ROI_POINT_ORDER = "clockwise_from_lexicographically_smallest_xy"
ROI_BORDER_SAMPLES_PER_EDGE = planar_bin_roi.ROI_BORDER_SAMPLES_PER_EDGE
ROI_GEOMETRY_METHOD = "undistorted_image_ray_intersection_platform_z0"


@dataclass(frozen=True)
class BinArucoSettings:
    dictionary_name: str
    marker_size_mm: float

    def validate(self) -> None:
        if self.dictionary_name not in ARUCO_5X5_DICTIONARIES:
            raise ValueError("Bin teach requires an exact 5x5 ArUco dictionary")
        if (
            type(self.marker_size_mm) not in {int, float}
            or not math.isfinite(float(self.marker_size_mm))
            or float(self.marker_size_mm) <= 0.0
        ):
            raise ValueError("ArUco marker size must be greater than zero millimetres")


@dataclass(frozen=True)
class AppliedBinTeachCalibration:
    platform: PlatformCalibrationArtifact
    camera: AppliedCameraCalibration


@dataclass(frozen=True)
class BinRoiPoint:
    x_m: float
    y_m: float
    source_marker_id: int
    source_marker_corner_index: int


@dataclass(frozen=True)
class BinTeachCapture:
    captured_at_utc: str
    color_stamp_sec: int
    color_stamp_nanosec: int
    frame_sequence: int
    calibration_mode: str
    calibration_reference_from_camera_link: np.ndarray
    base_from_tool: np.ndarray | None
    robot_tf_stamp_sec: int | None
    robot_tf_stamp_nanosec: int | None
    base_from_camera_link: np.ndarray
    camera_link_from_optical: np.ndarray
    platform_from_optical: np.ndarray
    points: tuple[BinRoiPoint, BinRoiPoint, BinRoiPoint, BinRoiPoint]


@dataclass(frozen=True)
class BinTeachUiState:
    saved_at_utc: str
    platform_calibration_filename: str
    dictionary_name: str
    marker_size_mm: float


@dataclass(frozen=True)
class BinTeachArtifact:
    path: Path
    sha256: str
    created_at_utc: str
    source_robot_lan1_ip: str
    source_platform_calibration_filename: str
    source_platform_calibration_sha256: str
    source_camera_calibration_mode: str
    reference_convention: str
    dictionary_name: str
    marker_size_mm: float
    points: tuple[BinRoiPoint, BinRoiPoint, BinRoiPoint, BinRoiPoint]


def bin_platform_warning(
    template: BinTeachArtifact,
    platform: PlatformCalibrationArtifact,
) -> str | None:
    """Compare recorded file identity, not physical alignment or deployment eligibility."""
    if template.source_platform_calibration_sha256 == platform.sha256:
        return None
    return (
        "WARNING: Bin/platform mismatch — different platform calibration.\n"
        f"Taught with: {template.source_platform_calibration_filename}\n"
        f"Selected: {platform.path.name}\n"
        "Platform SHA-256 checksums differ. Portable reuse is allowed; verify the "
        "same physical origin, X/Y directions, bin size and placement. "
        "The ROI uses the selected platform, not the teaching station's transform."
    )


def place_bin_roi(
    template: BinTeachArtifact,
    platform: PlatformCalibrationArtifact,
) -> np.ndarray:
    """Place portable XY geometry using only the explicitly selected platform."""
    if (
        template.reference_convention != PLATFORM_REFERENCE_CONVENTION
        or platform.reference_convention != template.reference_convention
        or platform.platform_frame != PLATFORM_FRAME
    ):
        raise ValueError("Bin ROI and station platform reference conventions differ")
    _validate_ordered_roi_points(template.points)
    transform = _validate_rigid_transform(
        platform.base_from_platform, "Selected station platform transform"
    )
    points = np.asarray(
        [(point.x_m, point.y_m, 0.0, 1.0) for point in template.points],
        dtype=np.float64,
    )
    return (transform @ points.T).T[:, :3]


def bin_border_in_optical(points, platform_from_optical: np.ndarray) -> np.ndarray:
    """Sample the saved platform-plane border before lens-distorted projection."""
    _validate_ordered_roi_points(points)
    transform = _validate_rigid_transform(
        platform_from_optical, "Platform from current optical frame"
    )
    return planar_bin_roi.border_in_optical(
        [(point.x_m, point.y_m) for point in points], transform, np,
    )


def load_bin_teach_calibration_context(
    platform_path: Path,
    root: Path | None = None,
) -> AppliedBinTeachCalibration:
    platform = load_platform_calibration(platform_path, root=root)
    robot_lan1_ip = load_robot_lan1_ip(root)
    if platform.robot_lan1_ip != robot_lan1_ip:
        raise ValueError(
            "Platform calibration robot identity conflicts with root .env: "
            f"artifact={platform.robot_lan1_ip}, configured={robot_lan1_ip}"
        )
    camera_path = calibration_directory(root) / platform.camera_calibration_filename
    camera = load_camera_calibration(camera_path, root=root)
    if camera.sha256 != platform.camera_calibration_sha256:
        raise ValueError(
            "Platform calibration references a camera calibration with a different "
            "SHA-256"
        )
    if camera.calibration_mode != platform.camera_calibration_mode:
        raise ValueError(
            "Platform and camera calibration modes conflict: "
            f"platform={platform.camera_calibration_mode}, "
            f"camera={camera.calibration_mode}"
        )
    if camera.settings != platform.camera_settings:
        raise ValueError("Platform and camera calibration settings conflict")
    if not np.allclose(
        camera.reference_from_camera_link,
        platform.calibration_reference_from_camera_link,
        atol=1e-9,
    ):
        raise ValueError("Platform and camera mounting transforms conflict")
    return AppliedBinTeachCalibration(platform=platform, camera=camera)


def validate_applied_sources(
    applied: AppliedBinTeachCalibration,
    root: Path | None = None,
) -> None:
    if applied.platform.reference_convention != PLATFORM_REFERENCE_CONVENTION:
        raise ValueError("Applied platform reference convention is invalid")
    required_directory = calibration_directory(root).resolve()
    if (
        applied.platform.path.parent != required_directory
        or applied.camera.path.parent != required_directory
    ):
        raise ValueError("Applied calibration sources escaped root calibration/")
    configured_robot_ip = load_robot_lan1_ip(root)
    if configured_robot_ip != applied.platform.robot_lan1_ip:
        raise ValueError(
            "Root .env robot identity changed after Bin Teach settings were applied"
        )
    for label, path, expected_digest in (
        ("platform", applied.platform.path, applied.platform.sha256),
        ("camera", applied.camera.path, applied.camera.sha256),
    ):
        try:
            actual_digest = hashlib.sha256(path.read_bytes()).hexdigest()
        except OSError as exc:
            raise ValueError(f"Applied {label} calibration is unavailable: {exc}") from exc
        if actual_digest != expected_digest:
            raise ValueError(
                f"Selected {label} calibration changed after it was applied"
            )
    if applied.camera.sha256 != applied.platform.camera_calibration_sha256:
        raise ValueError("Platform and camera calibration SHA-256 values conflict")
    if applied.camera.calibration_mode != applied.platform.camera_calibration_mode:
        raise ValueError("Platform and camera calibration modes conflict")
    if applied.camera.settings != applied.platform.camera_settings:
        raise ValueError("Platform and camera calibration settings conflict")
    if not np.allclose(
        applied.camera.reference_from_camera_link,
        applied.platform.calibration_reference_from_camera_link,
        atol=1e-9,
    ):
        raise ValueError("Platform and camera mounting transforms conflict")


def compose_platform_from_optical(
    base_from_platform,
    base_from_camera_link,
    camera_link_from_optical,
) -> np.ndarray:
    base_from_platform = _validate_rigid_transform(
        base_from_platform,
        "base_link from platform_reference",
    )
    base_from_camera_link = _validate_rigid_transform(
        base_from_camera_link,
        "base_link from camera_link",
    )
    camera_link_from_optical = _validate_rigid_transform(
        camera_link_from_optical,
        "camera_link from optical frame",
    )
    return _validate_rigid_transform(
        invert_transform(base_from_platform)
        @ base_from_camera_link
        @ camera_link_from_optical,
        "platform_reference from optical frame",
    )


def project_marker_rays_to_plane(
    platform_from_optical,
    optical_rays_by_id: dict[int, np.ndarray],
) -> dict[int, np.ndarray]:
    """Intersect detected-corner rays with the unchanged platform's tilted XY plane."""
    transform = _validate_rigid_transform(
        platform_from_optical,
        "platform_reference from optical frame",
    )
    if set(optical_rays_by_id) != set(REQUIRED_BIN_MARKER_IDS):
        raise ValueError("Marker-corner input must contain exact IDs 0, 1, 2, and 3")
    output = {}
    for marker_id in REQUIRED_BIN_MARKER_IDS:
        rays = np.asarray(optical_rays_by_id[marker_id], dtype=np.float64)
        if (rays.shape != (4, 3) or not np.all(np.isfinite(rays))
                or not np.all(rays[:, 2] == 1.0)):
            raise ValueError(f"Marker ID {marker_id} requires four finite optical rays (x,y,1)")
        directions = rays @ transform[:3, :3].T
        if np.any(np.abs(directions[:, 2]) <= 1e-9):
            raise ValueError(f"Marker ID {marker_id}: viewing ray is parallel to platform plane")
        distances = -transform[2, 3] / directions[:, 2]
        if not np.all(np.isfinite(distances)) or np.any(distances <= 0.0):
            raise ValueError(f"Marker ID {marker_id}: platform intersection is behind camera")
        corners = transform[:3, 3] + distances[:, None] * directions
        if not np.all(np.isfinite(corners)):
            raise ValueError(f"Marker ID {marker_id}: non-finite platform-plane intersection")
        # Z=0 follows from the ray/plane intersection, not from dropping a PnP Z.
        corners[:, 2] = 0.0
        output[marker_id] = np.ascontiguousarray(corners)
    return output


def _polygon_signed_area(points: np.ndarray) -> float:
    return 0.5 * float(
        np.sum(
            (points[:, 0] * np.roll(points[:, 1], -1))
            - (np.roll(points[:, 0], -1) * points[:, 1])
        )
    )


def _validate_ordered_roi_points(points: tuple[BinRoiPoint, ...]) -> None:
    if len(points) != 4:
        raise ValueError("Bin ROI must contain exactly four points")
    marker_ids = tuple(point.source_marker_id for point in points)
    if set(marker_ids) != set(REQUIRED_BIN_MARKER_IDS):
        raise ValueError("Bin ROI points must originate once from IDs 0, 1, 2, and 3")
    coordinates = np.asarray([(point.x_m, point.y_m) for point in points], dtype=np.float64)
    if not np.all(np.isfinite(coordinates)):
        raise ValueError("Bin ROI points must be finite")
    if any(
        type(point.source_marker_corner_index) is not int
        or not 0 <= point.source_marker_corner_index < 4
        for point in points
    ):
        raise ValueError("Bin ROI marker-corner indices must be integers from 0 to 3")
    if min(
        np.linalg.norm(coordinates[index] - coordinates[other])
        for index in range(4)
        for other in range(index + 1, 4)
    ) <= 1e-6:
        raise ValueError("Bin ROI points are duplicate or too close")
    area = _polygon_signed_area(coordinates)
    if area >= -1e-10:
        raise ValueError("Bin ROI points must be in strict clockwise order")
    cross_products = []
    for index in range(4):
        first = coordinates[(index + 1) % 4] - coordinates[index]
        second = coordinates[(index + 2) % 4] - coordinates[(index + 1) % 4]
        cross_products.append((first[0] * second[1]) - (first[1] * second[0]))
    if not all(value < -1e-10 for value in cross_products):
        raise ValueError("Bin ROI must be a non-degenerate convex quadrilateral")


def select_outside_roi_corners(
    platform_corners_by_id: dict[int, np.ndarray],
) -> tuple[BinRoiPoint, BinRoiPoint, BinRoiPoint, BinRoiPoint]:
    if set(platform_corners_by_id) != set(REQUIRED_BIN_MARKER_IDS):
        raise ValueError("Outside-corner selection requires exact IDs 0, 1, 2, and 3")
    validated = {}
    marker_centers = []
    for marker_id in REQUIRED_BIN_MARKER_IDS:
        corners = np.asarray(platform_corners_by_id[marker_id], dtype=np.float64)
        if corners.shape != (4, 3) or not np.all(np.isfinite(corners)):
            raise ValueError(f"Marker ID {marker_id} must contain four finite 3D corners")
        validated[marker_id] = corners
        marker_centers.append(np.mean(corners[:, :2], axis=0))
    group_center = np.mean(np.asarray(marker_centers), axis=0)
    selected = []
    for marker_id in REQUIRED_BIN_MARKER_IDS:
        corners = validated[marker_id]
        squared_distances = np.sum((corners[:, :2] - group_center) ** 2, axis=1)
        order = np.argsort(squared_distances)
        if squared_distances[order[-1]] - squared_distances[order[-2]] <= 1e-12:
            raise ValueError(
                f"Marker ID {marker_id} has an ambiguous outside corner relative "
                "to the four-marker center"
            )
        corner_index = int(order[-1])
        selected.append(
            BinRoiPoint(
                x_m=float(corners[corner_index, 0]),
                y_m=float(corners[corner_index, 1]),
                source_marker_id=marker_id,
                source_marker_corner_index=corner_index,
            )
        )
    selected_center = np.mean(
        np.asarray([(point.x_m, point.y_m) for point in selected]),
        axis=0,
    )
    ordered = sorted(
        selected,
        key=lambda point: math.atan2(
            point.y_m - selected_center[1],
            point.x_m - selected_center[0],
        ),
        reverse=True,
    )
    start = min(
        range(4),
        key=lambda index: (ordered[index].x_m, ordered[index].y_m),
    )
    ordered = tuple(ordered[start:] + ordered[:start])
    _validate_ordered_roi_points(ordered)
    return ordered


def bin_teach_directory(root: Path | None = None) -> Path:
    project_root = workspace_root() if root is None else Path(root).resolve()
    return project_root / "offline_teach" / "bin_teach"


def bin_output_path(
    robot_lan1_ip: str,
    root: Path | None = None,
    created_at: datetime | None = None,
) -> Path:
    try:
        address = ipaddress.ip_address(robot_lan1_ip)
    except ValueError as exc:
        raise ValueError("Robot LAN1 identity must be a valid IPv4 address") from exc
    if address.version != 4:
        raise ValueError("Robot LAN1 identity must be IPv4")
    timestamp = created_at or datetime.now(timezone.utc)
    if timestamp.tzinfo is None:
        raise ValueError("Bin-teach output timestamp must include a timezone")
    stamp = timestamp.astimezone(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    return bin_teach_directory(root) / f"bin_teach_{stamp}_{address}.yaml"


def write_bin_teach(
    path: Path,
    applied: AppliedBinTeachCalibration,
    aruco_settings: BinArucoSettings,
    capture: BinTeachCapture,
    root: Path | None = None,
) -> None:
    aruco_settings.validate()
    output_path = Path(path).resolve()
    required_directory = bin_teach_directory(root).resolve()
    if output_path.parent != required_directory:
        raise ValueError("Bin-teach output must be in root offline_teach/bin_teach/")
    if output_path.exists():
        raise ValueError(f"Bin-teach output already exists: {output_path}")
    validate_applied_sources(applied, root=root)
    _validate_ordered_roi_points(capture.points)
    if capture.frame_sequence < 1:
        raise ValueError("Bin-teach capture frame sequence must be positive")
    if (
        type(capture.color_stamp_sec) is not int
        or type(capture.color_stamp_nanosec) is not int
        or capture.color_stamp_sec < 0
        or not 0 <= capture.color_stamp_nanosec < 1_000_000_000
        or (capture.color_stamp_sec == 0 and capture.color_stamp_nanosec == 0)
    ):
        raise ValueError("Bin-teach capture color timestamp is invalid")
    if capture.calibration_mode != applied.camera.calibration_mode:
        raise ValueError("Bin capture camera mode conflicts with applied calibration")
    calibration_reference_from_camera_link = _validate_rigid_transform(
        capture.calibration_reference_from_camera_link,
        "Bin capture calibrated-reference from camera_link",
    )
    if not np.allclose(
        calibration_reference_from_camera_link,
        applied.camera.reference_from_camera_link,
        atol=1e-12,
    ):
        raise ValueError("Bin capture mounting transform conflicts with calibration")
    if applied.camera.calibration_mode == CAMERA_TO_HAND:
        if (
            capture.base_from_tool is not None
            or capture.robot_tf_stamp_sec is not None
            or capture.robot_tf_stamp_nanosec is not None
        ):
            raise ValueError("Camera-to-hand bin capture must not contain robot TF")
        expected_base_from_camera_link = resolve_base_from_camera_link(applied.camera)
        robot_tf_payload = {"required": False}
    elif applied.camera.calibration_mode == CAMERA_ON_HAND:
        if capture.base_from_tool is None:
            raise ValueError("Camera-on-hand bin capture requires robot TF")
        _validate_nonzero_stamp(
            capture.robot_tf_stamp_sec,
            capture.robot_tf_stamp_nanosec,
            "Bin capture robot TF timestamp",
        )
        base_from_tool = _validate_rigid_transform(
            capture.base_from_tool,
            f"Bin capture {BASE_FRAME} from {TOOL_FRAME}",
        )
        expected_base_from_camera_link = resolve_base_from_camera_link(
            applied.camera,
            base_from_tool,
        )
        robot_tf_payload = {
            "required": True,
            "target_frame": BASE_FRAME,
            "source_frame": TOOL_FRAME,
            "maximum_age_sec": ROBOT_TF_MAX_AGE_SEC,
            "stamp": {
                "sec": capture.robot_tf_stamp_sec,
                "nanosec": capture.robot_tf_stamp_nanosec,
            },
            "base_from_tool": _transform_payload(
                base_from_tool,
                f"Bin capture {BASE_FRAME} from {TOOL_FRAME}",
            ),
        }
    else:
        raise ValueError("Applied camera calibration mode is invalid")
    base_from_camera_link = _validate_rigid_transform(
        capture.base_from_camera_link,
        "Bin capture resolved base_link from camera_link",
    )
    if not np.allclose(
        base_from_camera_link,
        expected_base_from_camera_link,
        atol=1e-9,
    ):
        raise ValueError("Bin capture resolved camera transform conflicts with its chain")
    camera_link_from_optical = _validate_rigid_transform(
        capture.camera_link_from_optical,
        "Bin capture camera_link from optical frame",
    )
    expected_platform_from_optical = compose_platform_from_optical(
        applied.platform.base_from_platform,
        base_from_camera_link,
        camera_link_from_optical,
    )
    platform_from_optical = _validate_rigid_transform(
        capture.platform_from_optical,
        "Bin capture platform_reference from optical frame",
    )
    if not np.allclose(
        platform_from_optical,
        expected_platform_from_optical,
        atol=1e-9,
    ):
        raise ValueError("Bin capture optical transform conflicts with its chain")
    created_utc = _canonical_utc_value(capture.captured_at_utc, "Bin capture timestamp")
    created_at = datetime.fromisoformat(created_utc[:-1] + "+00:00")
    expected_path = bin_output_path(
        applied.platform.robot_lan1_ip,
        root=root,
        created_at=created_at,
    ).resolve()
    if output_path != expected_path:
        raise ValueError(f"Bin-teach output name must be exactly {expected_path.name}")
    camera_settings = applied.camera.settings
    payload = {
        "schema_version": BIN_ARTIFACT_SCHEMA_VERSION,
        "artifact_type": "bin_teach",
        "created_utc": created_utc,
        "reference": dict(PLATFORM_REFERENCE_DEFINITION),
        "robot": {
            "lan1_ip": applied.platform.robot_lan1_ip,
            "base_frame": BASE_FRAME,
        },
        "platform_calibration": {
            "filename": applied.platform.path.name,
            "sha256": applied.platform.sha256,
            "schema_version": PLATFORM_ARTIFACT_SCHEMA_VERSION,
            "created_utc": applied.platform.created_at_utc,
            "platform_frame": PLATFORM_FRAME,
        },
        "camera_calibration": {
            "filename": applied.camera.path.name,
            "sha256": applied.camera.sha256,
            "schema_version": CALIBRATION_ARTIFACT_SCHEMA_VERSION,
            "calibration_mode": applied.camera.calibration_mode,
            "reference_frame": applied.camera.reference_frame,
            "created_utc": applied.camera.created_at_utc,
        },
        "camera": {
            "prefix": camera_settings.camera_prefix,
            "link_frame": camera_settings.camera_link_frame,
            "optical_frame": camera_settings.optical_frame,
            "color_topic": camera_settings.color_topic,
            "camera_info_topic": camera_settings.camera_info_topic,
        },
        "aruco": {
            "dictionary": aruco_settings.dictionary_name,
            "marker_size_mm": float(aruco_settings.marker_size_mm),
            "required_marker_ids": list(REQUIRED_BIN_MARKER_IDS),
            "corner_selection": ROI_CORNER_SELECTION,
            "opencv_version": REQUIRED_OPENCV_VERSION,
            "opencv_wheel_sha256": REQUIRED_OPENCV_WHEEL_SHA256,
            "api": REQUIRED_ARUCO_API,
        },
        "roi": {
            "coordinate_frame": PLATFORM_FRAME,
            "units": "metres",
            "ordering": ROI_POINT_ORDER,
            "points": [
                {
                    "x": point.x_m,
                    "y": point.y_m,
                    "source_marker_id": point.source_marker_id,
                    "source_marker_corner_index": point.source_marker_corner_index,
                }
                for point in capture.points
            ],
        },
        "capture": {
            "base_from_platform": _transform_payload(
                applied.platform.base_from_platform,
                "Source station base_link from platform_reference",
            ),
            "frame_sequence": capture.frame_sequence,
            "color_stamp": {
                "sec": capture.color_stamp_sec,
                "nanosec": capture.color_stamp_nanosec,
            },
            "calibration_reference_from_camera_link": _transform_payload(
                calibration_reference_from_camera_link,
                "Bin capture calibrated-reference from camera_link",
            ),
            "robot_tf": robot_tf_payload,
            "base_from_camera_link": _transform_payload(
                base_from_camera_link,
                "Bin capture resolved base_link from camera_link",
            ),
            "camera_link_from_optical": _transform_payload(
                camera_link_from_optical,
                "Bin capture camera_link from optical frame",
            ),
            "platform_from_optical": _transform_payload(
                platform_from_optical,
                "Bin capture platform_reference from optical frame",
            ),
        },
    }
    # Source station evidence is retained, but is never a deployment binding.
    payload["teaching_provenance"] = {
        key: payload.pop(key)
        for key in (
            "robot", "platform_calibration", "camera_calibration",
            "camera", "aruco", "capture",
        )
    }
    content = yaml.safe_dump(payload, sort_keys=False, allow_unicode=False)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=output_path.parent,
            prefix=f".{output_path.name}.",
            delete=False,
        ) as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
            temporary_path = Path(stream.name)
        os.link(temporary_path, output_path)
        temporary_path.unlink()
        temporary_path = None
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def load_bin_ui_state(path: Path) -> BinTeachUiState | None:
    state = load_package_ui_state(path)
    if state is None or state.bin_platform_calibration_filename is None:
        return None
    return BinTeachUiState(
        saved_at_utc=state.saved_at_utc,
        platform_calibration_filename=state.bin_platform_calibration_filename,
        dictionary_name=state.bin_aruco_dictionary,
        marker_size_mm=state.bin_marker_size_mm,
    )


def write_bin_state(
    path: Path,
    platform_calibration_filename: str,
    dictionary_name: str,
    marker_size_mm: float,
    saved_at: datetime | None = None,
) -> BinTeachUiState:
    state = write_bin_ui_state(
        path,
        platform_calibration_filename,
        dictionary_name,
        marker_size_mm,
        saved_at=saved_at,
    )
    return BinTeachUiState(
        saved_at_utc=state.saved_at_utc,
        platform_calibration_filename=state.bin_platform_calibration_filename,
        dictionary_name=state.bin_aruco_dictionary,
        marker_size_mm=state.bin_marker_size_mm,
    )


def load_bin_teach(path: Path, root: Path | None = None, *, deployment=False) -> BinTeachArtifact:
    if Path(path).expanduser().absolute().is_symlink():
        raise ValueError("Bin-teach artifact must be a regular file, not a symlink")
    candidate = Path(path).expanduser().resolve()
    project_root = workspace_root() if root is None else Path(root).resolve()
    directory = project_root / "runtime_teach" if deployment else bin_teach_directory(root)
    if directory.is_symlink():
        raise ValueError("Bin-teach directory must not be a symlink")
    directory = directory.resolve()
    if candidate.parent != directory or not candidate.is_file():
        location = "runtime_teach/" if deployment else "offline_teach/bin_teach/"
        raise ValueError(f"Bin-teach artifact must be a file in root {location}")
    try:
        content = candidate.read_bytes()
        payload = yaml.safe_load(content.decode("utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f"Cannot read bin-teach artifact {candidate}: {exc}") from exc
    required_root = {
        "schema_version",
        "artifact_type",
        "created_utc",
        "reference",
        "roi",
        "teaching_provenance",
    }
    if isinstance(payload, dict) and (
        type(payload.get("schema_version")) is not int
        or payload.get("schema_version") != BIN_ARTIFACT_SCHEMA_VERSION
    ):
        raise ValueError(f"Bin-teach schema_version must be {BIN_ARTIFACT_SCHEMA_VERSION}")
    if not isinstance(payload, dict) or set(payload) != required_root:
        raise ValueError("Bin-teach artifact has unsupported or missing root fields")
    if payload["artifact_type"] != "bin_teach":
        raise ValueError("Bin-teach artifact_type is invalid")
    if payload["reference"] != PLATFORM_REFERENCE_DEFINITION:
        raise ValueError("Bin-teach reference convention is invalid")
    provenance = payload["teaching_provenance"]
    if not isinstance(provenance, dict) or set(provenance) != {
        "robot", "platform_calibration", "camera_calibration",
        "camera", "aruco", "capture",
    }:
        raise ValueError("Bin-teach teaching provenance fields are invalid")
    created_utc = _canonical_utc_value(payload["created_utc"], "Bin created_utc")
    robot = provenance["robot"]
    if not isinstance(robot, dict) or set(robot) != {"lan1_ip", "base_frame"}:
        raise ValueError("Bin-teach robot fields are invalid")
    if robot["base_frame"] != BASE_FRAME:
        raise ValueError(f"Bin-teach base frame must be {BASE_FRAME}")
    try:
        address = ipaddress.ip_address(robot["lan1_ip"])
    except (TypeError, ValueError) as exc:
        raise ValueError("Bin-teach robot LAN1 IP is invalid") from exc
    if address.version != 4:
        raise ValueError("Bin-teach robot LAN1 IP must be IPv4")
    platform = provenance["platform_calibration"]
    if not isinstance(platform, dict) or set(platform) != {
        "filename", "sha256", "schema_version", "created_utc", "platform_frame"
    }:
        raise ValueError("Bin-teach platform-calibration fields are invalid")
    platform_filename = platform["filename"]
    if (
        type(platform_filename) is not str
        or Path(platform_filename).name != platform_filename
        or not platform_filename.startswith("platform_calibration_")
        or Path(platform_filename).suffix != ".yaml"
    ):
        raise ValueError("Bin-teach platform-calibration filename is invalid")
    if platform["schema_version"] != PLATFORM_ARTIFACT_SCHEMA_VERSION:
        raise ValueError("Bin-teach platform-calibration schema is invalid")
    if platform["platform_frame"] != PLATFORM_FRAME:
        raise ValueError("Bin-teach platform frame is invalid")
    if re.fullmatch(r"[0-9a-f]{64}", str(platform["sha256"])) is None:
        raise ValueError("Bin-teach platform SHA-256 is invalid")
    _canonical_utc_value(platform["created_utc"], "Platform source created_utc")
    source_platform_date = datetime.fromisoformat(platform["created_utc"][:-1] + "+00:00")
    if platform_filename != platform_output_path(
        str(address), root=root, created_at=source_platform_date
    ).name:
        raise ValueError("Bin-teach platform filename conflicts with teaching provenance")
    camera_calibration = provenance["camera_calibration"]
    if not isinstance(camera_calibration, dict) or set(camera_calibration) != {
        "filename",
        "sha256",
        "schema_version",
        "calibration_mode",
        "reference_frame",
        "created_utc",
    }:
        raise ValueError("Bin-teach camera-calibration fields are invalid")
    calibration_mode = camera_calibration["calibration_mode"]
    expected_prefixes = {
        CAMERA_TO_HAND: "camera_to_hand_calibration_",
        CAMERA_ON_HAND: "camera_on_hand_calibration_",
    }
    expected_reference_frame = (
        BASE_FRAME if calibration_mode == CAMERA_TO_HAND else TOOL_FRAME
    )
    if (
        camera_calibration["schema_version"] != CALIBRATION_ARTIFACT_SCHEMA_VERSION
        or calibration_mode not in expected_prefixes
        or camera_calibration["reference_frame"] != expected_reference_frame
        or re.fullmatch(r"[0-9a-f]{64}", str(camera_calibration["sha256"])) is None
    ):
        raise ValueError("Bin-teach camera-calibration provenance is invalid")
    camera_filename = camera_calibration["filename"]
    if (
        type(camera_filename) is not str
        or Path(camera_filename).name != camera_filename
        or not camera_filename.startswith(expected_prefixes[calibration_mode])
        or Path(camera_filename).suffix != ".yaml"
    ):
        raise ValueError("Bin-teach camera-calibration filename is invalid")
    _canonical_utc_value(camera_calibration["created_utc"], "Camera source created_utc")
    camera = provenance["camera"]
    if not isinstance(camera, dict) or set(camera) != {
        "prefix", "link_frame", "optical_frame", "color_topic", "camera_info_topic"
    }:
        raise ValueError("Bin-teach camera fields are invalid")
    prefix = camera["prefix"]
    if type(prefix) is not str or CAMERA_PREFIX_PATTERN.fullmatch(prefix) is None:
        raise ValueError("Bin-teach camera prefix is invalid")
    expected_camera = {
        "link_frame": f"{prefix}_link",
        "optical_frame": f"{prefix}_color_optical_frame",
        "color_topic": f"/{prefix}/color/image_raw",
        "camera_info_topic": f"/{prefix}/color/camera_info",
    }
    for key, expected in expected_camera.items():
        if camera[key] != expected:
            raise ValueError(f"Bin-teach camera.{key} conflicts with prefix")
    aruco = provenance["aruco"]
    if not isinstance(aruco, dict) or set(aruco) != {
        "dictionary",
        "marker_size_mm",
        "required_marker_ids",
        "corner_selection",
        "opencv_version",
        "opencv_wheel_sha256",
        "api",
    }:
        raise ValueError("Bin-teach ArUco fields are invalid")
    settings = BinArucoSettings(aruco["dictionary"], aruco["marker_size_mm"])
    settings.validate()
    if (
        not isinstance(aruco["required_marker_ids"], list)
        or tuple(aruco["required_marker_ids"]) != REQUIRED_BIN_MARKER_IDS
    ):
        raise ValueError("Bin-teach required marker IDs must be exactly 0,1,2,3")
    expected_aruco = {
        "corner_selection": ROI_CORNER_SELECTION,
        "opencv_version": REQUIRED_OPENCV_VERSION,
        "opencv_wheel_sha256": REQUIRED_OPENCV_WHEEL_SHA256,
        "api": REQUIRED_ARUCO_API,
    }
    for key, expected in expected_aruco.items():
        if aruco[key] != expected:
            raise ValueError(f"Bin-teach aruco.{key} is not canonical")
    roi = payload["roi"]
    if not isinstance(roi, dict) or set(roi) != {
        "coordinate_frame", "units", "ordering", "points"
    }:
        raise ValueError("Bin-teach ROI fields are invalid")
    if (
        roi["coordinate_frame"] != PLATFORM_FRAME
        or roi["units"] != "metres"
        or roi["ordering"] != ROI_POINT_ORDER
        or not isinstance(roi["points"], list)
        or len(roi["points"]) != 4
    ):
        raise ValueError("Bin-teach ROI contract is invalid")
    points = []
    for item in roi["points"]:
        if not isinstance(item, dict) or set(item) != {
            "x", "y", "source_marker_id", "source_marker_corner_index"
        }:
            raise ValueError("Bin-teach ROI point fields are invalid")
        if type(item["x"]) not in {int, float} or type(item["y"]) not in {int, float}:
            raise ValueError("Bin-teach ROI XY values must be numeric")
        points.append(
            BinRoiPoint(
                float(item["x"]),
                float(item["y"]),
                item["source_marker_id"],
                item["source_marker_corner_index"],
            )
        )
    points_tuple = tuple(points)
    _validate_ordered_roi_points(points_tuple)
    capture = provenance["capture"]
    if not isinstance(capture, dict) or set(capture) != {
        "base_from_platform",
        "frame_sequence",
        "color_stamp",
        "calibration_reference_from_camera_link",
        "robot_tf",
        "base_from_camera_link",
        "camera_link_from_optical",
        "platform_from_optical",
    }:
        raise ValueError("Bin-teach capture fields are invalid")
    if type(capture["frame_sequence"]) is not int or capture["frame_sequence"] < 1:
        raise ValueError("Bin-teach capture frame sequence must be positive")
    stamp = capture["color_stamp"]
    if not isinstance(stamp, dict) or set(stamp) != {"sec", "nanosec"}:
        raise ValueError("Bin-teach capture timestamp fields are invalid")
    _validate_nonzero_stamp(
        stamp["sec"], stamp["nanosec"], "Bin-teach capture timestamp"
    )
    calibration_reference_from_camera_link = _transform_from_payload(
        capture["calibration_reference_from_camera_link"],
        "Bin capture calibrated-reference from camera_link",
    )
    source_base_from_platform = _transform_from_payload(
        capture["base_from_platform"], "Source station platform transform"
    )
    robot_tf = capture["robot_tf"]
    if calibration_mode == CAMERA_TO_HAND:
        if robot_tf != {"required": False}:
            raise ValueError("Camera-to-hand bin robot_tf must be exactly unused")
        expected_base_from_camera_link = calibration_reference_from_camera_link
    else:
        required_robot_tf_fields = {
            "required",
            "target_frame",
            "source_frame",
            "maximum_age_sec",
            "stamp",
            "base_from_tool",
        }
        if not isinstance(robot_tf, dict) or set(robot_tf) != required_robot_tf_fields:
            raise ValueError("Camera-on-hand bin robot_tf fields are invalid")
        if (
            robot_tf["required"] is not True
            or robot_tf["target_frame"] != BASE_FRAME
            or robot_tf["source_frame"] != TOOL_FRAME
            or robot_tf["maximum_age_sec"] != ROBOT_TF_MAX_AGE_SEC
        ):
            raise ValueError("Camera-on-hand bin robot TF contract is invalid")
        robot_stamp = robot_tf["stamp"]
        if not isinstance(robot_stamp, dict) or set(robot_stamp) != {"sec", "nanosec"}:
            raise ValueError("Camera-on-hand bin robot TF stamp fields are invalid")
        _validate_nonzero_stamp(
            robot_stamp["sec"],
            robot_stamp["nanosec"],
            "Camera-on-hand bin robot TF timestamp",
        )
        base_from_tool = _transform_from_payload(
            robot_tf["base_from_tool"],
            f"Bin capture {BASE_FRAME} from {TOOL_FRAME}",
        )
        expected_base_from_camera_link = _validate_rigid_transform(
            base_from_tool @ calibration_reference_from_camera_link,
            "Bin capture resolved base_link from camera_link",
        )
    base_from_camera_link = _transform_from_payload(
        capture["base_from_camera_link"],
        "Bin capture resolved base_link from camera_link",
    )
    if not np.allclose(
        base_from_camera_link,
        expected_base_from_camera_link,
        atol=1e-9,
    ):
        raise ValueError("Bin capture resolved camera transform conflicts with its chain")
    camera_link_from_optical = _transform_from_payload(
        capture["camera_link_from_optical"],
        "Bin capture camera_link from optical frame",
    )
    platform_from_optical = _transform_from_payload(
        capture["platform_from_optical"],
        "Bin capture platform_reference from optical frame",
    )
    expected_platform_from_optical = compose_platform_from_optical(
        source_base_from_platform,
        base_from_camera_link,
        camera_link_from_optical,
    )
    if not np.allclose(
        platform_from_optical,
        expected_platform_from_optical,
        atol=1e-9,
    ):
        raise ValueError("Bin capture optical transform conflicts with its chain")
    parsed_created_at = datetime.fromisoformat(created_utc[:-1] + "+00:00")
    expected_path = bin_output_path(str(address), root=root, created_at=parsed_created_at)
    if candidate != directory / expected_path.name:
        raise ValueError(f"Bin-teach filename must be exactly {expected_path.name}")
    return BinTeachArtifact(
        path=candidate,
        sha256=hashlib.sha256(content).hexdigest(),
        created_at_utc=created_utc,
        source_robot_lan1_ip=str(address),
        source_platform_calibration_filename=platform_filename,
        source_platform_calibration_sha256=platform["sha256"],
        source_camera_calibration_mode=calibration_mode,
        reference_convention=PLATFORM_REFERENCE_CONVENTION,
        dictionary_name=settings.dictionary_name,
        marker_size_mm=float(settings.marker_size_mm),
        points=points_tuple,
    )
