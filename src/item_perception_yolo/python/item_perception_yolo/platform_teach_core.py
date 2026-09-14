import hashlib
import ipaddress
import json
import math
import os
import re
import tempfile
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

from camera_calibration_gui.calibration_core import (
    BASE_FRAME,
    CALIBRATION_ARTIFACT_SCHEMA_VERSION,
    CAMERA_ON_HAND,
    CAMERA_TO_HAND,
    CharucoSettings,
    REQUIRED_CHARUCO_LEGACY_PATTERN,
    REQUIRED_OPENCV_API,
    REQUIRED_OPENCV_VERSION,
    REQUIRED_OPENCV_WHEEL_SHA256,
    TOOL_FRAME,
    load_calibration_yaml,
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
    rotation_matrix_to_rpy_deg,
    workspace_root,
)

from .ui_state import (
    PACKAGE_UI_STATE_SCHEMA_VERSION,
    load_package_ui_state,
    write_platform_ui_state,
)


PLATFORM_FRAME = "platform_reference"
PLATFORM_ARTIFACT_SCHEMA_VERSION = 3
PLATFORM_REFERENCE_CONVENTION = "charuco_pick_corner_xy_v1"
PLATFORM_REFERENCE_DEFINITION = {
    "convention": PLATFORM_REFERENCE_CONVENTION,
    "origin": "charuco_board_origin_at_robot_pick_area_corner",
    "axes": "charuco_board_axes",
    "units": "metres",
    "roi_plane_z_m": 0.0,
    "placement": "same_bin_size_origin_axes_and_bin_offset_at_each_station",
}
PLATFORM_UI_STATE_SCHEMA_VERSION = PACKAGE_UI_STATE_SCHEMA_VERSION
TARGET_MAX_AGE_SEC = 0.5
ROBOT_TF_MAX_AGE_SEC = 1.0
MAX_LOG_EVENTS = 1000
UI_STATE_FILENAME = "last_session.json"
EVENT_LOG_FILENAME = "events.jsonl"
_ENV_KEY_PATTERN = re.compile(r"^[A-Z][A-Z0-9_]*$")


@dataclass(frozen=True)
class AppliedCameraCalibration:
    path: Path
    sha256: str
    created_at_utc: str
    calibration_mode: str
    reference_frame: str
    settings: CharucoSettings
    reference_from_camera_link: np.ndarray


@dataclass(frozen=True)
class PlatformTeachUiState:
    saved_at_utc: str
    camera_calibration_filename: str


@dataclass(frozen=True)
class PlatformCapture:
    captured_at_utc: str
    color_stamp_sec: int
    color_stamp_nanosec: int
    frame_sequence: int
    charuco_corner_count: int
    calibration_mode: str
    calibration_reference_from_camera_link: np.ndarray
    base_from_tool: np.ndarray | None
    robot_tf_stamp_sec: int | None
    robot_tf_stamp_nanosec: int | None
    base_from_camera_link: np.ndarray
    camera_link_from_optical: np.ndarray
    optical_from_board: np.ndarray
    base_from_platform: np.ndarray


@dataclass(frozen=True)
class PlatformCalibrationArtifact:
    path: Path
    sha256: str
    created_at_utc: str
    robot_lan1_ip: str
    platform_frame: str
    reference_convention: str
    camera_calibration_filename: str
    camera_calibration_sha256: str
    camera_calibration_mode: str
    camera_settings: CharucoSettings
    calibration_reference_from_camera_link: np.ndarray
    base_from_platform: np.ndarray


def _canonical_utc_text(timestamp: datetime) -> str:
    if timestamp.tzinfo is None:
        raise ValueError("Timestamp must include a timezone")
    return (
        timestamp.astimezone(timezone.utc)
        .isoformat(timespec="microseconds")
        .replace("+00:00", "Z")
    )


def _validate_nonzero_stamp(sec, nanosec, label: str) -> tuple[int, int]:
    if (
        type(sec) is not int
        or type(nanosec) is not int
        or sec < 0
        or not 0 <= nanosec < 1_000_000_000
        or (sec == 0 and nanosec == 0)
    ):
        raise ValueError(f"{label} is invalid")
    return sec, nanosec


def _validate_rigid_transform(value, label: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{label} must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12):
        raise ValueError(f"{label} has an invalid homogeneous row")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError(f"{label} rotation must be orthonormal")
    if abs(float(np.linalg.det(rotation)) - 1.0) > 1e-6:
        raise ValueError(f"{label} rotation determinant must be +1")
    return np.array(matrix, dtype=np.float64, order="C", copy=True)


def calibration_directory(root: Path | None = None) -> Path:
    project_root = workspace_root() if root is None else Path(root).resolve()
    return project_root / "calibration"


def package_log_directory(root: Path | None = None) -> Path:
    project_root = workspace_root() if root is None else Path(root).resolve()
    return project_root / "logs" / "item_perception_yolo"


def ui_state_path(root: Path | None = None) -> Path:
    return package_log_directory(root) / UI_STATE_FILENAME


def _parse_env_file(path: Path) -> dict[str, str]:
    if not path.is_file():
        raise ValueError(f"Required project configuration is not a file: {path}")
    values: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as exc:
        raise ValueError(f"Cannot read project configuration {path}: {exc}") from exc
    for line_number, raw_line in enumerate(lines, start=1):
        if not raw_line or raw_line.startswith("#"):
            continue
        if raw_line.strip() != raw_line or raw_line.count("=") != 1:
            raise ValueError(
                f"Invalid strict KEY=value syntax at {path}:{line_number}"
            )
        key, value = raw_line.split("=", 1)
        if _ENV_KEY_PATTERN.fullmatch(key) is None:
            raise ValueError(f"Invalid configuration key at {path}:{line_number}")
        if key in values:
            raise ValueError(f"Duplicate configuration key {key} in {path}")
        values[key] = value
    return values


def load_robot_lan1_ip(root: Path | None = None) -> str:
    project_root = workspace_root() if root is None else Path(root).resolve()
    example_values = _parse_env_file(project_root / ".env.example")
    values = _parse_env_file(project_root / ".env")
    if set(values) != set(example_values):
        missing = sorted(set(example_values) - set(values))
        unsupported = sorted(set(values) - set(example_values))
        raise ValueError(
            "Root .env keys must exactly match .env.example; "
            f"missing={missing}, unsupported={unsupported}"
        )
    if values.get("ROS_LOCALHOST_ONLY") != "1":
        raise ValueError("ROS_LOCALHOST_ONLY must be exactly 1")
    raw_address = values.get("DOBOT_ROBOT_LAN1_IP", "")
    try:
        address = ipaddress.ip_address(raw_address)
    except ValueError as exc:
        raise ValueError("DOBOT_ROBOT_LAN1_IP must be a valid IPv4 address") from exc
    if address.version != 4:
        raise ValueError("DOBOT_ROBOT_LAN1_IP must be an IPv4 address")
    return str(address)


def load_camera_calibration(
    path: Path,
    root: Path | None = None,
) -> AppliedCameraCalibration:
    candidate = Path(path).expanduser().resolve()
    required_directory = calibration_directory(root).resolve()
    if candidate.parent != required_directory:
        raise ValueError(
            "Camera calibration must be selected directly from the root calibration/ "
            f"directory: {required_directory}"
        )
    valid_prefixes = {
        CAMERA_TO_HAND: "camera_to_hand_calibration_",
        CAMERA_ON_HAND: "camera_on_hand_calibration_",
    }
    if candidate.suffix != ".yaml" or not any(
        candidate.name.startswith(prefix) for prefix in valid_prefixes.values()
    ):
        raise ValueError(
            "Camera calibration filename must match camera_to_hand_calibration_"
            "<timestamp>.yaml or camera_on_hand_calibration_<timestamp>.yaml"
        )
    artifact = load_calibration_yaml(candidate)
    if artifact.calibration_mode not in valid_prefixes:
        raise ValueError(
            "Platform teaching requires a schema-7 camera_to_hand or "
            f"camera_on_hand calibration; received {artifact.calibration_mode}"
        )
    required_prefix = valid_prefixes[artifact.calibration_mode]
    if not candidate.name.startswith(required_prefix):
        raise ValueError(
            "Camera calibration filename conflicts with its calibration mode: "
            f"mode={artifact.calibration_mode}, filename={candidate.name}"
        )
    reference_frame = (
        BASE_FRAME if artifact.calibration_mode == CAMERA_TO_HAND else TOOL_FRAME
    )
    digest = hashlib.sha256(candidate.read_bytes()).hexdigest()
    return AppliedCameraCalibration(
        path=candidate,
        sha256=digest,
        created_at_utc=artifact.created_at_utc,
        calibration_mode=artifact.calibration_mode,
        reference_frame=reference_frame,
        settings=artifact.settings,
        reference_from_camera_link=_validate_rigid_transform(
            artifact.reference_from_camera_link,
            f"{reference_frame} from camera_link calibration",
        ),
    )


def resolve_base_from_camera_link(
    camera_calibration: AppliedCameraCalibration,
    base_from_tool=None,
) -> np.ndarray:
    calibrated = _validate_rigid_transform(
        camera_calibration.reference_from_camera_link,
        f"{camera_calibration.reference_frame} from camera_link calibration",
    )
    if camera_calibration.calibration_mode == CAMERA_TO_HAND:
        if camera_calibration.reference_frame != BASE_FRAME:
            raise ValueError("Camera-to-hand calibration reference must be base_link")
        if base_from_tool is not None:
            raise ValueError("Camera-to-hand resolution must not use a robot TF")
        return calibrated
    if camera_calibration.calibration_mode == CAMERA_ON_HAND:
        if camera_calibration.reference_frame != TOOL_FRAME:
            raise ValueError(f"Camera-on-hand calibration reference must be {TOOL_FRAME}")
        if base_from_tool is None:
            raise ValueError(
                f"Camera-on-hand resolution requires live {BASE_FRAME} <- {TOOL_FRAME}"
            )
        return _validate_rigid_transform(
            _validate_rigid_transform(
                base_from_tool,
                f"{BASE_FRAME} from {TOOL_FRAME}",
            )
            @ calibrated,
            f"Resolved {BASE_FRAME} from camera_link",
        )
    raise ValueError(
        "Camera calibration mode must be exactly camera_to_hand or camera_on_hand"
    )


def compose_platform_transform(
    base_from_camera_link,
    camera_link_from_optical,
    optical_from_board,
) -> np.ndarray:
    base_from_camera_link = _validate_rigid_transform(
        base_from_camera_link,
        "base_link from camera_link",
    )
    camera_link_from_optical = _validate_rigid_transform(
        camera_link_from_optical,
        "camera_link from color optical frame",
    )
    optical_from_board = _validate_rigid_transform(
        optical_from_board,
        "color optical frame from ChArUco board",
    )
    return _validate_rigid_transform(
        base_from_camera_link @ camera_link_from_optical @ optical_from_board,
        "base_link from platform_reference",
    )


def transform_summary(matrix: np.ndarray) -> tuple[str, str]:
    transform = _validate_rigid_transform(matrix, "Platform transform")
    translation = transform[:3, 3]
    roll, pitch, yaw = rotation_matrix_to_rpy_deg(transform[:3, :3])
    return (
        "XYZ [m]: "
        f"{translation[0]:+.4f}, {translation[1]:+.4f}, {translation[2]:+.4f}",
        f"RPY [deg]: {roll:+.2f}, {pitch:+.2f}, {yaw:+.2f}",
    )


def _transform_payload(matrix: np.ndarray, label: str) -> dict:
    transform = _validate_rigid_transform(matrix, label)
    translation = transform[:3, 3]
    qx, qy, qz, qw = rotation_matrix_to_quaternion(transform[:3, :3])
    return {
        "translation_m": {
            "x": float(translation[0]),
            "y": float(translation[1]),
            "z": float(translation[2]),
        },
        "rotation_xyzw": {"x": qx, "y": qy, "z": qz, "w": qw},
    }


def _transform_from_payload(value, label: str) -> np.ndarray:
    if not isinstance(value, dict) or set(value) != {
        "translation_m",
        "rotation_xyzw",
    }:
        raise ValueError(f"{label} must contain exact transform fields")
    translation = value["translation_m"]
    rotation = value["rotation_xyzw"]
    if not isinstance(translation, dict) or set(translation) != {"x", "y", "z"}:
        raise ValueError(f"{label}.translation_m must contain x, y, z")
    if not isinstance(rotation, dict) or set(rotation) != {"x", "y", "z", "w"}:
        raise ValueError(f"{label}.rotation_xyzw must contain x, y, z, w")
    numbers = []
    for section, keys in ((translation, ("x", "y", "z")), (rotation, ("x", "y", "z", "w"))):
        for key in keys:
            value_number = section[key]
            if type(value_number) not in {int, float} or not math.isfinite(float(value_number)):
                raise ValueError(f"{label} contains a non-finite numeric field")
            numbers.append(float(value_number))
    quaternion = np.asarray(numbers[3:], dtype=np.float64)
    if abs(float(np.linalg.norm(quaternion)) - 1.0) > 1e-5:
        raise ValueError(f"{label} quaternion must have unit length")
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_to_rotation_matrix(*numbers[3:])
    matrix[:3, 3] = numbers[:3]
    return _validate_rigid_transform(matrix, label)


def platform_output_path(
    robot_lan1_ip: str,
    root: Path | None = None,
    created_at: datetime | None = None,
) -> Path:
    try:
        address = ipaddress.ip_address(robot_lan1_ip)
    except ValueError as exc:
        raise ValueError("Robot LAN1 identity must be a valid IPv4 address") from exc
    if address.version != 4:
        raise ValueError("Robot LAN1 identity must be an IPv4 address")
    timestamp = created_at or datetime.now(timezone.utc)
    if timestamp.tzinfo is None:
        raise ValueError("Platform output timestamp must include a timezone")
    stamp = timestamp.astimezone(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    return calibration_directory(root) / f"platform_calibration_{stamp}_{address}.yaml"


def write_platform_calibration(
    path: Path,
    camera_calibration: AppliedCameraCalibration,
    robot_lan1_ip: str,
    capture: PlatformCapture,
    root: Path | None = None,
) -> None:
    output_path = Path(path).resolve()
    required_directory = calibration_directory(root).resolve()
    if output_path.parent != required_directory:
        raise ValueError("Platform calibration output must be in root calibration/")
    if camera_calibration.path.parent != required_directory:
        raise ValueError("Source camera calibration must be in root calibration/")
    if output_path.exists():
        raise ValueError(f"Platform calibration output already exists: {output_path}")
    if hashlib.sha256(camera_calibration.path.read_bytes()).hexdigest() != (
        camera_calibration.sha256
    ):
        raise ValueError("Selected camera calibration changed after it was applied")
    if capture.charuco_corner_count < 4:
        raise ValueError("Platform capture requires at least four ChArUco corners")
    if capture.frame_sequence < 1:
        raise ValueError("Platform capture frame sequence must be positive")
    _validate_nonzero_stamp(
        capture.color_stamp_sec,
        capture.color_stamp_nanosec,
        "Platform capture color timestamp",
    )
    try:
        address = ipaddress.ip_address(robot_lan1_ip)
    except ValueError as exc:
        raise ValueError("Robot LAN1 identity must be a valid IPv4 address") from exc
    if address.version != 4:
        raise ValueError("Robot LAN1 identity must be an IPv4 address")
    captured_at = datetime.fromisoformat(
        _canonical_utc_value(
            capture.captured_at_utc,
            "Platform capture timestamp",
        )[:-1]
        + "+00:00"
    )
    expected_output = platform_output_path(
        str(address),
        root=root,
        created_at=captured_at,
    ).resolve()
    if output_path != expected_output:
        raise ValueError(
            f"Platform calibration output name must be exactly {expected_output.name}"
        )
    settings = camera_calibration.settings
    settings.validate()
    if capture.calibration_mode != camera_calibration.calibration_mode:
        raise ValueError("Captured camera mode conflicts with applied calibration")
    calibration_reference_from_camera_link = _validate_rigid_transform(
        capture.calibration_reference_from_camera_link,
        "Captured calibrated-reference from camera_link",
    )
    if not np.allclose(
        calibration_reference_from_camera_link,
        camera_calibration.reference_from_camera_link,
        atol=1e-12,
    ):
        raise ValueError("Captured mounting transform conflicts with calibration")
    if camera_calibration.calibration_mode == CAMERA_TO_HAND:
        if (
            capture.base_from_tool is not None
            or capture.robot_tf_stamp_sec is not None
            or capture.robot_tf_stamp_nanosec is not None
        ):
            raise ValueError("Camera-to-hand platform capture must not contain robot TF")
        expected_base_from_camera_link = resolve_base_from_camera_link(
            camera_calibration
        )
        robot_tf_payload = {"required": False}
    elif camera_calibration.calibration_mode == CAMERA_ON_HAND:
        if capture.base_from_tool is None:
            raise ValueError("Camera-on-hand platform capture requires robot TF")
        _validate_nonzero_stamp(
            capture.robot_tf_stamp_sec,
            capture.robot_tf_stamp_nanosec,
            "Platform capture robot TF timestamp",
        )
        base_from_tool = _validate_rigid_transform(
            capture.base_from_tool,
            f"Captured {BASE_FRAME} from {TOOL_FRAME}",
        )
        expected_base_from_camera_link = resolve_base_from_camera_link(
            camera_calibration,
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
                f"Captured {BASE_FRAME} from {TOOL_FRAME}",
            ),
        }
    else:
        raise ValueError("Applied camera calibration mode is invalid")
    base_from_camera_link = _validate_rigid_transform(
        capture.base_from_camera_link,
        "Captured resolved base_link from camera_link",
    )
    if not np.allclose(
        base_from_camera_link,
        expected_base_from_camera_link,
        atol=1e-12,
    ):
        raise ValueError("Captured resolved camera transform conflicts with its chain")
    recomposed = compose_platform_transform(
        base_from_camera_link,
        capture.camera_link_from_optical,
        capture.optical_from_board,
    )
    if not np.allclose(recomposed, capture.base_from_platform, atol=1e-9):
        raise ValueError("Captured platform transform conflicts with its transform chain")
    payload = {
        "schema_version": PLATFORM_ARTIFACT_SCHEMA_VERSION,
        "artifact_type": "platform_calibration",
        "created_utc": capture.captured_at_utc,
        "robot": {
            "lan1_ip": str(address),
            "base_frame": BASE_FRAME,
        },
        "platform": {
            "frame": PLATFORM_FRAME,
            "reference": dict(PLATFORM_REFERENCE_DEFINITION),
        },
        "camera_calibration": {
            "filename": camera_calibration.path.name,
            "sha256": camera_calibration.sha256,
            "schema_version": CALIBRATION_ARTIFACT_SCHEMA_VERSION,
            "calibration_mode": camera_calibration.calibration_mode,
            "reference_frame": camera_calibration.reference_frame,
            "created_utc": camera_calibration.created_at_utc,
        },
        "camera": {
            "prefix": settings.camera_prefix,
            "link_frame": settings.camera_link_frame,
            "optical_frame": settings.optical_frame,
            "color_topic": settings.color_topic,
            "camera_info_topic": settings.camera_info_topic,
        },
        "charuco": {
            "dictionary": settings.dictionary_name,
            "squares_x": settings.squares_x,
            "squares_y": settings.squares_y,
            "square_length_mm": settings.square_length_mm,
            "marker_length_mm": settings.marker_length_mm,
            "legacy_pattern": REQUIRED_CHARUCO_LEGACY_PATTERN,
        },
        "detector": {
            "opencv_version": REQUIRED_OPENCV_VERSION,
            "opencv_wheel_sha256": REQUIRED_OPENCV_WHEEL_SHA256,
            "api": REQUIRED_OPENCV_API,
        },
        "capture": {
            "frame_sequence": capture.frame_sequence,
            "color_stamp": {
                "sec": capture.color_stamp_sec,
                "nanosec": capture.color_stamp_nanosec,
            },
            "charuco_corner_count": capture.charuco_corner_count,
            "calibration_reference_from_camera_link": _transform_payload(
                capture.calibration_reference_from_camera_link,
                "Captured calibrated-reference from camera_link",
            ),
            "robot_tf": robot_tf_payload,
            "base_from_camera_link": _transform_payload(
                capture.base_from_camera_link,
                "Captured resolved base_link from camera_link",
            ),
            "camera_link_from_optical": _transform_payload(
                capture.camera_link_from_optical,
                "Captured camera_link from optical transform",
            ),
            "optical_from_board": _transform_payload(
                capture.optical_from_board,
                "Captured optical from board transform",
            ),
        },
        "transform": {
            "target_frame": BASE_FRAME,
            "source_frame": PLATFORM_FRAME,
            "convention": "target_from_source",
            **_transform_payload(
                capture.base_from_platform,
                "Captured base_link from platform_reference transform",
            ),
        },
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


def _canonical_utc_value(value, label: str) -> str:
    if type(value) is not str or not value.endswith("Z"):
        raise ValueError(f"{label} must be canonical UTC")
    try:
        parsed = datetime.fromisoformat(value[:-1] + "+00:00")
    except ValueError as exc:
        raise ValueError(f"{label} must be canonical UTC") from exc
    if value != _canonical_utc_text(parsed):
        raise ValueError(f"{label} must be canonical UTC")
    return value


def load_platform_calibration(
    path: Path,
    root: Path | None = None,
) -> PlatformCalibrationArtifact:
    candidate = Path(path).expanduser().resolve()
    if candidate.parent != calibration_directory(root).resolve():
        raise ValueError("Platform calibration must be in root calibration/")
    if not candidate.is_file():
        raise ValueError(f"Platform calibration is not a file: {candidate}")
    try:
        content = candidate.read_bytes()
        payload = yaml.safe_load(content.decode("utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f"Cannot read platform calibration {candidate}: {exc}") from exc
    required = {
        "schema_version",
        "artifact_type",
        "created_utc",
        "robot",
        "platform",
        "camera_calibration",
        "camera",
        "charuco",
        "detector",
        "capture",
        "transform",
    }
    if not isinstance(payload, dict) or set(payload) != required:
        raise ValueError("Platform calibration has unsupported or missing root fields")
    if (
        type(payload["schema_version"]) is not int
        or payload["schema_version"] != PLATFORM_ARTIFACT_SCHEMA_VERSION
    ):
        raise ValueError(
            "Platform calibration schema_version must be exactly "
            f"{PLATFORM_ARTIFACT_SCHEMA_VERSION}"
        )
    if payload["artifact_type"] != "platform_calibration":
        raise ValueError("Platform calibration artifact_type is invalid")
    created_at_utc = _canonical_utc_value(
        payload["created_utc"],
        "Platform calibration created_utc",
    )
    robot = payload["robot"]
    platform = payload["platform"]
    camera_calibration = payload["camera_calibration"]
    transform = payload["transform"]
    if not isinstance(robot, dict) or set(robot) != {"lan1_ip", "base_frame"}:
        raise ValueError("Platform calibration robot fields are invalid")
    if robot["base_frame"] != BASE_FRAME:
        raise ValueError(f"Platform calibration base frame must be {BASE_FRAME}")
    try:
        address = ipaddress.ip_address(robot["lan1_ip"])
    except (TypeError, ValueError) as exc:
        raise ValueError("Platform calibration robot LAN1 IP is invalid") from exc
    if address.version != 4:
        raise ValueError("Platform calibration robot LAN1 IP must be IPv4")
    if not isinstance(platform, dict) or set(platform) != {"frame", "reference"}:
        raise ValueError("Platform calibration platform fields are invalid")
    if platform["frame"] != PLATFORM_FRAME:
        raise ValueError(f"Platform frame must be exactly {PLATFORM_FRAME}")
    if platform["reference"] != PLATFORM_REFERENCE_DEFINITION:
        raise ValueError("Platform calibration reference convention is invalid")
    if not isinstance(camera_calibration, dict) or (
        set(camera_calibration)
        != {
            "filename",
            "sha256",
            "schema_version",
            "calibration_mode",
            "reference_frame",
            "created_utc",
        }
    ):
        raise ValueError("Platform calibration camera-calibration fields are invalid")
    if camera_calibration["schema_version"] != CALIBRATION_ARTIFACT_SCHEMA_VERSION:
        raise ValueError("Platform calibration camera schema is not 7")
    calibration_mode = camera_calibration["calibration_mode"]
    expected_prefixes = {
        CAMERA_TO_HAND: "camera_to_hand_calibration_",
        CAMERA_ON_HAND: "camera_on_hand_calibration_",
    }
    if calibration_mode not in expected_prefixes:
        raise ValueError("Platform calibration camera mode is invalid")
    expected_reference_frame = (
        BASE_FRAME if calibration_mode == CAMERA_TO_HAND else TOOL_FRAME
    )
    if camera_calibration["reference_frame"] != expected_reference_frame:
        raise ValueError("Platform calibration camera reference frame is invalid")
    filename = camera_calibration["filename"]
    if (
        type(filename) is not str
        or Path(filename).name != filename
        or not filename.startswith(expected_prefixes[calibration_mode])
        or Path(filename).suffix != ".yaml"
    ):
        raise ValueError("Platform calibration camera filename is invalid")
    digest = camera_calibration["sha256"]
    if type(digest) is not str or re.fullmatch(r"[0-9a-f]{64}", digest) is None:
        raise ValueError("Platform calibration camera SHA-256 is invalid")
    _canonical_utc_value(
        camera_calibration["created_utc"],
        "Platform calibration source-camera created_utc",
    )

    camera = payload["camera"]
    if not isinstance(camera, dict) or set(camera) != {
        "prefix",
        "link_frame",
        "optical_frame",
        "color_topic",
        "camera_info_topic",
    }:
        raise ValueError("Platform calibration camera fields are invalid")
    charuco = payload["charuco"]
    if not isinstance(charuco, dict) or set(charuco) != {
        "dictionary",
        "squares_x",
        "squares_y",
        "square_length_mm",
        "marker_length_mm",
        "legacy_pattern",
    }:
        raise ValueError("Platform calibration ChArUco fields are invalid")
    if type(camera["prefix"]) is not str or type(charuco["dictionary"]) is not str:
        raise ValueError("Platform calibration camera/ChArUco names must be strings")
    for key in ("squares_x", "squares_y"):
        if type(charuco[key]) is not int:
            raise ValueError(f"Platform calibration charuco.{key} must be an integer")
    for key in ("square_length_mm", "marker_length_mm"):
        if type(charuco[key]) not in {int, float} or not math.isfinite(
            float(charuco[key])
        ):
            raise ValueError(f"Platform calibration charuco.{key} must be finite")
    settings = CharucoSettings(
        camera_prefix=camera["prefix"],
        dictionary_name=charuco["dictionary"],
        squares_x=charuco["squares_x"],
        squares_y=charuco["squares_y"],
        square_length_mm=charuco["square_length_mm"],
        marker_length_mm=charuco["marker_length_mm"],
    )
    settings.validate()
    expected_camera = {
        "link_frame": settings.camera_link_frame,
        "optical_frame": settings.optical_frame,
        "color_topic": settings.color_topic,
        "camera_info_topic": settings.camera_info_topic,
    }
    for key, expected in expected_camera.items():
        if camera[key] != expected:
            raise ValueError(
                f"Platform calibration camera.{key} conflicts with prefix"
            )
    if charuco["legacy_pattern"] is not REQUIRED_CHARUCO_LEGACY_PATTERN:
        raise ValueError("Platform calibration requires the legacy ChArUco pattern")

    detector = payload["detector"]
    expected_detector = {
        "opencv_version": REQUIRED_OPENCV_VERSION,
        "opencv_wheel_sha256": REQUIRED_OPENCV_WHEEL_SHA256,
        "api": REQUIRED_OPENCV_API,
    }
    if not isinstance(detector, dict) or set(detector) != set(expected_detector):
        raise ValueError("Platform calibration detector fields are invalid")
    for key, expected in expected_detector.items():
        if detector[key] != expected:
            raise ValueError(
                f"Platform calibration detector.{key} is not canonical"
            )

    capture = payload["capture"]
    if not isinstance(capture, dict) or set(capture) != {
        "frame_sequence",
        "color_stamp",
        "charuco_corner_count",
        "calibration_reference_from_camera_link",
        "robot_tf",
        "base_from_camera_link",
        "camera_link_from_optical",
        "optical_from_board",
    }:
        raise ValueError("Platform calibration capture fields are invalid")
    if type(capture["frame_sequence"]) is not int or capture["frame_sequence"] < 1:
        raise ValueError("Platform capture frame_sequence must be positive")
    if (
        type(capture["charuco_corner_count"]) is not int
        or capture["charuco_corner_count"] < 4
    ):
        raise ValueError("Platform capture requires at least four ChArUco corners")
    stamp = capture["color_stamp"]
    if not isinstance(stamp, dict) or set(stamp) != {"sec", "nanosec"}:
        raise ValueError("Platform capture color_stamp fields are invalid")
    _validate_nonzero_stamp(
        stamp["sec"],
        stamp["nanosec"],
        "Platform capture color_stamp",
    )
    calibration_reference_from_camera_link = _transform_from_payload(
        capture["calibration_reference_from_camera_link"],
        "Platform capture calibrated-reference from camera_link",
    )
    robot_tf = capture["robot_tf"]
    if calibration_mode == CAMERA_TO_HAND:
        if robot_tf != {"required": False}:
            raise ValueError(
                "Camera-to-hand platform capture robot_tf must be exactly unused"
            )
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
            raise ValueError("Camera-on-hand platform capture robot_tf fields are invalid")
        if robot_tf["required"] is not True:
            raise ValueError("Camera-on-hand platform capture robot_tf is required")
        if (
            robot_tf["target_frame"] != BASE_FRAME
            or robot_tf["source_frame"] != TOOL_FRAME
            or robot_tf["maximum_age_sec"] != ROBOT_TF_MAX_AGE_SEC
        ):
            raise ValueError("Camera-on-hand platform robot TF contract is invalid")
        robot_stamp = robot_tf["stamp"]
        if not isinstance(robot_stamp, dict) or set(robot_stamp) != {"sec", "nanosec"}:
            raise ValueError("Camera-on-hand platform robot TF stamp fields are invalid")
        _validate_nonzero_stamp(
            robot_stamp["sec"],
            robot_stamp["nanosec"],
            "Camera-on-hand platform robot TF stamp",
        )
        base_from_tool = _transform_from_payload(
            robot_tf["base_from_tool"],
            f"Platform capture {BASE_FRAME} from {TOOL_FRAME}",
        )
        expected_base_from_camera_link = _validate_rigid_transform(
            base_from_tool @ calibration_reference_from_camera_link,
            "Platform capture resolved base_link from camera_link",
        )
    base_from_camera_link = _transform_from_payload(
        capture["base_from_camera_link"],
        "Platform capture base_from_camera_link",
    )
    if not np.allclose(
        base_from_camera_link,
        expected_base_from_camera_link,
        atol=1e-9,
    ):
        raise ValueError("Platform resolved camera transform conflicts with its chain")
    camera_link_from_optical = _transform_from_payload(
        capture["camera_link_from_optical"],
        "Platform capture camera_link_from_optical",
    )
    optical_from_board = _transform_from_payload(
        capture["optical_from_board"],
        "Platform capture optical_from_board",
    )

    if not isinstance(transform, dict) or set(transform) != {
        "target_frame",
        "source_frame",
        "convention",
        "translation_m",
        "rotation_xyzw",
    }:
        raise ValueError("Platform calibration transform fields are invalid")
    if transform["target_frame"] != BASE_FRAME:
        raise ValueError(f"Platform transform target must be {BASE_FRAME}")
    if transform["source_frame"] != PLATFORM_FRAME:
        raise ValueError(f"Platform transform source must be {PLATFORM_FRAME}")
    if transform["convention"] != "target_from_source":
        raise ValueError("Platform transform convention must be target_from_source")
    base_from_platform = _transform_from_payload(
        {
            "translation_m": transform["translation_m"],
            "rotation_xyzw": transform["rotation_xyzw"],
        },
        "Platform transform",
    )
    recomposed = compose_platform_transform(
        base_from_camera_link,
        camera_link_from_optical,
        optical_from_board,
    )
    if not np.allclose(base_from_platform, recomposed, atol=1e-9):
        raise ValueError(
            "Platform transform conflicts with its captured transform chain"
        )
    parsed_created_at = datetime.fromisoformat(created_at_utc[:-1] + "+00:00")
    expected_path = platform_output_path(
        str(address),
        root=root,
        created_at=parsed_created_at,
    ).resolve()
    if candidate != expected_path:
        raise ValueError(
            f"Platform calibration filename must be exactly {expected_path.name}"
        )
    return PlatformCalibrationArtifact(
        path=candidate,
        sha256=hashlib.sha256(content).hexdigest(),
        created_at_utc=created_at_utc,
        robot_lan1_ip=str(address),
        platform_frame=PLATFORM_FRAME,
        reference_convention=PLATFORM_REFERENCE_CONVENTION,
        camera_calibration_filename=filename,
        camera_calibration_sha256=digest,
        camera_calibration_mode=calibration_mode,
        camera_settings=settings,
        calibration_reference_from_camera_link=(
            calibration_reference_from_camera_link
        ),
        base_from_platform=base_from_platform,
    )


def load_ui_state(path: Path) -> PlatformTeachUiState | None:
    state = load_package_ui_state(path)
    if state is None or state.platform_camera_calibration_filename is None:
        return None
    return PlatformTeachUiState(
        state.saved_at_utc,
        state.platform_camera_calibration_filename,
    )


def write_ui_state(
    path: Path,
    camera_calibration_filename: str,
    saved_at: datetime | None = None,
) -> PlatformTeachUiState:
    package_state = write_platform_ui_state(
        path,
        camera_calibration_filename,
        saved_at=saved_at,
    )
    state = PlatformTeachUiState(
        package_state.saved_at_utc,
        package_state.platform_camera_calibration_filename,
    )
    return state


class PackageEventLogger:
    def __init__(self, root: Path | None = None, node_name: str = "platform_teach") -> None:
        self.path = package_log_directory(root) / EVENT_LOG_FILENAME
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._node_name = node_name

    def record(self, level: str, event: str, message: str, **fields) -> None:
        record = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat().replace(
                "+00:00",
                "Z",
            ),
            "package": "item_perception_yolo",
            "node": self._node_name,
            "level": str(level),
            "event": str(event),
            "message": str(message),
            **fields,
        }
        with self._lock:
            count = 0
            if self.path.is_file():
                with self.path.open("r", encoding="utf-8") as stream:
                    count = sum(1 for line in stream if line.strip())
            mode = "w" if count >= MAX_LOG_EVENTS else "a"
            with self.path.open(mode, encoding="utf-8") as stream:
                stream.write(
                    json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n"
                )
