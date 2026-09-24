import json
import math
import os
import re
import tempfile
import threading
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml


MAX_LOG_EVENTS = 1000
REQUIRED_OPENCV_VERSION = "4.10.0"
REQUIRED_OPENCV_THREAD_COUNT = 1
REQUIRED_OPENCV_WHEEL_SHA256 = (
    "9ace140fc6d647fbe1c692bcb2abce768973491222c067c131d80957c595b71f"
)
REQUIRED_OPENCV_API = "CharucoDetector.detectBoard+matchImagePoints+solvePnP"
REQUIRED_CHARUCO_LEGACY_PATTERN = True
REQUIRED_HAND_EYE_METHOD = "CALIB_HAND_EYE_TSAI"
MINIMUM_CALIBRATION_SAMPLES = 5
MINIMUM_CHARUCO_CORNERS = 4
MINIMUM_SAMPLE_ROTATION_DEG = 5.0
MOVEIT_REFERENCE_URL = "https://github.com/moveit/moveit_calibration"
MOVEIT_REFERENCE_COMMIT = "3f9d48ebe843caf1de060bfafe78160585c7c26f"
LEAVE_ONE_OUT_MINIMUM_SAMPLES = MINIMUM_CALIBRATION_SAMPLES + 1
CALIBRATION_ARTIFACT_SCHEMA_VERSION = 7
UI_STATE_SCHEMA_VERSION = 4
UI_STATE_FILENAME = "last_session.json"
CAMERA_PREFIX_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
CAMERA_TO_HAND = "camera_to_hand"
CAMERA_ON_HAND = "camera_on_hand"
CALIBRATION_MODES = (CAMERA_TO_HAND, CAMERA_ON_HAND)
BASE_FRAME = "base_link"
TOOL_FRAME = "Link6"
JOINT_STATE_TOPIC = "/joint_states"
JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
CHARUCO_DICTIONARIES = {
    name: name
    for name in (
        "DICT_4X4_50",
        "DICT_4X4_100",
        "DICT_4X4_250",
        "DICT_4X4_1000",
        "DICT_5X5_50",
        "DICT_5X5_100",
        "DICT_5X5_250",
        "DICT_5X5_1000",
        "DICT_6X6_50",
        "DICT_6X6_100",
        "DICT_6X6_250",
        "DICT_6X6_1000",
        "DICT_7X7_50",
        "DICT_7X7_100",
        "DICT_7X7_250",
        "DICT_7X7_1000",
        "DICT_ARUCO_ORIGINAL",
    )
}


class InvalidPoseError(ValueError):
    """Signal a malformed native pose that makes the worker unsafe to continue."""


def _worker_opencv_module():
    """Return the already-loaded worker OpenCV module without importing it here."""
    module = sys.modules.get("cv2")
    if module is None or getattr(module, "__version__", None) != REQUIRED_OPENCV_VERSION:
        raise RuntimeError(
            "OpenCV calibration math may run only inside the verified OpenCV worker"
        )
    return module


def workspace_root() -> Path:
    candidates = [Path.cwd(), Path(__file__).resolve()]
    for variable in ("COLCON_PREFIX_PATH", "AMENT_PREFIX_PATH"):
        for token in os.environ.get(variable, "").split(os.pathsep):
            if token:
                candidates.append(Path(token))

    for start in candidates:
        path = start.expanduser().resolve()
        if path.is_file():
            path = path.parent
        for candidate in (path, *path.parents):
            if (candidate / "src").is_dir() and (candidate / "README.md").is_file():
                return candidate
    raise RuntimeError("Could not resolve PicknPlace workspace root")


@dataclass(frozen=True)
class CharucoSettings:
    camera_prefix: str
    dictionary_name: str
    squares_x: int
    squares_y: int
    square_length_mm: float
    marker_length_mm: float

    def validate(self) -> None:
        if CAMERA_PREFIX_PATTERN.fullmatch(self.camera_prefix) is None:
            raise ValueError(
                "Camera prefix is required and must start with a letter and contain only "
                "letters, numbers, and _."
            )
        if self.dictionary_name not in CHARUCO_DICTIONARIES:
            raise ValueError(f"Unsupported ChArUco dictionary: {self.dictionary_name}")
        if self.squares_x < 3 or self.squares_y < 3:
            raise ValueError("ChArUco squares X and Y must each be at least 3")
        if not math.isfinite(self.square_length_mm) or self.square_length_mm <= 0.0:
            raise ValueError("Checker square size must be greater than zero millimetres")
        if not math.isfinite(self.marker_length_mm) or self.marker_length_mm <= 0.0:
            raise ValueError("ArUco marker size must be greater than zero millimetres")
        if self.marker_length_mm >= self.square_length_mm:
            raise ValueError("ArUco marker size must be smaller than checker square size")

    @property
    def color_topic(self) -> str:
        return f"/{self.camera_prefix}/color/image_raw"

    @property
    def camera_info_topic(self) -> str:
        return f"/{self.camera_prefix}/color/camera_info"

    @property
    def optical_frame(self) -> str:
        return f"{self.camera_prefix}_color_optical_frame"

    @property
    def camera_link_frame(self) -> str:
        return f"{self.camera_prefix}_link"


@dataclass(frozen=True)
class CalibrationSample:
    sample_id: str
    base_from_tool: np.ndarray
    camera_from_target: np.ndarray
    joint_positions_rad: tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class SampleResidual:
    sample_id: str
    translation_mm: float
    rotation_deg: float


@dataclass(frozen=True)
class CalibrationQuality:
    sample_count: int
    translation_rms_mm: float
    rotation_rms_deg: float
    max_translation_residual_mm: float
    max_translation_sample_id: str
    max_rotation_residual_deg: float
    max_rotation_sample_id: str
    residuals: tuple[SampleResidual, ...]
    constant_transform: np.ndarray


@dataclass(frozen=True)
class SolutionChangeDiagnostics:
    available: bool
    translation_rms_delta_mm: float | None
    rotation_rms_delta_deg: float | None
    camera_translation_delta_mm: float | None
    camera_rotation_delta_deg: float | None


@dataclass(frozen=True)
class LeaveOneOutEntry:
    sample_id: str
    camera_translation_delta_mm: float
    camera_rotation_delta_deg: float


@dataclass(frozen=True)
class LeaveOneOutDiagnostics:
    status: str
    entries: tuple[LeaveOneOutEntry, ...]
    failed_sample_ids: tuple[str, ...]
    translation_rms_mm: float | None
    rotation_rms_deg: float | None
    max_translation_delta_mm: float | None
    max_translation_sample_id: str | None
    max_rotation_delta_deg: float | None
    max_rotation_sample_id: str | None


@dataclass(frozen=True)
class PoseCoverageDiagnostics:
    translation_span_mm: float
    rotation_span_deg: float


@dataclass(frozen=True)
class AxXbDiagnostics:
    pair_count: int
    translation_rms_mm: float
    rotation_rms_deg: float


@dataclass(frozen=True)
class AccuracyDiagnostics:
    fit: CalibrationQuality
    solution_change: SolutionChangeDiagnostics
    leave_one_out: LeaveOneOutDiagnostics
    pose_coverage: PoseCoverageDiagnostics
    ax_xb: AxXbDiagnostics

    @property
    def save_allowed(self) -> bool:
        return self.leave_one_out.status != "failed"


@dataclass(frozen=True)
class SampleDiagnosticRow:
    sample_id: str
    translation_residual_mm: float | None
    rotation_residual_deg: float | None
    leave_one_out_translation_mm: float | None
    leave_one_out_rotation_deg: float | None
    flags: tuple[str, ...]


@dataclass(frozen=True)
class CalibrationUiState:
    saved_at_utc: str
    calibration_mode: str
    settings: CharucoSettings
    minimum_samples: int


@dataclass(frozen=True)
class CalibrationArtifact:
    created_at_utc: str
    calibration_mode: str
    settings: CharucoSettings
    reference_from_camera_link: np.ndarray
    samples: tuple[CalibrationSample, ...]


def calibration_ui_state_path(root: Path | None = None) -> Path:
    project_root = root or workspace_root()
    return project_root / "logs" / "camera_calibration" / UI_STATE_FILENAME


def _canonical_utc_text(timestamp: datetime) -> str:
    if timestamp.tzinfo is None:
        raise ValueError("UI-state timestamp must include a timezone")
    return (
        timestamp.astimezone(timezone.utc)
        .isoformat(timespec="microseconds")
        .replace("+00:00", "Z")
    )


def _ui_state_integer(payload: dict, key: str) -> int:
    value = payload[key]
    if type(value) is not int:
        raise ValueError(f"Camera-calibration UI state {key} must be an integer")
    return value


def _ui_state_number(payload: dict, key: str) -> float:
    value = payload[key]
    if type(value) not in {int, float} or not math.isfinite(float(value)):
        raise ValueError(f"Camera-calibration UI state {key} must be a finite number")
    return float(value)


def _ui_state_from_payload(payload: object) -> CalibrationUiState:
    required_keys = {
        "schema_version",
        "saved_at_utc",
        "calibration_mode",
        "camera_prefix",
        "dictionary_name",
        "squares_x",
        "squares_y",
        "square_length_mm",
        "marker_length_mm",
        "minimum_samples",
    }
    if not isinstance(payload, dict) or set(payload) != required_keys:
        actual_keys = sorted(payload) if isinstance(payload, dict) else []
        raise ValueError(
            "Camera-calibration UI state must contain exactly the canonical keys; "
            f"actual={actual_keys}"
        )
    if type(payload["schema_version"]) is not int or (
        payload["schema_version"] != UI_STATE_SCHEMA_VERSION
    ):
        raise ValueError(
            f"Camera-calibration UI state schema_version must be exactly "
            f"{UI_STATE_SCHEMA_VERSION}"
        )
    for key in (
        "saved_at_utc",
        "calibration_mode",
        "camera_prefix",
        "dictionary_name",
    ):
        if type(payload[key]) is not str:
            raise ValueError(f"Camera-calibration UI state {key} must be a string")
    saved_at_text = payload["saved_at_utc"]
    if not saved_at_text.endswith("Z"):
        raise ValueError("Camera-calibration UI state saved_at_utc must end with Z")
    try:
        saved_at = datetime.fromisoformat(saved_at_text[:-1] + "+00:00")
    except ValueError as exc:
        raise ValueError(
            "Camera-calibration UI state saved_at_utc is invalid"
        ) from exc
    if saved_at_text != _canonical_utc_text(saved_at):
        raise ValueError(
            "Camera-calibration UI state saved_at_utc is not canonical UTC"
        )

    calibration_mode = validate_calibration_mode(payload["calibration_mode"])
    settings = CharucoSettings(
        camera_prefix=payload["camera_prefix"],
        dictionary_name=payload["dictionary_name"],
        squares_x=_ui_state_integer(payload, "squares_x"),
        squares_y=_ui_state_integer(payload, "squares_y"),
        square_length_mm=_ui_state_number(payload, "square_length_mm"),
        marker_length_mm=_ui_state_number(payload, "marker_length_mm"),
    )
    settings.validate()
    minimum_samples = _ui_state_integer(payload, "minimum_samples")
    if minimum_samples != MINIMUM_CALIBRATION_SAMPLES:
        raise ValueError(
            "Camera-calibration UI state minimum_samples must be exactly "
            f"{MINIMUM_CALIBRATION_SAMPLES}"
        )
    return CalibrationUiState(
        saved_at_utc=saved_at_text,
        calibration_mode=calibration_mode,
        settings=settings,
        minimum_samples=minimum_samples,
    )


def load_calibration_ui_state(path: Path) -> CalibrationUiState | None:
    if not path.exists():
        return None
    if not path.is_file():
        raise ValueError(f"Camera-calibration UI state is not a file: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"Cannot read camera-calibration UI state {path}: {exc}") from exc
    return _ui_state_from_payload(payload)


def write_calibration_ui_state(
    path: Path,
    calibration_mode: str,
    settings: CharucoSettings,
    saved_at: datetime | None = None,
) -> CalibrationUiState:
    timestamp = saved_at or datetime.now(timezone.utc)
    payload = {
        "schema_version": UI_STATE_SCHEMA_VERSION,
        "saved_at_utc": _canonical_utc_text(timestamp),
        "calibration_mode": calibration_mode,
        "camera_prefix": settings.camera_prefix,
        "dictionary_name": settings.dictionary_name,
        "squares_x": settings.squares_x,
        "squares_y": settings.squares_y,
        "square_length_mm": settings.square_length_mm,
        "marker_length_mm": settings.marker_length_mm,
        "minimum_samples": MINIMUM_CALIBRATION_SAMPLES,
    }
    state = _ui_state_from_payload(payload)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            delete=False,
        ) as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
            temporary_path = Path(stream.name)
        os.replace(temporary_path, path)
        temporary_path = None
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
    return state


class PackageEventLogger:
    def __init__(self, root: Path | None = None) -> None:
        project_root = root or workspace_root()
        self.path = project_root / "logs" / "camera_calibration" / "events.jsonl"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def record(self, level: str, event: str, message: str, **fields) -> None:
        record = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "package": "camera_calibration",
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
                stream.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")


def validate_calibration_mode(calibration_mode: str) -> str:
    if calibration_mode not in CALIBRATION_MODES:
        raise ValueError(
            "Calibration mode must be exactly camera_to_hand or camera_on_hand"
        )
    return calibration_mode


def reference_frame_for_mode(calibration_mode: str) -> str:
    validate_calibration_mode(calibration_mode)
    return BASE_FRAME if calibration_mode == CAMERA_TO_HAND else TOOL_FRAME


def invert_transform(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError("Transform must be a 4x4 matrix")
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = matrix[:3, :3].T
    inverse[:3, 3] = -(inverse[:3, :3] @ matrix[:3, 3])
    return inverse


def rotation_angle_deg(rotation: np.ndarray) -> float:
    cosine = (float(np.trace(rotation)) - 1.0) * 0.5
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0, 1] + matrix[1, 0]) / scale
        qz = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / scale
        qx = (matrix[0, 1] + matrix[1, 0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / scale
        qx = (matrix[0, 2] + matrix[2, 0]) / scale
        qy = (matrix[1, 2] + matrix[2, 1]) / scale
        qz = 0.25 * scale
    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


def quaternion_to_rotation_matrix(
    x: float,
    y: float,
    z: float,
    w: float,
) -> np.ndarray:
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    if not np.all(np.isfinite(quaternion)):
        raise ValueError("Quaternion must contain four finite values")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError("Quaternion has zero length")
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


def rotation_matrix_to_rpy_deg(rotation: np.ndarray) -> tuple[float, float, float]:
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("Rotation must be a finite 3x3 matrix")
    pitch = math.asin(max(-1.0, min(1.0, -float(matrix[2, 0]))))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
        yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    else:
        roll = math.atan2(-float(matrix[1, 2]), float(matrix[1, 1]))
        yaw = 0.0
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def _validate_transform(matrix: np.ndarray, label: str) -> np.ndarray:
    candidate = np.asarray(matrix, dtype=np.float64)
    if candidate.shape != (4, 4) or not np.all(np.isfinite(candidate)):
        raise ValueError(f"{label} must be a finite 4x4 matrix")
    return candidate


def sample_rotation_conflict(
    base_from_tool: np.ndarray,
    camera_from_target: np.ndarray,
    previous: list[CalibrationSample],
) -> tuple[str, str, float] | None:
    """Match MoveIt's all-prior robot AND target orientation acceptance rule."""
    for label, current, attribute in (
        ("Robot", base_from_tool, "base_from_tool"),
        ("Board", camera_from_target, "camera_from_target"),
    ):
        for prior in previous:
            angle = rotation_angle_deg(
                current[:3, :3].T @ getattr(prior, attribute)[:3, :3]
            )
            # Acos roundoff must not reject the inclusive five-degree boundary.
            if angle < MINIMUM_SAMPLE_ROTATION_DEG and not math.isclose(
                angle, MINIMUM_SAMPLE_ROTATION_DEG, rel_tol=0.0, abs_tol=1e-10
            ):
                return prior.sample_id, label, angle
    return None


def _validate_samples(samples: list[CalibrationSample]) -> None:
    if len(samples) < MINIMUM_CALIBRATION_SAMPLES:
        raise ValueError(
            f"At least {MINIMUM_CALIBRATION_SAMPLES} calibration samples are required, "
            f"received {len(samples)}"
        )
    seen_ids = set()
    for index, sample in enumerate(samples, start=1):
        if not isinstance(sample, CalibrationSample):
            raise ValueError(f"Calibration sample {index} must be a CalibrationSample")
        if re.fullmatch(r"C[1-9][0-9]*", sample.sample_id) is None:
            raise ValueError(f"Calibration sample {index} has invalid ID {sample.sample_id!r}")
        if sample.sample_id in seen_ids:
            raise ValueError(f"Calibration sample ID is duplicated: {sample.sample_id}")
        seen_ids.add(sample.sample_id)
        _validate_rigid_transform(
            sample.base_from_tool, f"Calibration sample {index} base_from_tool"
        )
        _validate_rigid_transform(
            sample.camera_from_target,
            f"Calibration sample {index} camera_from_target",
        )
        if (
            not isinstance(sample.joint_positions_rad, tuple)
            or len(sample.joint_positions_rad) != len(JOINT_NAMES)
            or any(
                type(position) is not float or not math.isfinite(position)
                for position in sample.joint_positions_rad
            )
        ):
            raise ValueError(
                f"Calibration sample {index} joint_positions_rad must contain "
                f"exactly {len(JOINT_NAMES)} finite floats"
            )
        conflict = sample_rotation_conflict(
            sample.base_from_tool, sample.camera_from_target, samples[:index - 1]
        )
        if conflict is not None:
            prior_id, label, angle = conflict
            raise ValueError(
                f"Sample {sample.sample_id}: {label} orientation is too similar to "
                f"{prior_id}: {angle:.3f}deg; require at least "
                f"{MINIMUM_SAMPLE_ROTATION_DEG:.0f}deg"
            )


def ax_xb_diagnostics(
    calibration_mode: str,
    samples: list[CalibrationSample],
    reference_from_optical: np.ndarray,
) -> AxXbDiagnostics:
    """Adjacent-pair MoveIt residuals, explicitly named in mm and degrees.

    Formula attribution: MoveIt handeye_solver_base.h at MOVEIT_REFERENCE_COMMIT;
    see NOTICE.md for the upstream BSD license. Upstream returns rotation first,
    despite its GUI labelling the first value as translation. We use named fields.
    """
    validate_calibration_mode(calibration_mode)
    _validate_samples(samples)
    _validate_rigid_transform(reference_from_optical, "AX=XB camera transform")
    translation_errors = []
    rotation_errors = []
    for left, right in zip(samples, samples[1:]):
        if calibration_mode == CAMERA_ON_HAND:
            a_matrix = invert_transform(left.base_from_tool) @ right.base_from_tool
        else:
            a_matrix = left.base_from_tool @ invert_transform(right.base_from_tool)
        b_matrix = left.camera_from_target @ invert_transform(right.camera_from_target)
        ax = a_matrix @ reference_from_optical
        xb = reference_from_optical @ b_matrix
        translation_errors.append(
            0.5 * (
                float(np.linalg.norm(ax[:3, 3] - xb[:3, 3]))
                + float(np.linalg.norm(
                    invert_transform(ax)[:3, 3] - invert_transform(xb)[:3, 3]
                ))
            )
        )
        rotation_errors.append(rotation_angle_deg(ax[:3, :3].T @ xb[:3, :3]))
    return AxXbDiagnostics(
        len(samples) - 1,
        1000.0 * math.sqrt(float(np.mean(np.square(translation_errors)))),
        math.sqrt(float(np.mean(np.square(rotation_errors)))),
    )


def calibration_quality_from_transforms(
    sample_ids: list[str],
    transforms: list[np.ndarray],
) -> CalibrationQuality:
    if not transforms or len(sample_ids) != len(transforms):
        raise ValueError("Quality calculation requires one ID for every transform")
    for index, matrix in enumerate(transforms, start=1):
        _validate_transform(matrix, f"Quality transform {index}")
    mean_translation = np.mean([matrix[:3, 3] for matrix in transforms], axis=0)
    translation_errors = [
        np.linalg.norm(matrix[:3, 3] - mean_translation) for matrix in transforms
    ]
    rotation_sum = np.sum([matrix[:3, :3] for matrix in transforms], axis=0)
    u_matrix, _singular, vt_matrix = np.linalg.svd(rotation_sum)
    mean_rotation = u_matrix @ vt_matrix
    if np.linalg.det(mean_rotation) < 0.0:
        u_matrix[:, -1] *= -1.0
        mean_rotation = u_matrix @ vt_matrix
    rotation_errors = [
        rotation_angle_deg(mean_rotation.T @ matrix[:3, :3]) for matrix in transforms
    ]
    residuals = tuple(
        SampleResidual(sample_id, 1000.0 * float(translation_error), float(rotation_error))
        for sample_id, translation_error, rotation_error in zip(
            sample_ids,
            translation_errors,
            rotation_errors,
        )
    )
    maximum_translation = max(residuals, key=lambda residual: residual.translation_mm)
    maximum_rotation = max(residuals, key=lambda residual: residual.rotation_deg)
    constant_transform = np.eye(4, dtype=np.float64)
    constant_transform[:3, :3] = mean_rotation
    constant_transform[:3, 3] = mean_translation
    return CalibrationQuality(
        sample_count=len(transforms),
        translation_rms_mm=1000.0 * math.sqrt(float(np.mean(np.square(translation_errors)))),
        rotation_rms_deg=math.sqrt(float(np.mean(np.square(rotation_errors)))),
        max_translation_residual_mm=maximum_translation.translation_mm,
        max_translation_sample_id=maximum_translation.sample_id,
        max_rotation_residual_deg=maximum_rotation.rotation_deg,
        max_rotation_sample_id=maximum_rotation.sample_id,
        residuals=residuals,
        constant_transform=constant_transform,
    )


def _validate_solution(transform: np.ndarray) -> None:
    if not np.all(np.isfinite(transform)):
        raise InvalidPoseError("OpenCV returned a non-finite calibration transform")
    determinant = float(np.linalg.det(transform[:3, :3]))
    if abs(determinant - 1.0) > 1e-3:
        raise InvalidPoseError(
            f"OpenCV returned an invalid rotation determinant: {determinant}"
        )


def solve_camera_to_hand(
    samples: list[CalibrationSample],
) -> tuple[np.ndarray, CalibrationQuality]:
    """Solve a fixed camera as base_link <- camera optical frame."""
    _validate_samples(samples)

    rotations_robot = []
    translations_robot = []
    rotations_target = []
    translations_target = []
    for sample in samples:
        base_from_tool = sample.base_from_tool
        camera_from_target = sample.camera_from_target
        tool_from_base = invert_transform(base_from_tool)
        rotations_robot.append(tool_from_base[:3, :3])
        translations_robot.append(tool_from_base[:3, 3].reshape(3, 1))
        rotations_target.append(camera_from_target[:3, :3])
        translations_target.append(camera_from_target[:3, 3].reshape(3, 1))

    cv2 = _worker_opencv_module()
    rotation, translation = cv2.calibrateHandEye(
        rotations_robot,
        translations_robot,
        rotations_target,
        translations_target,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    base_from_camera = np.eye(4, dtype=np.float64)
    base_from_camera[:3, :3] = np.asarray(rotation, dtype=np.float64)
    base_from_camera[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    _validate_solution(base_from_camera)

    tool_from_targets = [
        invert_transform(sample.base_from_tool)
        @ base_from_camera
        @ sample.camera_from_target
        for sample in samples
    ]
    return base_from_camera, calibration_quality_from_transforms(
        [sample.sample_id for sample in samples],
        tool_from_targets,
    )


def solve_camera_on_hand(
    samples: list[CalibrationSample],
) -> tuple[np.ndarray, CalibrationQuality]:
    """Solve a wrist camera as Link6 <- camera optical frame."""
    _validate_samples(samples)

    rotations_robot = []
    translations_robot = []
    rotations_target = []
    translations_target = []
    for sample in samples:
        base_from_tool = sample.base_from_tool
        camera_from_target = sample.camera_from_target
        rotations_robot.append(base_from_tool[:3, :3])
        translations_robot.append(base_from_tool[:3, 3].reshape(3, 1))
        rotations_target.append(camera_from_target[:3, :3])
        translations_target.append(camera_from_target[:3, 3].reshape(3, 1))

    cv2 = _worker_opencv_module()
    rotation, translation = cv2.calibrateHandEye(
        rotations_robot,
        translations_robot,
        rotations_target,
        translations_target,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    tool_from_camera = np.eye(4, dtype=np.float64)
    tool_from_camera[:3, :3] = np.asarray(rotation, dtype=np.float64)
    tool_from_camera[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    _validate_solution(tool_from_camera)

    base_from_targets = [
        sample.base_from_tool @ tool_from_camera @ sample.camera_from_target
        for sample in samples
    ]
    return tool_from_camera, calibration_quality_from_transforms(
        [sample.sample_id for sample in samples],
        base_from_targets,
    )


def solve_calibration(
    calibration_mode: str,
    samples: list[CalibrationSample],
) -> tuple[np.ndarray, CalibrationQuality]:
    validate_calibration_mode(calibration_mode)
    if calibration_mode == CAMERA_TO_HAND:
        return solve_camera_to_hand(samples)
    return solve_camera_on_hand(samples)


def solution_change_diagnostics(
    previous_solution: np.ndarray | None,
    previous_quality: CalibrationQuality | None,
    current_solution: np.ndarray,
    current_quality: CalibrationQuality,
) -> SolutionChangeDiagnostics:
    _validate_transform(current_solution, "Current camera solution")
    if previous_solution is None or previous_quality is None:
        return SolutionChangeDiagnostics(False, None, None, None, None)
    previous = _validate_transform(previous_solution, "Previous camera solution")
    return SolutionChangeDiagnostics(
        available=True,
        translation_rms_delta_mm=(
            current_quality.translation_rms_mm - previous_quality.translation_rms_mm
        ),
        rotation_rms_delta_deg=(
            current_quality.rotation_rms_deg - previous_quality.rotation_rms_deg
        ),
        camera_translation_delta_mm=(
            1000.0
            * float(np.linalg.norm(current_solution[:3, 3] - previous[:3, 3]))
        ),
        camera_rotation_delta_deg=rotation_angle_deg(
            previous[:3, :3].T @ current_solution[:3, :3]
        ),
    )


def pose_coverage_diagnostics(
    samples: list[CalibrationSample],
) -> PoseCoverageDiagnostics:
    if not samples:
        return PoseCoverageDiagnostics(0.0, 0.0)
    maximum_translation = 0.0
    maximum_rotation = 0.0
    for left_index, left in enumerate(samples):
        for right in samples[left_index + 1:]:
            maximum_translation = max(
                maximum_translation,
                1000.0
                * float(
                    np.linalg.norm(
                        left.base_from_tool[:3, 3] - right.base_from_tool[:3, 3]
                    )
                ),
            )
            maximum_rotation = max(
                maximum_rotation,
                rotation_angle_deg(
                    left.base_from_tool[:3, :3].T
                    @ right.base_from_tool[:3, :3]
                ),
            )
    return PoseCoverageDiagnostics(maximum_translation, maximum_rotation)


def leave_one_out_diagnostics(
    calibration_mode: str,
    samples: list[CalibrationSample],
    camera_link_from_optical: np.ndarray,
    full_reference_from_camera_link: np.ndarray,
) -> LeaveOneOutDiagnostics:
    validate_calibration_mode(calibration_mode)
    _validate_transform(camera_link_from_optical, "Camera link from optical transform")
    _validate_transform(full_reference_from_camera_link, "Full camera-link solution")
    if len(samples) < LEAVE_ONE_OUT_MINIMUM_SAMPLES:
        return LeaveOneOutDiagnostics(
            "not_available", (), (), None, None, None, None, None, None
        )

    cv2 = _worker_opencv_module()
    optical_from_camera_link = invert_transform(camera_link_from_optical)
    entries = []
    failed_ids = []
    for omitted in samples:
        subset = [sample for sample in samples if sample.sample_id != omitted.sample_id]
        try:
            reference_from_optical, _quality = solve_calibration(
                calibration_mode,
                subset,
            )
            candidate = reference_from_optical @ optical_from_camera_link
            _validate_solution(candidate)
        except (ValueError, cv2.error):
            failed_ids.append(omitted.sample_id)
            continue
        entries.append(
            LeaveOneOutEntry(
                sample_id=omitted.sample_id,
                camera_translation_delta_mm=(
                    1000.0
                    * float(
                        np.linalg.norm(
                            candidate[:3, 3] - full_reference_from_camera_link[:3, 3]
                        )
                    )
                ),
                camera_rotation_delta_deg=rotation_angle_deg(
                    full_reference_from_camera_link[:3, :3].T @ candidate[:3, :3]
                ),
            )
        )

    if entries:
        translation_rms = math.sqrt(
            float(
                np.mean(
                    np.square(
                        [entry.camera_translation_delta_mm for entry in entries]
                    )
                )
            )
        )
        rotation_rms = math.sqrt(
            float(
                np.mean(
                    np.square([entry.camera_rotation_delta_deg for entry in entries])
                )
            )
        )
        maximum_translation = max(
            entries,
            key=lambda entry: entry.camera_translation_delta_mm,
        )
        maximum_rotation = max(
            entries,
            key=lambda entry: entry.camera_rotation_delta_deg,
        )
    else:
        translation_rms = None
        rotation_rms = None
        maximum_translation = None
        maximum_rotation = None
    return LeaveOneOutDiagnostics(
        status="failed" if failed_ids else "valid",
        entries=tuple(entries),
        failed_sample_ids=tuple(failed_ids),
        translation_rms_mm=translation_rms,
        rotation_rms_deg=rotation_rms,
        max_translation_delta_mm=(
            None
            if maximum_translation is None
            else maximum_translation.camera_translation_delta_mm
        ),
        max_translation_sample_id=(
            None if maximum_translation is None else maximum_translation.sample_id
        ),
        max_rotation_delta_deg=(
            None if maximum_rotation is None else maximum_rotation.camera_rotation_delta_deg
        ),
        max_rotation_sample_id=(
            None if maximum_rotation is None else maximum_rotation.sample_id
        ),
    )


def sample_diagnostic_rows(
    diagnostics: AccuracyDiagnostics,
) -> tuple[SampleDiagnosticRow, ...]:
    leave_one_out = {entry.sample_id: entry for entry in diagnostics.leave_one_out.entries}
    rows = []
    for residual in diagnostics.fit.residuals:
        loo_entry = leave_one_out.get(residual.sample_id)
        flags = []
        if residual.sample_id == diagnostics.fit.max_translation_sample_id:
            flags.append("MAX FIT T")
        if residual.sample_id == diagnostics.fit.max_rotation_sample_id:
            flags.append("MAX FIT R")
        if residual.sample_id == diagnostics.leave_one_out.max_translation_sample_id:
            flags.append("MAX LOO T")
        if residual.sample_id == diagnostics.leave_one_out.max_rotation_sample_id:
            flags.append("MAX LOO R")
        if residual.sample_id in diagnostics.leave_one_out.failed_sample_ids:
            flags.append("LOO FAILED")
        rows.append(
            SampleDiagnosticRow(
                residual.sample_id,
                residual.translation_mm,
                residual.rotation_deg,
                None if loo_entry is None else loo_entry.camera_translation_delta_mm,
                None if loo_entry is None else loo_entry.camera_rotation_delta_deg,
                tuple(flags),
            )
        )
    return tuple(rows)


def output_path_for_mode(
    calibration_mode: str,
    root: Path | None = None,
    created_at: datetime | None = None,
    filename: str | None = None,
) -> Path:
    validate_calibration_mode(calibration_mode)
    project_root = root or workspace_root()
    timestamp = created_at or datetime.now(timezone.utc)
    if timestamp.tzinfo is None:
        raise ValueError("Calibration output timestamp must include a timezone")
    timestamp_token = timestamp.astimezone(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    if filename is None:
        filename = f"{calibration_mode}_calibration_{timestamp_token}.yaml"
    else:
        if (not filename or filename != filename.strip() or filename.startswith(".")
                or any(char in filename for char in ("/", "\\", "\x00"))
                or any(ord(char) < 32 for char in filename)):
            raise ValueError(
                "Enter a filename only, inside calibration/ (no paths or hidden files)")
        if not Path(filename).suffix:
            filename += ".yaml"
        if not filename.endswith(".yaml"):
            raise ValueError("Calibration filename must end in .yaml")
    return project_root / "calibration" / filename


def output_pattern_for_mode(calibration_mode: str, root: Path | None = None) -> Path:
    validate_calibration_mode(calibration_mode)
    project_root = root or workspace_root()
    return project_root / "calibration" / f"{calibration_mode}_calibration_<UTC_TIMESTAMP>.yaml"


def _matrix_payload(matrix: np.ndarray, label: str) -> list[list[float]]:
    candidate = _validate_rigid_transform(matrix, label)
    return [[float(value) for value in row] for row in candidate]


def _validate_rigid_transform(matrix: np.ndarray, label: str) -> np.ndarray:
    candidate = _validate_transform(matrix, label)
    if not np.allclose(candidate[3, :], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{label} must have homogeneous bottom row [0, 0, 0, 1]")
    rotation = candidate[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        raise ValueError(f"{label} rotation must be orthonormal")
    if abs(float(np.linalg.det(rotation)) - 1.0) > 1e-5:
        raise ValueError(f"{label} rotation determinant must be 1")
    return candidate


def _exact_mapping(value: object, label: str, keys: set[str]) -> dict:
    if not isinstance(value, dict) or set(value) != keys:
        actual = sorted(value) if isinstance(value, dict) else []
        raise ValueError(f"{label} must contain exactly {sorted(keys)}; actual={actual}")
    return value


def _artifact_text(mapping: dict, key: str, label: str) -> str:
    value = mapping[key]
    if type(value) is not str:
        raise ValueError(f"{label}.{key} must be a string")
    return value


def _artifact_integer(mapping: dict, key: str, label: str) -> int:
    value = mapping[key]
    if type(value) is not int:
        raise ValueError(f"{label}.{key} must be an integer")
    return value


def _artifact_number(
    mapping: dict,
    key: str,
    label: str,
    *,
    optional: bool = False,
) -> float | None:
    value = mapping[key]
    if optional and value is None:
        return None
    if type(value) not in {int, float} or not math.isfinite(float(value)):
        suffix = " or null" if optional else ""
        raise ValueError(f"{label}.{key} must be a finite number{suffix}")
    return float(value)


def _artifact_matrix(value: object, label: str) -> np.ndarray:
    if not isinstance(value, list) or len(value) != 4:
        raise ValueError(f"{label} must contain exactly four rows")
    if any(not isinstance(row, list) or len(row) != 4 for row in value):
        raise ValueError(f"{label} rows must each contain exactly four numbers")
    if any(type(item) not in {int, float} for row in value for item in row):
        raise ValueError(f"{label} entries must be numbers")
    return _validate_rigid_transform(np.asarray(value, dtype=np.float64), label)


def calibration_pipeline_payload() -> dict:
    return {
        "reference_repository": MOVEIT_REFERENCE_URL,
        "reference_commit": MOVEIT_REFERENCE_COMMIT,
        "opencv_version": REQUIRED_OPENCV_VERSION,
        "opencv_wheel_sha256": REQUIRED_OPENCV_WHEEL_SHA256,
        "detector_api": REQUIRED_OPENCV_API,
        "charuco_legacy_pattern": REQUIRED_CHARUCO_LEGACY_PATTERN,
        "marker_corner_refinement": "CORNER_REFINE_NONE",
        "adjacent_marker_minimum": 2,
        "marker_recovery": False,
        "pose_method": "SOLVEPNP_ITERATIVE",
        "hand_eye_method": REQUIRED_HAND_EYE_METHOD,
        "minimum_charuco_corners": MINIMUM_CHARUCO_CORNERS,
        "minimum_samples": MINIMUM_CALIBRATION_SAMPLES,
        "minimum_sample_rotation_deg": MINIMUM_SAMPLE_ROTATION_DEG,
    }


def _calibration_artifact_payload(
    calibration_mode: str,
    settings: CharucoSettings,
    reference_from_camera_link: np.ndarray,
    diagnostics: AccuracyDiagnostics,
    samples: list[CalibrationSample],
    created_at: datetime,
) -> dict:
    reference_frame = reference_frame_for_mode(calibration_mode)
    translation = reference_from_camera_link[:3, 3]
    qx, qy, qz, qw = rotation_matrix_to_quaternion(reference_from_camera_link[:3, :3])
    camera_mount = (
        "fixed_relative_to_base_link"
        if calibration_mode == CAMERA_TO_HAND
        else "mounted_on_Link6"
    )
    charuco_mount = (
        "mounted_on_Link6"
        if calibration_mode == CAMERA_TO_HAND
        else "fixed_relative_to_base_link"
    )
    fit = diagnostics.fit
    change = diagnostics.solution_change
    leave_one_out = diagnostics.leave_one_out
    rows = sample_diagnostic_rows(diagnostics)
    return {
        "schema_version": CALIBRATION_ARTIFACT_SCHEMA_VERSION,
        "pipeline": calibration_pipeline_payload(),
        "calibration_mode": calibration_mode,
        "created_utc": _canonical_utc_text(created_at),
        "camera_mount": camera_mount,
        "charuco_mount": charuco_mount,
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
        },
        "transform": {
            "target_frame": reference_frame,
            "source_frame": settings.camera_link_frame,
            "convention": "target_from_source",
            "translation_m": {
                "x": float(translation[0]),
                "y": float(translation[1]),
                "z": float(translation[2]),
            },
            "rotation_xyzw": {"x": qx, "y": qy, "z": qz, "w": qw},
        },
        "robot": {
            "base_frame": BASE_FRAME,
            "tool_frame": TOOL_FRAME,
            "joint_state_topic": JOINT_STATE_TOPIC,
            "joint_names": list(JOINT_NAMES),
            "joint_position_unit": "radian",
        },
        "quality": {
            "ax_xb": {
                "pair_count": diagnostics.ax_xb.pair_count,
                "translation_rms_mm": diagnostics.ax_xb.translation_rms_mm,
                "rotation_rms_deg": diagnostics.ax_xb.rotation_rms_deg,
            },
            "calibration_fit": {
                "sample_count": fit.sample_count,
                "translation_rms_mm": fit.translation_rms_mm,
                "rotation_rms_deg": fit.rotation_rms_deg,
                "max_translation_residual_mm": fit.max_translation_residual_mm,
                "max_translation_sample_id": fit.max_translation_sample_id,
                "max_rotation_residual_deg": fit.max_rotation_residual_deg,
                "max_rotation_sample_id": fit.max_rotation_sample_id,
            },
            "solution_change": {
                "available": change.available,
                "translation_rms_delta_mm": change.translation_rms_delta_mm,
                "rotation_rms_delta_deg": change.rotation_rms_delta_deg,
                "camera_translation_delta_mm": change.camera_translation_delta_mm,
                "camera_rotation_delta_deg": change.camera_rotation_delta_deg,
            },
            "leave_one_out": {
                "status": leave_one_out.status,
                "required_sample_count": LEAVE_ONE_OUT_MINIMUM_SAMPLES,
                "evaluated_sample_count": len(leave_one_out.entries),
                "failed_sample_ids": list(leave_one_out.failed_sample_ids),
                "translation_rms_mm": leave_one_out.translation_rms_mm,
                "rotation_rms_deg": leave_one_out.rotation_rms_deg,
                "max_translation_delta_mm": leave_one_out.max_translation_delta_mm,
                "max_translation_sample_id": leave_one_out.max_translation_sample_id,
                "max_rotation_delta_deg": leave_one_out.max_rotation_delta_deg,
                "max_rotation_sample_id": leave_one_out.max_rotation_sample_id,
            },
            "pose_coverage": {
                "translation_span_mm": diagnostics.pose_coverage.translation_span_mm,
                "rotation_span_deg": diagnostics.pose_coverage.rotation_span_deg,
            },
        },
        "captured_samples": [
            {
                "id": sample.sample_id,
                "base_from_tool": _matrix_payload(
                    sample.base_from_tool,
                    f"Calibration sample {sample.sample_id} base_from_tool",
                ),
                "camera_from_target": _matrix_payload(
                    sample.camera_from_target,
                    f"Calibration sample {sample.sample_id} camera_from_target",
                ),
                "joint_positions_rad": {
                    joint_name: position
                    for joint_name, position in zip(
                        JOINT_NAMES,
                        sample.joint_positions_rad,
                    )
                },
            }
            for sample in samples
        ],
        "sample_diagnostics": [
            {
                "id": row.sample_id,
                "fit_translation_residual_mm": row.translation_residual_mm,
                "fit_rotation_residual_deg": row.rotation_residual_deg,
                "leave_one_out_translation_delta_mm": (
                    row.leave_one_out_translation_mm
                ),
                "leave_one_out_rotation_delta_deg": row.leave_one_out_rotation_deg,
                "flags": list(row.flags),
            }
            for row in rows
        ],
    }


def write_calibration_yaml(
    path: Path,
    calibration_mode: str,
    settings: CharucoSettings,
    reference_from_camera_link: np.ndarray,
    diagnostics: AccuracyDiagnostics,
    samples: list[CalibrationSample],
    created_at: datetime | None = None,
) -> None:
    validate_calibration_mode(calibration_mode)
    settings.validate()
    _validate_rigid_transform(
        reference_from_camera_link,
        "Camera-link calibration transform",
    )
    _validate_samples(samples)
    if not isinstance(diagnostics, AccuracyDiagnostics):
        raise ValueError("Calibration YAML requires complete accuracy diagnostics")
    if not diagnostics.save_allowed:
        raise ValueError("Calibration YAML cannot be saved while leave-one-out diagnostics fail")
    sample_ids = [sample.sample_id for sample in samples]
    if diagnostics.fit.sample_count != len(samples):
        raise ValueError("Calibration diagnostics sample count does not match captured samples")
    if [item.sample_id for item in diagnostics.fit.residuals] != sample_ids:
        raise ValueError("Calibration diagnostics IDs do not match captured sample IDs")
    timestamp = created_at or datetime.now(timezone.utc)
    if timestamp.tzinfo is None:
        raise ValueError("Calibration creation timestamp must include a timezone")
    payload = _calibration_artifact_payload(
        calibration_mode,
        settings,
        reference_from_camera_link,
        diagnostics,
        samples,
        timestamp,
    )
    content = yaml.safe_dump(payload, sort_keys=False, allow_unicode=False)

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            delete=False,
        ) as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
            temporary_path = Path(stream.name)
        os.link(temporary_path, path)
        temporary_path.unlink()
        temporary_path = None
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def load_calibration_yaml(path: Path) -> CalibrationArtifact:
    if not path.is_file():
        raise ValueError(f"Calibration artifact is not a file: {path}")
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f"Cannot read calibration artifact {path}: {exc}") from exc
    if (
        not isinstance(payload, dict)
        or type(payload.get("schema_version")) is not int
        or payload["schema_version"] != CALIBRATION_ARTIFACT_SCHEMA_VERSION
    ):
        raise ValueError(
            "Calibration artifact schema_version must be exactly "
            f"{CALIBRATION_ARTIFACT_SCHEMA_VERSION}; older artifacts are not converted"
        )
    root = _exact_mapping(
        payload,
        "Calibration artifact",
        {
            "schema_version",
            "calibration_mode",
            "created_utc",
            "camera_mount",
            "charuco_mount",
            "camera",
            "pipeline",
            "charuco",
            "transform",
            "robot",
            "quality",
            "captured_samples",
            "sample_diagnostics",
        },
    )
    if type(root["schema_version"]) is not int or (
        root["schema_version"] != CALIBRATION_ARTIFACT_SCHEMA_VERSION
    ):
        raise ValueError(
            "Calibration artifact schema_version must be exactly "
            f"{CALIBRATION_ARTIFACT_SCHEMA_VERSION}"
        )
    expected_pipeline = calibration_pipeline_payload()
    pipeline = _exact_mapping(root["pipeline"], "Calibration pipeline", set(expected_pipeline))
    for key, expected in expected_pipeline.items():
        if type(pipeline[key]) is not type(expected) or pipeline[key] != expected:
            raise ValueError(f"Calibration pipeline.{key} must be exactly {expected!r}")
    calibration_mode = validate_calibration_mode(
        _artifact_text(root, "calibration_mode", "Calibration artifact")
    )
    created_at_text = _artifact_text(root, "created_utc", "Calibration artifact")
    if not created_at_text.endswith("Z"):
        raise ValueError("Calibration artifact created_utc must end with Z")
    try:
        created_at = datetime.fromisoformat(created_at_text[:-1] + "+00:00")
    except ValueError as exc:
        raise ValueError("Calibration artifact created_utc is invalid") from exc
    if created_at_text != _canonical_utc_text(created_at):
        raise ValueError("Calibration artifact created_utc is not canonical UTC")

    expected_camera_mount = (
        "fixed_relative_to_base_link"
        if calibration_mode == CAMERA_TO_HAND
        else "mounted_on_Link6"
    )
    expected_charuco_mount = (
        "mounted_on_Link6"
        if calibration_mode == CAMERA_TO_HAND
        else "fixed_relative_to_base_link"
    )
    if _artifact_text(root, "camera_mount", "Calibration artifact") != expected_camera_mount:
        raise ValueError("Calibration artifact camera_mount conflicts with calibration_mode")
    if _artifact_text(root, "charuco_mount", "Calibration artifact") != expected_charuco_mount:
        raise ValueError("Calibration artifact charuco_mount conflicts with calibration_mode")

    camera = _exact_mapping(
        root["camera"],
        "Calibration artifact camera",
        {
            "prefix",
            "link_frame",
            "optical_frame",
            "color_topic",
            "camera_info_topic",
        },
    )
    charuco = _exact_mapping(
        root["charuco"],
        "Calibration artifact charuco",
        {
            "dictionary",
            "squares_x",
            "squares_y",
            "square_length_mm",
            "marker_length_mm",
        },
    )
    settings = CharucoSettings(
        camera_prefix=_artifact_text(camera, "prefix", "Calibration artifact camera"),
        dictionary_name=_artifact_text(
            charuco,
            "dictionary",
            "Calibration artifact charuco",
        ),
        squares_x=_artifact_integer(charuco, "squares_x", "Calibration artifact charuco"),
        squares_y=_artifact_integer(charuco, "squares_y", "Calibration artifact charuco"),
        square_length_mm=_artifact_number(
            charuco,
            "square_length_mm",
            "Calibration artifact charuco",
        ),
        marker_length_mm=_artifact_number(
            charuco,
            "marker_length_mm",
            "Calibration artifact charuco",
        ),
    )
    settings.validate()
    expected_camera = {
        "link_frame": settings.camera_link_frame,
        "optical_frame": settings.optical_frame,
        "color_topic": settings.color_topic,
        "camera_info_topic": settings.camera_info_topic,
    }
    for key, expected in expected_camera.items():
        if _artifact_text(camera, key, "Calibration artifact camera") != expected:
            raise ValueError(f"Calibration artifact camera.{key} conflicts with prefix")

    transform = _exact_mapping(
        root["transform"],
        "Calibration artifact transform",
        {"target_frame", "source_frame", "convention", "translation_m", "rotation_xyzw"},
    )
    if _artifact_text(transform, "target_frame", "Calibration artifact transform") != (
        reference_frame_for_mode(calibration_mode)
    ):
        raise ValueError("Calibration artifact transform.target_frame conflicts with mode")
    if _artifact_text(transform, "source_frame", "Calibration artifact transform") != (
        settings.camera_link_frame
    ):
        raise ValueError("Calibration artifact transform.source_frame conflicts with prefix")
    if _artifact_text(transform, "convention", "Calibration artifact transform") != (
        "target_from_source"
    ):
        raise ValueError("Calibration artifact transform.convention must be target_from_source")
    translation = _exact_mapping(
        transform["translation_m"],
        "Calibration artifact transform.translation_m",
        {"x", "y", "z"},
    )
    quaternion = _exact_mapping(
        transform["rotation_xyzw"],
        "Calibration artifact transform.rotation_xyzw",
        {"x", "y", "z", "w"},
    )
    translation_values = [
        _artifact_number(
            translation,
            key,
            "Calibration artifact transform.translation_m",
        )
        for key in ("x", "y", "z")
    ]
    quaternion_values = [
        _artifact_number(
            quaternion,
            key,
            "Calibration artifact transform.rotation_xyzw",
        )
        for key in ("x", "y", "z", "w")
    ]
    if abs(float(np.linalg.norm(quaternion_values)) - 1.0) > 1e-5:
        raise ValueError("Calibration artifact transform quaternion must have unit length")
    reference_from_camera_link = np.eye(4, dtype=np.float64)
    reference_from_camera_link[:3, :3] = quaternion_to_rotation_matrix(
        *quaternion_values
    )
    reference_from_camera_link[:3, 3] = translation_values
    _validate_rigid_transform(
        reference_from_camera_link,
        "Calibration artifact camera-link transform",
    )

    robot = _exact_mapping(
        root["robot"],
        "Calibration artifact robot",
        {
            "base_frame",
            "tool_frame",
            "joint_state_topic",
            "joint_names",
            "joint_position_unit",
        },
    )
    if _artifact_text(robot, "base_frame", "Calibration artifact robot") != BASE_FRAME:
        raise ValueError(f"Calibration artifact robot.base_frame must be {BASE_FRAME}")
    if _artifact_text(robot, "tool_frame", "Calibration artifact robot") != TOOL_FRAME:
        raise ValueError(f"Calibration artifact robot.tool_frame must be {TOOL_FRAME}")
    if (
        _artifact_text(robot, "joint_state_topic", "Calibration artifact robot")
        != JOINT_STATE_TOPIC
    ):
        raise ValueError(
            f"Calibration artifact robot.joint_state_topic must be {JOINT_STATE_TOPIC}"
        )
    if robot["joint_names"] != list(JOINT_NAMES):
        raise ValueError(
            "Calibration artifact robot.joint_names must be exactly "
            f"{list(JOINT_NAMES)}"
        )
    if (
        _artifact_text(robot, "joint_position_unit", "Calibration artifact robot")
        != "radian"
    ):
        raise ValueError(
            "Calibration artifact robot.joint_position_unit must be exactly radian"
        )

    raw_samples = root["captured_samples"]
    if not isinstance(raw_samples, list):
        raise ValueError("Calibration artifact captured_samples must be a list")
    samples = []
    for index, raw_sample in enumerate(raw_samples, start=1):
        sample = _exact_mapping(
            raw_sample,
            f"Calibration artifact captured_samples[{index}]",
            {
                "id",
                "base_from_tool",
                "camera_from_target",
                "joint_positions_rad",
            },
        )
        sample_id = _artifact_text(
            sample,
            "id",
            f"Calibration artifact captured_samples[{index}]",
        )
        joint_positions = _exact_mapping(
            sample["joint_positions_rad"],
            f"Calibration artifact sample {sample_id} joint_positions_rad",
            set(JOINT_NAMES),
        )
        samples.append(
            CalibrationSample(
                sample_id,
                _artifact_matrix(
                    sample["base_from_tool"],
                    f"Calibration artifact sample {sample_id} base_from_tool",
                ),
                _artifact_matrix(
                    sample["camera_from_target"],
                    f"Calibration artifact sample {sample_id} camera_from_target",
                ),
                tuple(
                    _artifact_number(
                        joint_positions,
                        joint_name,
                        (
                            f"Calibration artifact sample {sample_id} "
                            "joint_positions_rad"
                        ),
                    )
                    for joint_name in JOINT_NAMES
                ),
            )
        )
    _validate_samples(samples)
    sample_ids = [sample.sample_id for sample in samples]

    quality = _exact_mapping(
        root["quality"],
        "Calibration artifact quality",
        {"calibration_fit", "solution_change", "leave_one_out", "pose_coverage", "ax_xb"},
    )
    ax_xb = _exact_mapping(
        quality["ax_xb"],
        "Calibration artifact quality.ax_xb",
        {"pair_count", "translation_rms_mm", "rotation_rms_deg"},
    )
    if _artifact_integer(ax_xb, "pair_count", "AX=XB") != len(samples) - 1:
        raise ValueError("AX=XB pair count must equal sample count minus one")
    for key in ("translation_rms_mm", "rotation_rms_deg"):
        if _artifact_number(ax_xb, key, "AX=XB") < 0.0:
            raise ValueError(f"AX=XB {key} must be non-negative")
    fit = _exact_mapping(
        quality["calibration_fit"],
        "Calibration artifact quality.calibration_fit",
        {
            "sample_count",
            "translation_rms_mm",
            "rotation_rms_deg",
            "max_translation_residual_mm",
            "max_translation_sample_id",
            "max_rotation_residual_deg",
            "max_rotation_sample_id",
        },
    )
    if _artifact_integer(
        fit,
        "sample_count",
        "Calibration artifact quality.calibration_fit",
    ) != len(samples):
        raise ValueError(
            "Calibration artifact quality sample count conflicts with captured samples"
        )
    for key in (
        "translation_rms_mm",
        "rotation_rms_deg",
        "max_translation_residual_mm",
        "max_rotation_residual_deg",
    ):
        _artifact_number(fit, key, "Calibration artifact quality.calibration_fit")
    for key in ("max_translation_sample_id", "max_rotation_sample_id"):
        if _artifact_text(
            fit,
            key,
            "Calibration artifact quality.calibration_fit",
        ) not in sample_ids:
            raise ValueError(f"Calibration artifact quality.{key} is not a captured sample")

    change = _exact_mapping(
        quality["solution_change"],
        "Calibration artifact quality.solution_change",
        {
            "available",
            "translation_rms_delta_mm",
            "rotation_rms_delta_deg",
            "camera_translation_delta_mm",
            "camera_rotation_delta_deg",
        },
    )
    if type(change["available"]) is not bool:
        raise ValueError("Calibration artifact quality.solution_change.available must be boolean")
    for key in (
        "translation_rms_delta_mm",
        "rotation_rms_delta_deg",
        "camera_translation_delta_mm",
        "camera_rotation_delta_deg",
    ):
        value = _artifact_number(
            change,
            key,
            "Calibration artifact quality.solution_change",
            optional=True,
        )
        if change["available"] != (value is not None):
            raise ValueError(
                "Calibration artifact solution-change values must all match available"
            )

    leave_one_out = _exact_mapping(
        quality["leave_one_out"],
        "Calibration artifact quality.leave_one_out",
        {
            "status",
            "required_sample_count",
            "evaluated_sample_count",
            "failed_sample_ids",
            "translation_rms_mm",
            "rotation_rms_deg",
            "max_translation_delta_mm",
            "max_translation_sample_id",
            "max_rotation_delta_deg",
            "max_rotation_sample_id",
        },
    )
    status = _artifact_text(
        leave_one_out,
        "status",
        "Calibration artifact quality.leave_one_out",
    )
    expected_status = (
        "not_available"
        if len(samples) < LEAVE_ONE_OUT_MINIMUM_SAMPLES
        else "valid"
    )
    if status != expected_status:
        raise ValueError(
            f"Calibration artifact leave-one-out status must be {expected_status}"
        )
    if _artifact_integer(
        leave_one_out,
        "required_sample_count",
        "Calibration artifact quality.leave_one_out",
    ) != LEAVE_ONE_OUT_MINIMUM_SAMPLES:
        raise ValueError("Calibration artifact leave-one-out required count is not canonical")
    expected_evaluated = (
        0 if len(samples) < LEAVE_ONE_OUT_MINIMUM_SAMPLES else len(samples)
    )
    if _artifact_integer(
        leave_one_out,
        "evaluated_sample_count",
        "Calibration artifact quality.leave_one_out",
    ) != expected_evaluated:
        raise ValueError("Calibration artifact leave-one-out evaluated count is inconsistent")
    if leave_one_out["failed_sample_ids"] != []:
        raise ValueError("A saved calibration artifact cannot contain leave-one-out failures")
    loo_metric_keys = (
        "translation_rms_mm",
        "rotation_rms_deg",
        "max_translation_delta_mm",
        "max_rotation_delta_deg",
    )
    for key in loo_metric_keys:
        value = _artifact_number(
            leave_one_out,
            key,
            "Calibration artifact quality.leave_one_out",
            optional=True,
        )
        if (status == "valid") != (value is not None):
            raise ValueError("Calibration artifact leave-one-out metrics conflict with status")
    for key in ("max_translation_sample_id", "max_rotation_sample_id"):
        value = leave_one_out[key]
        if status == "not_available" and value is not None:
            raise ValueError("Unavailable leave-one-out sample IDs must be null")
        if status == "valid" and (type(value) is not str or value not in sample_ids):
            raise ValueError("Valid leave-one-out sample IDs must reference captured samples")

    coverage = _exact_mapping(
        quality["pose_coverage"],
        "Calibration artifact quality.pose_coverage",
        {"translation_span_mm", "rotation_span_deg"},
    )
    for key in ("translation_span_mm", "rotation_span_deg"):
        _artifact_number(coverage, key, "Calibration artifact quality.pose_coverage")

    diagnostic_rows = root["sample_diagnostics"]
    if not isinstance(diagnostic_rows, list) or len(diagnostic_rows) != len(samples):
        raise ValueError(
            "Calibration artifact sample_diagnostics must match captured sample count"
        )
    diagnostic_ids = []
    for index, raw_row in enumerate(diagnostic_rows, start=1):
        row = _exact_mapping(
            raw_row,
            f"Calibration artifact sample_diagnostics[{index}]",
            {
                "id",
                "fit_translation_residual_mm",
                "fit_rotation_residual_deg",
                "leave_one_out_translation_delta_mm",
                "leave_one_out_rotation_delta_deg",
                "flags",
            },
        )
        diagnostic_ids.append(
            _artifact_text(row, "id", f"Calibration artifact sample_diagnostics[{index}]")
        )
        for key in (
            "fit_translation_residual_mm",
            "fit_rotation_residual_deg",
        ):
            _artifact_number(row, key, f"Calibration artifact sample_diagnostics[{index}]")
        for key in (
            "leave_one_out_translation_delta_mm",
            "leave_one_out_rotation_delta_deg",
        ):
            value = _artifact_number(
                row,
                key,
                f"Calibration artifact sample_diagnostics[{index}]",
                optional=True,
            )
            if (status == "valid") != (value is not None):
                raise ValueError(
                    "Calibration artifact per-sample leave-one-out values conflict with status"
                )
        if not isinstance(row["flags"], list) or any(
            type(flag) is not str for flag in row["flags"]
        ):
            raise ValueError("Calibration artifact diagnostic flags must be a string list")
    if diagnostic_ids != sample_ids:
        raise ValueError("Calibration artifact diagnostic IDs do not match captured samples")
    return CalibrationArtifact(
        created_at_utc=created_at_text,
        calibration_mode=calibration_mode,
        settings=settings,
        reference_from_camera_link=reference_from_camera_link,
        samples=tuple(samples),
    )
