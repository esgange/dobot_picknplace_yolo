import json
import math
import os
import re
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path


PACKAGE_UI_STATE_SCHEMA_VERSION = 6
ARUCO_5X5_DICTIONARIES = (
    "DICT_5X5_50",
    "DICT_5X5_100",
    "DICT_5X5_250",
    "DICT_5X5_1000",
)


@dataclass(frozen=True)
class PackageUiState:
    saved_at_utc: str
    platform_camera_calibration_filename: str | None
    bin_platform_calibration_filename: str | None
    bin_aruco_dictionary: str | None
    bin_marker_size_mm: float | None
    item_profile_filename: str | None = None
    item_preview_camera_prefix: str | None = None
    item_platform_filename: str | None = None
    item_bin_filename: str | None = None


def _canonical_utc_text(timestamp: datetime) -> str:
    if timestamp.tzinfo is None:
        raise ValueError("UI-state timestamp must include a timezone")
    return (
        timestamp.astimezone(timezone.utc)
        .isoformat(timespec="microseconds")
        .replace("+00:00", "Z")
    )


def _canonical_utc_value(value) -> str:
    if type(value) is not str or not value.endswith("Z"):
        raise ValueError("UI-state saved_at_utc must be canonical UTC")
    try:
        parsed = datetime.fromisoformat(value[:-1] + "+00:00")
    except ValueError as exc:
        raise ValueError("UI-state saved_at_utc must be canonical UTC") from exc
    if value != _canonical_utc_text(parsed):
        raise ValueError("UI-state saved_at_utc must be canonical UTC")
    return value


def _validate_filename(value, prefix: str, label: str) -> str | None:
    if value is None:
        return None
    if (
        type(value) is not str
        or Path(value).name != value
        or not value.startswith(prefix)
        or Path(value).suffix != ".yaml"
    ):
        raise ValueError(f"{label} filename is invalid")
    return value


def _validate_camera_calibration_filename(value, label: str) -> str | None:
    if value is None:
        return None
    if (
        type(value) is not str
        or Path(value).name != value
        or Path(value).suffix != ".yaml"
        or not value.startswith(
            ("camera_to_hand_calibration_", "camera_on_hand_calibration_")
        )
    ):
        raise ValueError(f"{label} filename is invalid")
    return value


def load_package_ui_state(path: Path) -> PackageUiState | None:
    if not path.exists():
        return None
    if not path.is_file():
        raise ValueError(f"Item-perception UI state is not a file: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"Cannot read item-perception UI state {path}: {exc}") from exc
    if not isinstance(payload, dict) or set(payload) != {
        "schema_version",
        "saved_at_utc",
        "platform_teach",
        "bin_teach",
        "item_teach",
    }:
        raise ValueError("Item-perception UI state has unsupported or missing fields")
    if payload["schema_version"] != PACKAGE_UI_STATE_SCHEMA_VERSION:
        raise ValueError(
            "Item-perception UI state schema_version must be exactly "
            f"{PACKAGE_UI_STATE_SCHEMA_VERSION}"
        )
    platform = payload["platform_teach"]
    bin_teach = payload["bin_teach"]
    item = payload["item_teach"]
    if type(item) is not dict or set(item) != {"profile_filename", "preview_camera_prefix",
                                               "platform_filename", "bin_filename"}:
        raise ValueError("Item-teach UI fields are invalid")
    item_filename = _validate_filename(item["profile_filename"], "item_teach_", "Item teach")
    item_platform = _validate_filename(
        item["platform_filename"], "platform_calibration_", "Platform")
    item_bin = _validate_filename(item["bin_filename"], "bin_teach_", "Bin")
    if (item_platform is None) != (item_bin is None):
        raise ValueError("Item station fields must be both null or both selected")
    preview_prefix = item["preview_camera_prefix"]
    if preview_prefix is not None and (
        type(preview_prefix) is not str
        or re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", preview_prefix) is None
    ):
        raise ValueError("Item preview camera prefix is invalid")
    if not isinstance(platform, dict) or set(platform) != {
        "camera_calibration_filename"
    }:
        raise ValueError("Item-perception platform_teach UI fields are invalid")
    if not isinstance(bin_teach, dict) or set(bin_teach) != {
        "platform_calibration_filename",
        "aruco_dictionary",
        "marker_size_mm",
    }:
        raise ValueError("Item-perception bin_teach UI fields are invalid")
    platform_filename = _validate_camera_calibration_filename(
        platform["camera_calibration_filename"],
        "Platform-teach camera calibration",
    )
    bin_filename = _validate_filename(
        bin_teach["platform_calibration_filename"],
        "platform_calibration_",
        "Bin-teach platform calibration",
    )
    dictionary = bin_teach["aruco_dictionary"]
    marker_size = bin_teach["marker_size_mm"]
    bin_values = (bin_filename, dictionary, marker_size)
    if all(value is None for value in bin_values):
        dictionary = None
        marker_size = None
    elif any(value is None for value in bin_values):
        raise ValueError("Bin-teach UI fields must be all null or all configured")
    else:
        if type(dictionary) is not str or dictionary not in ARUCO_5X5_DICTIONARIES:
            raise ValueError("Bin-teach UI dictionary must be an exact 5x5 dictionary")
        if type(marker_size) not in {int, float} or not math.isfinite(float(marker_size)):
            raise ValueError("Bin-teach UI marker_size_mm must be finite")
        marker_size = float(marker_size)
        if marker_size <= 0.0:
            raise ValueError("Bin-teach UI marker_size_mm must be greater than zero")
    return PackageUiState(
        saved_at_utc=_canonical_utc_value(payload["saved_at_utc"]),
        platform_camera_calibration_filename=platform_filename,
        bin_platform_calibration_filename=bin_filename,
        bin_aruco_dictionary=dictionary,
        bin_marker_size_mm=marker_size,
        item_profile_filename=item_filename,
        item_preview_camera_prefix=preview_prefix,
        item_platform_filename=item_platform, item_bin_filename=item_bin,
    )


def _write(path: Path, state: PackageUiState) -> PackageUiState:
    payload = {
        "schema_version": PACKAGE_UI_STATE_SCHEMA_VERSION,
        "saved_at_utc": state.saved_at_utc,
        "platform_teach": {
            "camera_calibration_filename": state.platform_camera_calibration_filename,
        },
        "bin_teach": {
            "platform_calibration_filename": state.bin_platform_calibration_filename,
            "aruco_dictionary": state.bin_aruco_dictionary,
            "marker_size_mm": state.bin_marker_size_mm,
        },
        "item_teach": {"profile_filename": state.item_profile_filename,
                       "preview_camera_prefix": state.item_preview_camera_prefix,
                       "platform_filename": state.item_platform_filename,
                       "bin_filename": state.item_bin_filename},
    }
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


def _current_or_blank(path: Path, saved_at: datetime) -> PackageUiState:
    current = load_package_ui_state(path)
    if current is not None:
        return current
    return PackageUiState(
        saved_at_utc=_canonical_utc_text(saved_at),
        platform_camera_calibration_filename=None,
        bin_platform_calibration_filename=None,
        bin_aruco_dictionary=None,
        bin_marker_size_mm=None,
    )


def write_platform_ui_state(
    path: Path,
    camera_calibration_filename: str,
    saved_at: datetime | None = None,
) -> PackageUiState:
    timestamp = saved_at or datetime.now(timezone.utc)
    filename = _validate_camera_calibration_filename(
        camera_calibration_filename,
        "Platform-teach camera calibration",
    )
    if filename is None:
        raise ValueError("Platform-teach camera calibration filename is required")
    current = _current_or_blank(path, timestamp)
    return _write(
        path,
        PackageUiState(
            saved_at_utc=_canonical_utc_text(timestamp),
            platform_camera_calibration_filename=filename,
            bin_platform_calibration_filename=current.bin_platform_calibration_filename,
            bin_aruco_dictionary=current.bin_aruco_dictionary,
            bin_marker_size_mm=current.bin_marker_size_mm,
            item_profile_filename=current.item_profile_filename,
            item_preview_camera_prefix=current.item_preview_camera_prefix,
            item_platform_filename=current.item_platform_filename,
            item_bin_filename=current.item_bin_filename,
        ),
    )


def write_bin_ui_state(
    path: Path,
    platform_calibration_filename: str,
    aruco_dictionary: str,
    marker_size_mm: float,
    saved_at: datetime | None = None,
) -> PackageUiState:
    timestamp = saved_at or datetime.now(timezone.utc)
    filename = _validate_filename(
        platform_calibration_filename,
        "platform_calibration_",
        "Bin-teach platform calibration",
    )
    if filename is None:
        raise ValueError("Bin-teach platform calibration filename is required")
    if aruco_dictionary not in ARUCO_5X5_DICTIONARIES:
        raise ValueError("Bin-teach dictionary must be an exact 5x5 dictionary")
    if type(marker_size_mm) not in {int, float} or not math.isfinite(
        float(marker_size_mm)
    ):
        raise ValueError("Bin-teach marker size must be finite")
    marker_size = float(marker_size_mm)
    if marker_size <= 0.0:
        raise ValueError("Bin-teach marker size must be greater than zero")
    current = _current_or_blank(path, timestamp)
    return _write(
        path,
        PackageUiState(
            saved_at_utc=_canonical_utc_text(timestamp),
            platform_camera_calibration_filename=(
                current.platform_camera_calibration_filename
            ),
            bin_platform_calibration_filename=filename,
            bin_aruco_dictionary=aruco_dictionary,
            bin_marker_size_mm=marker_size,
            item_profile_filename=current.item_profile_filename,
            item_preview_camera_prefix=current.item_preview_camera_prefix,
            item_platform_filename=current.item_platform_filename,
            item_bin_filename=current.item_bin_filename,
        ),
    )


def write_item_ui_state(path: Path, profile_filename: str) -> PackageUiState:
    from dataclasses import replace

    filename = _validate_filename(profile_filename, "item_teach_", "Item teach")
    if filename is None:
        raise ValueError("Item profile filename is required")
    timestamp = datetime.now(timezone.utc)
    current = _current_or_blank(path, timestamp)
    return _write(path, replace(
        current, saved_at_utc=_canonical_utc_text(timestamp), item_profile_filename=filename,
    ))


def write_item_preview_state(path: Path, prefix: str) -> PackageUiState:
    from dataclasses import replace
    if type(prefix) is not str or re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", prefix) is None:
        raise ValueError("Item preview camera prefix is invalid")
    timestamp = datetime.now(timezone.utc)
    current = _current_or_blank(path, timestamp)
    return _write(path, replace(
        current, saved_at_utc=_canonical_utc_text(timestamp), item_preview_camera_prefix=prefix,
    ))


def write_item_station_state(path: Path, platform_filename: str, bin_filename: str):
    from dataclasses import replace
    platform = _validate_filename(platform_filename, "platform_calibration_", "Platform")
    bin_name = _validate_filename(bin_filename, "bin_teach_", "Bin")
    if platform is None or bin_name is None:
        raise ValueError("Both station and bin filenames are required")
    timestamp = datetime.now(timezone.utc)
    current = _current_or_blank(path, timestamp)
    return _write(path, replace(current, saved_at_utc=_canonical_utc_text(timestamp),
                                item_platform_filename=platform, item_bin_filename=bin_name))
