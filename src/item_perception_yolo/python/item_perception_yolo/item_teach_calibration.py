"""Explicit Item Teach calibration selection and atomic root .env persistence."""

import os
from pathlib import Path
import re
import stat
import tempfile

from .bin_teach_core import load_bin_teach_calibration_context
from .platform_teach_core import (
    _parse_env_file, calibration_directory, load_camera_calibration, load_robot_lan1_ip,
    workspace_root)
from .station_calibration import _validate_robot_camera_binding


CALIBRATION_ENV_KEYS = (
    "ITEM_TEACH_PLATFORM_CALIBRATION",
    "ITEM_TEACH_BIN_CAMERA_CALIBRATION",
    "ITEM_TEACH_ROBOT_CAMERA_CALIBRATION",
)


def _root(root):
    return workspace_root() if root is None else Path(root).resolve()


def _filename(value):
    if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_. -]*\.yaml", value) is None:
        raise ValueError("Calibration selection must be a YAML filename inside calibration/")
    return value


def saved_calibration_paths(root=None):
    root = _root(root)
    load_robot_lan1_ip(root)  # Require the exact canonical .env key set and robot identity.
    values = _parse_env_file(root / ".env")
    if any(key not in values for key in CALIBRATION_ENV_KEYS):
        raise ValueError("Root .env is missing required Item Teach calibration selections")
    names = tuple(values[key] for key in CALIBRATION_ENV_KEYS)
    if not any(names):
        return None
    if not all(names):
        raise ValueError("Item Teach calibration selections must be all empty or all selected")
    return tuple(calibration_directory(root) / _filename(name) for name in names)


def _selected_path(path, root):
    candidate = Path(path).expanduser()
    _filename(candidate.name)
    if candidate.is_symlink() or not candidate.is_file():
        raise ValueError(f"Calibration must be a regular local file: {candidate}")
    candidate = candidate.resolve()
    if candidate.parent != calibration_directory(root).resolve():
        raise ValueError("Select calibration files directly inside root calibration/")
    return candidate


def selected_robot_camera(path, root=None):
    root = _root(root)
    return _validate_robot_camera_binding(
        load_camera_calibration(_selected_path(path, root), root=root))


def validate_selected_robot_camera(camera, root=None):
    current = selected_robot_camera(camera.path, root=root)
    if current.sha256 != camera.sha256:
        raise ValueError("Selected robot-camera calibration changed; reload calibration")
    return current


def load_calibration_selection(platform_path, camera_path, robot_camera_path, root=None):
    root = _root(root)
    paths = tuple(_selected_path(path, root)
                  for path in (platform_path, camera_path, robot_camera_path))
    applied = load_bin_teach_calibration_context(paths[0], root=root)
    if applied.camera.path != paths[1]:
        raise ValueError(
            f"Selected bin camera must be {applied.camera.path.name}, the calibration bound "
            "to this platform. Teach a new platform to use a different camera calibration.")
    return applied, selected_robot_camera(paths[2], root=root)


def save_calibration_selection(platform_path, camera_path, robot_camera_path, root=None):
    root = _root(root)
    env_path = root / ".env"
    original = env_path.read_bytes()
    # Validate all artifacts and the existing config before replacing any .env bytes.
    saved_calibration_paths(root)
    applied, robot_camera = load_calibration_selection(
        platform_path, camera_path, robot_camera_path, root=root)
    names = (applied.platform.path.name, applied.camera.path.name, robot_camera.path.name)
    updates = dict(zip(CALIBRATION_ENV_KEYS, names))
    lines = original.decode("utf-8").splitlines(keepends=True)
    output = "".join(f"{line.split('=', 1)[0]}={updates[line.split('=', 1)[0]]}\n"
                     if line.split("=", 1)[0] in updates else line for line in lines)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=root,
                                         prefix=".env.item_teach.", delete=False) as handle:
            temporary = Path(handle.name)
            handle.write(output)
            handle.flush()
            os.fsync(handle.fileno())
        temporary.chmod(stat.S_IMODE(env_path.stat().st_mode))
        if env_path.read_bytes() != original:
            raise ValueError("Root .env changed while saving calibration; load again")
        os.replace(temporary, env_path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return applied, robot_camera
