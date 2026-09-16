"""Deterministic latest station selection; never mix independently taught frames."""

from datetime import datetime, timezone
import re

import yaml

from .bin_teach_core import load_bin_teach_calibration_context
from .item_teach_core import _UniqueKeyLoader
from .platform_teach_core import calibration_directory, load_camera_calibration, load_robot_lan1_ip


ROBOT_CAMERA_PREFIX = "robot_camera"
ROBOT_CAMERA_REFERENCE_FRAME = "Link6"
ROBOT_CAMERA_LINK_FRAME = "robot_camera_link"


def _stamp(path, pattern):
    match = re.fullmatch(pattern, path.name)
    if match is None:
        raise ValueError(f"Noncanonical calibration filename: {path.name}")
    value = match.group(1)
    try:
        instant = datetime.strptime(value, "%Y%m%dT%H%M%S_%fZ").replace(tzinfo=timezone.utc)
    except ValueError as exc:
        raise ValueError(f"Invalid calibration filename timestamp: {path.name}") from exc
    if instant.strftime("%Y%m%dT%H%M%S_%fZ") != value:
        raise ValueError(f"Noncanonical calibration filename timestamp: {path.name}")
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"Calibration must be a regular local file: {path.name}")
    return instant


def _newest(candidates, label):
    if not candidates:
        raise ValueError(f"No {label} calibration found in root calibration/")
    instant = max(stamp for stamp, _ in candidates)
    latest = [path for stamp, path in candidates if stamp == instant]
    if len(latest) != 1:
        raise ValueError(f"Ambiguous latest {label} calibration: {[p.name for p in latest]}")
    return latest[0], instant


def latest_station_calibration(root=None):
    """Latest current-robot platform and latest camera of its prefix, strictly bound.

    Filename UTC timestamps, not mtimes, survive copying/transfer. Never skip an
    invalid selected artifact, substitute another station/prefix or load an older
    compatible pair when a newer same-camera calibration needs platform reteaching.
    Selection is performed at startup/reload, not while a request is in flight.
    """
    directory = calibration_directory(root).resolve()
    robot_ip = load_robot_lan1_ip(root)
    pattern = r"platform_calibration_(\d{8}T\d{6}_\d{6}Z)_" + re.escape(robot_ip) + r"\.yaml"
    platforms = [(_stamp(path, pattern), path) for path in
                 directory.glob(f"platform_calibration_*_{robot_ip}.yaml")]
    platform_path, platform_stamp = _newest(platforms, f"platform for robot {robot_ip}")
    applied = load_bin_teach_calibration_context(platform_path, root=root)
    if applied.platform.created_at_utc != platform_stamp.isoformat(
            timespec="microseconds").replace("+00:00", "Z"):
        raise ValueError(
            "Latest platform filename timestamp conflicts with its saved creation time")
    prefix = applied.camera.settings.camera_prefix
    cameras = []
    for path in sorted(directory.glob("camera_*_calibration_*.yaml")):
        camera_stamp = _stamp(
            path, r"camera_(?:to_hand|on_hand)_calibration_(\d{8}T\d{6}_\d{6}Z)\.yaml")
        try:
            payload = yaml.load(path.read_bytes(), Loader=_UniqueKeyLoader)
            camera_prefix = payload["camera"]["prefix"]
            if type(camera_prefix) is not str or not camera_prefix:
                raise ValueError("Missing camera prefix")
        except (OSError, yaml.YAMLError, KeyError, TypeError, UnicodeError, ValueError) as exc:
            raise ValueError(f"Cannot identify calibration camera: {path.name}: {exc}") from exc
        if camera_prefix == prefix:
            cameras.append((camera_stamp, path))
    camera_path, camera_stamp = _newest(cameras, f"camera {prefix}")
    camera = load_camera_calibration(camera_path, root=root)
    if camera.created_at_utc != camera_stamp.isoformat(
            timespec="microseconds").replace("+00:00", "Z"):
        raise ValueError("Latest camera filename timestamp conflicts with its saved creation time")
    if (camera.path != applied.camera.path or camera.sha256 != applied.camera.sha256
            or camera.settings.camera_prefix != prefix):
        raise ValueError(
            f"Latest camera {camera.path.name} is not the camera bound to latest platform "
            f"{platform_path.name}. Teach a new platform with that camera calibration; "
            "older-camera fallback and mixing transforms are forbidden.")
    return applied


def _validate_robot_camera_binding(camera):
    if (camera.calibration_mode != "camera_on_hand"
            or camera.reference_frame != ROBOT_CAMERA_REFERENCE_FRAME
            or camera.settings.camera_prefix != ROBOT_CAMERA_PREFIX
            or camera.settings.camera_link_frame != ROBOT_CAMERA_LINK_FRAME):
        raise ValueError(
            "Robot-camera calibration must be camera_on_hand "
            "Link6 <- robot_camera_link")
    return camera


def validate_robot_camera_calibration(camera, root=None):
    """Require the selected file still be the unchanged newest robot-camera file."""
    current = latest_robot_camera_calibration(root=root)
    if (current.path != camera.path or current.sha256 != camera.sha256
            or current.created_at_utc != camera.created_at_utc):
        raise ValueError("Selected robot-camera calibration changed; reload calibration")
    return current


def latest_robot_camera_calibration(root=None):
    """Select the newest strict robot_camera calibration by filename UTC.

    Selection never falls back past an invalid newest robot_camera artifact and
    never uses the bin-camera calibration as a substitute.
    """
    directory = calibration_directory(root).resolve()
    candidates = []
    for path in sorted(directory.glob("camera_*_calibration_*.yaml")):
        stamp = _stamp(
            path, r"camera_(?:to_hand|on_hand)_calibration_(\d{8}T\d{6}_\d{6}Z)\.yaml")
        try:
            payload = yaml.load(path.read_bytes(), Loader=_UniqueKeyLoader)
            prefix = payload["camera"]["prefix"]
            if type(prefix) is not str or not prefix:
                raise ValueError("Missing camera prefix")
        except (OSError, yaml.YAMLError, KeyError, TypeError, UnicodeError, ValueError) as exc:
            raise ValueError(f"Cannot identify calibration camera: {path.name}: {exc}") from exc
        if prefix == ROBOT_CAMERA_PREFIX:
            candidates.append((stamp, path))
    path, stamp = _newest(candidates, f"camera {ROBOT_CAMERA_PREFIX}")
    camera = load_camera_calibration(path, root=root)
    expected_created = stamp.isoformat(timespec="microseconds").replace("+00:00", "Z")
    if camera.created_at_utc != expected_created:
        raise ValueError(
            "Latest robot-camera filename timestamp conflicts with its saved creation time")
    return _validate_robot_camera_binding(camera)
