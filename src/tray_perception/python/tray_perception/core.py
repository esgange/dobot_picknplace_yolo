"""Strict tray artifacts and local GUI state; no native imports or robot commands."""

import copy
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
import threading

import numpy as np
import yaml

from item_perception_yolo.item_teach_core import (
    _fields, _number, _timestamp, _UniqueKeyLoader, file_sha256, load_item_profile,
    utc_now, validate_home, validate_yolo_settings)
from item_perception_yolo.platform_teach_core import _validate_rigid_transform


ORIGIN_CONVENTION = "nearest_base_corner_v1"
MAX_PLANE_ERROR_MM = 5.0


def tray_directory(root):
    return Path(root) / "offline_teach/tray_teach"


def validate_settings(settings):
    _fields(settings, ("name", "model_task", "geometry_source", "geometry", "yolo"),
            "Tray settings")
    if type(settings["name"]) is not str or not re.fullmatch(
            r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", settings["name"]):
        raise ValueError("Tray name must contain 1–64 letters, digits, underscores or hyphens")
    validate_yolo_settings(settings["model_task"], settings["yolo"])
    if settings["geometry_source"] != {"segment": "mask", "obb": "obb", "detect": "none"}[
            settings["model_task"]]:
        raise ValueError("Geometry must match the model's mask/OBB output")
    validate_geometry(settings["geometry"])


def validate_geometry(geometry):
    _fields(geometry, ("length_mm", "width_mm", "tolerance_mm"), "Tray geometry")
    for key, value in geometry.items():
        _number(value, key, low=0. if key == "tolerance_mm" else .001)
    if geometry["length_mm"] < geometry["width_mm"]:
        raise ValueError("Length is the long side and must be at least the width")
    if geometry["tolerance_mm"] >= geometry["width_mm"]:
        raise ValueError("Tolerance must be smaller than tray width")


def validate_preview(settings):
    """Preview needs model settings, not a complete production teach profile."""
    _fields(settings, ("model_task", "geometry_source", "geometry", "yolo",
                       "accepted_class_ids"), "Tray preview")
    validate_yolo_settings(settings["model_task"], settings["yolo"])
    if settings["geometry_source"] != {"segment": "mask", "obb": "obb", "detect": "none"}[
            settings["model_task"]]:
        raise ValueError("Geometry must match the loaded model")
    ids = settings["accepted_class_ids"]
    if (type(ids) is not list or any(
            type(i) is not int or i not in settings["yolo"]["class_ids"] for i in ids)
            or len(ids) != len(set(ids))):
        raise ValueError("Invalid accepted tray classes")
    if settings["geometry"] is not None:
        validate_geometry(settings["geometry"])


def validate_plane(plane):
    _fields(plane, ("corners_base_m", "pixels", "base_from_plane", "max_error_mm",
                    "source_stamp_ns", "depth_stamp_ns", "camera", "depth_camera"), "Tray plane")
    matrix = _validate_rigid_transform(plane["base_from_plane"], "base_from_plane")
    corners = np.asarray(plane["corners_base_m"], dtype=float)
    pixels = np.asarray(plane["pixels"], dtype=float)
    if corners.shape != (4, 3) or pixels.shape != (4, 2) or not (
            np.isfinite(corners).all() and np.isfinite(pixels).all()):
        raise ValueError("Plane requires four finite base-frame corners and four image points")
    local = (corners - matrix[:3, 3]) @ matrix[:3, :3]
    error = float(np.abs(local[:, 2]).max() * 1000)
    _number(plane["max_error_mm"], "Plane fit error", high=MAX_PLANE_ERROR_MM)
    if not np.isclose(error, plane["max_error_mm"], atol=1e-6):
        raise ValueError("Saved plane fit evidence does not match its corners")
    if np.linalg.svd(local[:, :2] - local[:, :2].mean(axis=0))[1][-1] < .001:
        raise ValueError("Plane corners are degenerate or collinear")
    if np.linalg.norm(local[0, :2]) > 1e-6:
        raise ValueError("Reference plane origin must project the taught nearest-base corner")
    for key in ("source_stamp_ns", "depth_stamp_ns"):
        if type(plane[key]) is not int or plane[key] <= 0:
            raise ValueError("Plane timestamps must be positive integer nanoseconds")
    for key in ("camera", "depth_camera"):
        camera = plane[key]
        _fields(camera, ("width", "height", "k", "d", "distortion_model"), key)
        if (type(camera["width"]) is not int or type(camera["height"]) is not int
                or not 0 < camera["width"] <= 4096 or not 0 < camera["height"] <= 4096
                or len(camera["k"]) != 9 or len(camera["d"]) not in (4, 5, 8, 12, 14)
                or camera["distortion_model"] not in ("plumb_bob", "rational_polynomial")
                or not np.isfinite(camera["k"] + camera["d"]).all()
                or camera["k"][0] <= 0 or camera["k"][4] <= 0):
            raise ValueError("Invalid plane camera evidence")


def copy_teach_position(path, root):
    profile, _ = load_item_profile(path, root=root)
    return copy.deepcopy(profile["home"])


def validate_profile(profile):
    _fields(profile, ("schema_version", "artifact_type", "created_at_utc", "settings", "model",
                      "camera_calibration", "tray_teach_position", "reference_plane",
                      "origin_convention", "frame_id", "units"), "Tray teach artifact")
    if type(profile["schema_version"]) is not int or profile["schema_version"] != 1:
        raise ValueError("Tray teach schema_version must be exactly 1; no migration")
    if profile["artifact_type"] != "tray_teach" or profile["frame_id"] != "base_link":
        raise ValueError("Expected base_link tray_teach artifact")
    if profile["origin_convention"] != ORIGIN_CONVENTION or profile["units"] != {
            "geometry": "mm", "plane": "m", "tray_teach_position": "rad"}:
        raise ValueError("Unsupported tray origin convention or units")
    _timestamp(profile["created_at_utc"], "created_at_utc")
    validate_settings(profile["settings"])
    if profile["settings"]["geometry_source"] == "none":
        raise ValueError("Saving a tray requires a segmentation or OBB model")
    validate_home(profile["tray_teach_position"])
    validate_plane(profile["reference_plane"])
    for key, suffix in (("model", ".pt"), ("camera_calibration", ".yaml")):
        value = profile[key]
        _fields(value, ("filename", "sha256"), key)
        name = value["filename"]
        if (type(name) is not str or Path(name).name != name or not name.endswith(suffix)
                or type(value["sha256"]) is not str
                or re.fullmatch(r"[0-9a-f]{64}", value["sha256"]) is None):
            raise ValueError(f"Invalid {key} filename or SHA-256")


def load_profile(path, root, *, deployment=False):
    original = Path(path).expanduser().absolute()
    directory = Path(root) / "runtime_teach" if deployment else tray_directory(root)
    if original.is_symlink() or directory.is_symlink():
        raise ValueError("Tray teaching files must be regular files, not symlinks")
    path = original.resolve()
    if path.parent != directory.resolve() or path.suffix != ".yaml":
        location = "runtime_teach/" if deployment else "offline_teach/tray_teach/"
        raise ValueError(f"Select a YAML directly inside {location}")
    try:
        profile = yaml.load(path.read_bytes(), Loader=_UniqueKeyLoader)
        validate_profile(profile)
    except (yaml.YAMLError, TypeError, KeyError, OSError) as exc:
        raise ValueError(f"Invalid tray teach file: {exc}") from exc
    model = path.with_suffix(".pt")
    if (model.name != profile["model"]["filename"] or model.is_symlink()
            or not model.is_file() or not model.stat().st_size
            or file_sha256(model) != profile["model"]["sha256"]):
        raise ValueError("Tray YAML requires its unchanged same-stem .pt model")
    return profile


def save_profile(settings, position, plane, camera, model, expected_sha256, root):
    validate_settings(settings)
    validate_home(position)
    validate_plane(plane)
    stamp = utc_now()
    token = stamp.replace("-", "").replace(":", "").replace(".", "_")
    directory = tray_directory(root)
    if directory.is_symlink():
        raise ValueError("Tray teach directory must not be a symlink")
    directory.mkdir(parents=True, exist_ok=True)
    output = directory / f"tray_teach_{settings['name']}_{token}.yaml"
    source = Path(model).resolve(strict=True)
    if source.suffix != ".pt" or not source.stat().st_size:
        raise ValueError("A non-empty local .pt model is required")
    if file_sha256(source) != expected_sha256 or file_sha256(camera.path) != camera.sha256:
        raise ValueError("Model or camera calibration changed; reload before saving")
    profile = copy.deepcopy({
        "schema_version": 1, "artifact_type": "tray_teach", "created_at_utc": stamp,
        "settings": settings, "tray_teach_position": position, "reference_plane": plane,
        "camera_calibration": {"filename": camera.path.name, "sha256": camera.sha256},
        "model": {"filename": output.with_suffix(".pt").name, "sha256": expected_sha256},
        "origin_convention": ORIGIN_CONVENTION, "frame_id": "base_link",
        "units": {"geometry": "mm", "plane": "m", "tray_teach_position": "rad"}})
    validate_profile(profile)
    with tempfile.TemporaryDirectory(prefix=".tray_teach_", dir=directory) as temporary:
        weights, artifact = Path(temporary) / "model.pt", Path(temporary) / "profile.yaml"
        shutil.copyfile(source, weights)
        if (file_sha256(weights) != expected_sha256 or file_sha256(source) != expected_sha256
                or file_sha256(camera.path) != camera.sha256):
            raise ValueError("Model or calibration changed while saving")
        with weights.open("rb") as stream:
            os.fsync(stream.fileno())
        with artifact.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(profile, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        # Publish YAML last; a complete pair never overwrites an existing artifact.
        os.link(weights, output.with_suffix(".pt"))
        try:
            os.link(artifact, output)
        except BaseException:
            output.with_suffix(".pt").unlink()
            raise
    return output


class EventLogger:
    def __init__(self, root, node="tray_teach"):
        self.path = Path(root) / "logs/tray_perception/events.jsonl"
        self.node = node
        self.lock = threading.Lock()

    def record(self, level, event, message, **fields):
        record = {"timestamp_utc": utc_now(), "package": "tray_perception", "node": self.node,
                  "level": level, "event": event, "message": str(message), **fields}
        with self.lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            count = len(self.path.read_text().splitlines()) if self.path.exists() else 0
            with self.path.open("w" if count >= 1000 else "a") as stream:
                stream.write(json.dumps(record, allow_nan=False) + "\n")


def session_path(root):
    return Path(root) / "logs/tray_perception/last_session.json"


def validate_session(state):
    _fields(state, ("schema_version", "profile_filename", "camera_filename", "item_filename",
                    "model_path", "settings"), "Tray UI state")
    if type(state["schema_version"]) is not int or state["schema_version"] != 1:
        raise ValueError("Tray UI schema_version must be exactly 1")
    for key in ("profile_filename", "camera_filename", "item_filename", "model_path"):
        if type(state[key]) is not str:
            raise ValueError(f"Invalid tray UI field: {key}")
        if key != "model_path" and state[key] and Path(state[key]).name != state[key]:
            raise ValueError(f"Tray UI {key} must be a filename")
    validate_settings(state["settings"])


def read_session(root):
    path = session_path(root)
    if not path.exists():
        return None
    state = json.loads(path.read_text())
    validate_session(state)
    return state


def write_session(root, state):
    validate_session(state)
    path = session_path(root)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        json.dump(state, stream, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    try:
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
