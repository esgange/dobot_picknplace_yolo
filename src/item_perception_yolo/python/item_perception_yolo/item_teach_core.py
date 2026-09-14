"""Strict stage-one item profiles: portable model pairing and recorded home joints.

This module deliberately never imports torch/cv2 or deserializes model weights.
File integrity is not a claim of model compatibility or permission to move.
"""

import copy
import hashlib
import ipaddress
import math
import os
import re
import shutil
import tempfile
from datetime import datetime, timezone
from pathlib import Path

import yaml

from camera_calibration_gui.calibration_core import workspace_root


ITEM_SCHEMA_VERSION = 3
JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
MODEL_TASKS = ("detect", "segment", "obb")
MOTION_FIELDS = ("standoff_height", "zheight_offset", "prepick_height", "retract_height")
GRIPPER_FIELDS = ("use_grip", "grip_onpick")
YOLO_FIELDS = ("confidence", "iou", "image_size", "max_detections", "class_ids")
NEW_PROFILE_IMAGE_SIZE = 640  # Not an operator field; loaded profiles retain their exact value.
GEOMETRY_FIELDS = ("height", "width", "tolerance", "pickdepth_radius")
DEFAULT_PICKDEPTH_DIAMETER_MM = 30.0
QUALITY_DEFAULTS = {
    "input_max_age_sec": 0.5, "sync_tolerance_sec": 0.1,
    "robot_tf_max_age_sec": 1.0, "request_timeout_sec": 10.0,
    "result_max_age_sec": 2.0, "minimum_depth_samples": 30,
    "minimum_depth_fraction": 0.5, "depth_min_mm": 200.0, "depth_max_mm": 1000.0,
}
IO_MAP = {
    "exhaust_do": 1, "finger_close_do": 2, "suction_do": 13,
    "finger_open_do": 14, "suction_detect_di": 1, "finger_full_open_di": 12,
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds").replace("+00:00", "Z")


def item_directory(root: Path | None = None) -> Path:
    project_root = workspace_root() if root is None else Path(root).resolve()
    return project_root / "offline_teach/item_teach"


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _fields(value, names, label):
    if type(value) is not dict or set(value) != set(names):
        raise ValueError(f"{label} must contain exactly: {', '.join(names)}")


def _number(value, label, *, low=0.0, high=None):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{label} must be finite and numeric")
    if value < low or (high is not None and value > high):
        raise ValueError(f"{label} is outside its allowed range")


def _integer(value, label, *, low=1, high=None):
    if type(value) is not int or value < low or (high is not None and value > high):
        raise ValueError(f"{label} must be an integer from {low}" + (
            "" if high is None else f" to {high}"
        ))


def _timestamp(value, label):
    if type(value) is not str or not value.endswith("Z"):
        raise ValueError(f"{label} must be a UTC timestamp")
    try:
        parsed = datetime.fromisoformat(value[:-1] + "+00:00")
    except ValueError as exc:
        raise ValueError(f"{label} must be a UTC timestamp") from exc
    if parsed.isoformat(timespec="microseconds").replace("+00:00", "Z") != value:
        raise ValueError(f"{label} must be canonical UTC with microseconds")


def validate_settings(settings):
    _fields(settings, ("item", "model_task", "motion", "timing", "gripper", "retry", "yolo",
                       "geometry", "geometry_source", "quality"),
            "Item settings")
    _fields(settings["item"], ("name",), "item")
    name = settings["item"]["name"]
    if type(name) is not str or re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}", name) is None:
        raise ValueError("Item name must be 1-64 letters, digits, underscores or hyphens")
    if settings["model_task"] not in MODEL_TASKS:
        raise ValueError("Model task must be exactly detect, segment or obb")
    _fields(settings["motion"], MOTION_FIELDS, "motion")
    for field in MOTION_FIELDS:
        _number(settings["motion"][field], field)
    _fields(settings["timing"], ("pick_settling",), "timing")
    _number(settings["timing"]["pick_settling"], "pick_settling")
    _fields(settings["gripper"], GRIPPER_FIELDS, "gripper")
    if any(type(settings["gripper"][key]) is not bool for key in GRIPPER_FIELDS):
        raise ValueError("Gripper flags must be booleans")
    _fields(settings["retry"], ("retry_limit",), "retry")
    _integer(settings["retry"]["retry_limit"], "retry_limit", high=1000)
    _fields(settings["geometry"], GEOMETRY_FIELDS, "geometry")
    for key in GEOMETRY_FIELDS:
        _number(settings["geometry"][key], key)
        if key != "tolerance" and settings["geometry"][key] <= 0:
            raise ValueError(f"{key} must be greater than zero millimetres")
    validate_yolo_settings(settings["model_task"], settings["yolo"])
    if settings["geometry"]["height"] < settings["geometry"]["width"]:
        raise ValueError("height is the long side and must be >= width")
    if settings["geometry_source"] not in ("mask", "obb", "none"):
        raise ValueError("geometry_source must be mask, obb or none (preview only)")
    validate_quality(settings["quality"])
    if settings["retry"]["retry_limit"] > settings["yolo"]["max_detections"]:
        raise ValueError("retry_limit cannot exceed max_detections")


def validate_quality(quality):
    _fields(quality, QUALITY_DEFAULTS, "quality")
    for key in QUALITY_DEFAULTS:
        _number(quality[key], key, low=0.000001)
    _integer(quality["minimum_depth_samples"], "minimum_depth_samples", low=3, high=100000)
    if quality["minimum_depth_fraction"] > 1:
        raise ValueError("minimum_depth_fraction must be <= 1")
    if quality["depth_min_mm"] >= quality["depth_max_mm"]:
        raise ValueError("depth_min_mm must be smaller than depth_max_mm")
    if quality["sync_tolerance_sec"] > quality["input_max_age_sec"]:
        raise ValueError("Synchronization tolerance cannot exceed input freshness")
    if quality["request_timeout_sec"] > 30 or quality["result_max_age_sec"] > 30:
        raise ValueError("Request/result deadline cannot exceed 30 seconds")


def detection_settings(settings):
    return copy.deepcopy({key: settings[key] for key in
                          ("model_task", "yolo", "geometry", "geometry_source", "quality")})


def validate_detection_settings(settings, *, geometry_required):
    validate_yolo_settings(settings["model_task"], settings["yolo"])
    validate_quality(settings["quality"])
    if geometry_required:
        if settings["geometry_source"] not in ("mask", "obb"):
            raise ValueError("Mask or OBB geometry is required")
        _fields(settings["geometry"], GEOMETRY_FIELDS, "geometry")
        for key, value in settings["geometry"].items():
            _number(value, key, low=0 if key == "tolerance" else 0.000001)
        if settings["geometry"]["height"] < settings["geometry"]["width"]:
            raise ValueError("height is the long side and must be >= width")


def validate_yolo_settings(task, yolo):
    if task not in MODEL_TASKS:
        raise ValueError("Model task must be exactly detect, segment or obb")
    _fields(yolo, YOLO_FIELDS, "yolo")
    for key in ("confidence", "iou"):
        _number(yolo[key], key, high=1.0)
    _integer(yolo["image_size"], "image_size", low=32, high=8192)
    if yolo["image_size"] % 32:
        raise ValueError("image_size must be a multiple of 32")
    _integer(yolo["max_detections"], "max_detections", high=1000)
    ids = yolo["class_ids"]
    if type(ids) is not list or not ids:
        raise ValueError("class_ids must be an explicit non-empty list")
    for value in ids:
        _integer(value, "class_id", low=0)
    if len(ids) != len(set(ids)):
        raise ValueError("class_ids must be unique")


def validate_home(home):
    _fields(home, ("robot_lan1_ip", "joint_names", "positions_rad", "recorded_at_utc",
                   "feedback_stamp", "source_topic", "publisher_node"), "home")
    try:
        address = ipaddress.ip_address(home["robot_lan1_ip"])
    except (TypeError, ValueError) as exc:
        raise ValueError("Home robot_lan1_ip must be IPv4") from exc
    if address.version != 4 or str(address) != home["robot_lan1_ip"]:
        raise ValueError("Home robot_lan1_ip must be canonical IPv4")
    if home["joint_names"] != list(JOINT_NAMES):
        raise ValueError("Home requires canonical joint1 through joint6")
    positions = home["positions_rad"]
    if type(positions) is not list or len(positions) != 6:
        raise ValueError("Home requires six recorded joint positions")
    for value in positions:
        _number(value, "home position", low=-math.inf)
    _timestamp(home["recorded_at_utc"], "Home recorded_at_utc")
    _fields(home["feedback_stamp"], ("sec", "nanosec"), "feedback_stamp")
    sec, nanosec = home["feedback_stamp"]["sec"], home["feedback_stamp"]["nanosec"]
    _integer(sec, "feedback sec", low=0)
    _integer(nanosec, "feedback nanosec", low=0, high=999999999)
    if sec == 0 and nanosec == 0:
        raise ValueError("Home feedback timestamp must be non-zero")
    if home["source_topic"] != "/joint_states":
        raise ValueError("Home source must be /joint_states")
    if (type(home["publisher_node"]) is not str
            or re.fullmatch(r"/[A-Za-z_][A-Za-z_0-9]*", home["publisher_node"]) is None):
        raise ValueError("Home publisher must be one root-namespace node")


def record_home(names, positions, sec, nanosec, *, now_ns, robot_ip, publisher):
    if len(names) != 6 or set(names) != set(JOINT_NAMES) or len(positions) != 6:
        raise ValueError(
            "Joint feedback must contain exactly joint1 through joint6 and six positions"
        )
    home = {
        "robot_lan1_ip": robot_ip,
        "joint_names": list(JOINT_NAMES),
        "positions_rad": [positions[names.index(name)] for name in JOINT_NAMES],
        "recorded_at_utc": utc_now(),
        "feedback_stamp": {"sec": sec, "nanosec": nanosec},
        "source_topic": "/joint_states",
        "publisher_node": publisher,
    }
    validate_home(home)
    age = (now_ns - (sec * 1_000_000_000 + nanosec)) / 1e9
    if not 0.0 <= age <= 1.0:
        raise ValueError(f"Home joint feedback is stale or future-dated: age={age:.3f}s")
    return home


def settings_from_profile(profile):
    return copy.deepcopy({
        "item": profile["item"], "model_task": profile["model"]["declared_task"],
        **{key: profile[key] for key in ("motion", "timing", "gripper", "retry", "yolo",
                                         "geometry", "geometry_source", "quality")},
    })


def validate_profile(profile):
    _fields(profile, ("schema_version", "artifact_type", "created_at_utc", "item", "model",
                      "units", "home", "motion", "timing", "gripper", "retry", "yolo",
                      "geometry", "geometry_source", "quality", "controller_contract"),
            "Item teach artifact")
    if (type(profile["schema_version"]) is not int
            or profile["schema_version"] != ITEM_SCHEMA_VERSION):
        raise ValueError("Item teach schema_version must be exactly 3; no compatibility reader")
    if profile["artifact_type"] != "item_teach":
        raise ValueError("Expected item_teach artifact")
    _timestamp(profile["created_at_utc"], "created_at_utc")
    _fields(profile["model"], ("filename", "sha256", "declared_task", "verification"), "model")
    model = profile["model"]
    if (type(model["filename"]) is not str or Path(model["filename"]).name != model["filename"]
            or not model["filename"].endswith(".pt")):
        raise ValueError("Model filename must name the paired local .pt file")
    if type(model["sha256"]) is not str or re.fullmatch(r"[0-9a-f]{64}", model["sha256"]) is None:
        raise ValueError("Model SHA-256 must be exactly 64 lowercase hexadecimal characters")
    if model["verification"] != "file_sha256_only":
        raise ValueError("Stage-one model verification must be file_sha256_only")
    if profile["units"] != {"distance": "mm", "time": "s", "home_joints": "rad"}:
        raise ValueError("Item units must be mm, seconds, and radians for home joints")
    contract = profile["controller_contract"]
    if type(contract) is not dict or contract.get("motion_enabled") is not False or contract != {
        "stage": "profile_validation_only", "motion_enabled": False,
        "target_link": "Link6", "routine": "vertical_pick_place",
        "home_usage": "start_and_end", "io_map": IO_MAP,
    }:
        raise ValueError("Controller contract must be the exact non-executing stage-one contract")
    validate_home(profile["home"])
    validate_settings(settings_from_profile(profile))


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise ValueError(f"Duplicate YAML key: {key}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping)


def load_item_profile(path: Path, *, root: Path | None = None):
    path = Path(path).expanduser().resolve()
    directory = item_directory(root).resolve()
    if path.parent != directory or path.suffix != ".yaml":
        raise ValueError("Item YAML must be directly in offline_teach/item_teach/")
    try:
        content = path.read_bytes()
        profile = yaml.load(content, Loader=_UniqueKeyLoader)
        validate_profile(profile)
    except (OSError, yaml.YAMLError, TypeError, KeyError, UnicodeError) as exc:
        raise ValueError(f"Cannot load item teach: {exc}") from exc
    model_path = path.parent / profile["model"]["filename"]
    if model_path.name != path.with_suffix(".pt").name or model_path.resolve().parent != directory:
        raise ValueError("Item YAML and its local .pt copy must have the same stem")
    if not model_path.is_file() or model_path.stat().st_size == 0:
        raise ValueError("Paired model file is missing or empty")
    if file_sha256(model_path) != profile["model"]["sha256"]:
        raise ValueError("Paired model SHA-256 mismatch")
    return profile, hashlib.sha256(content).hexdigest()


def save_item_profile(settings, home, model_source: Path, *, root: Path | None = None):
    validate_settings(settings)
    validate_home(home)
    source = Path(model_source).expanduser().resolve()
    if source.suffix != ".pt" or not source.is_file() or source.stat().st_size == 0:
        raise ValueError("Select an existing non-empty .pt file anywhere on this PC")
    source_digest = file_sha256(source)
    directory = item_directory(root)
    directory.mkdir(parents=True, exist_ok=True)
    timestamp = utc_now()
    stamp = datetime.fromisoformat(timestamp[:-1] + "+00:00").strftime("%Y%m%dT%H%M%S_%fZ")
    output = directory / f"item_teach_{settings['item']['name']}_{stamp}.yaml"
    model_output = output.with_suffix(".pt")
    profile = {
        "schema_version": ITEM_SCHEMA_VERSION, "artifact_type": "item_teach",
        "created_at_utc": timestamp, "item": copy.deepcopy(settings["item"]),
        "model": {"filename": model_output.name, "sha256": source_digest,
                  "declared_task": settings["model_task"], "verification": "file_sha256_only"},
        "units": {"distance": "mm", "time": "s", "home_joints": "rad"},
        "home": copy.deepcopy(home),
        **{key: copy.deepcopy(settings[key])
           for key in ("motion", "timing", "gripper", "retry", "yolo", "geometry",
                       "geometry_source", "quality")},
        "controller_contract": {
            "stage": "profile_validation_only", "motion_enabled": False,
            "target_link": "Link6", "routine": "vertical_pick_place",
            "home_usage": "start_and_end", "io_map": dict(IO_MAP),
        },
    }
    validate_profile(profile)
    with tempfile.TemporaryDirectory(prefix=".item_teach_", dir=directory) as temporary:
        staged_model = Path(temporary) / "model.pt"
        staged_yaml = Path(temporary) / "profile.yaml"
        shutil.copyfile(source, staged_model)
        if file_sha256(staged_model) != source_digest or file_sha256(source) != source_digest:
            raise ValueError("Selected model changed during copy; nothing was saved")
        with staged_model.open("rb") as stream:
            os.fsync(stream.fileno())
        with staged_yaml.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(profile, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        # Exclusive links avoid overwriting an existing pair; YAML is the commit marker.
        os.link(staged_model, model_output)
        try:
            os.link(staged_yaml, output)
        except BaseException:
            if model_output.exists() and os.path.samefile(staged_model, model_output):
                model_output.unlink()
            raise
    return output, profile
