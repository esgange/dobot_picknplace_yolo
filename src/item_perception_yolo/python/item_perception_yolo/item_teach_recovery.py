"""GUI-only recovery of untrusted item fields, never a production profile reader."""

import copy
import hashlib
import re
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from . import item_teach_core as core


@dataclass
class RecoveryDraft:
    values: dict = field(default_factory=dict)
    home: dict | None = None
    model_path: Path | None = None
    model_sha256: str | None = None
    digest: str | None = None
    issues: list = field(default_factory=list)


def recover_item_fields(path, *, root=None):
    """Return independently validated fields or None; never write/execute/arm anything.

    Ambiguous YAML syntax/duplicate keys yields a blank draft, not guessed parsing.
    The sole old-name mapping is explicit GUI recovery of retry_limit in schemas 1–3.
    Runtime load_item_profile remains strict and never calls this function.
    """
    path = Path(path).expanduser().resolve()
    if path.parent != core.item_directory(root).resolve() or path.suffix != ".yaml":
        raise ValueError("Item YAML must be directly in offline_teach/item_teach/")
    draft = RecoveryDraft()
    try:
        content = path.read_bytes()
        draft.digest = hashlib.sha256(content).hexdigest()
        payload = yaml.load(content, Loader=core._UniqueKeyLoader)
    except (OSError, ValueError, yaml.YAMLError, TypeError, UnicodeError) as exc:
        draft.issues.append(f"Cannot interpret YAML; all fields cleared: {exc}")
        return draft
    if (type(payload) is not dict or payload.get("artifact_type") != "item_teach"
            or type(payload.get("schema_version")) is not int
            or payload["schema_version"] not in (1, 2, 3, 4, 5, 6)):
        draft.issues.append("Unrecognized item format/schema; all fields cleared")
        return draft

    def group(name):
        value = payload.get(name)
        return value if type(value) is dict else {}

    def accept(key, value, valid, reason="missing or invalid"):
        if valid:
            draft.values[key] = copy.deepcopy(value)
        else:
            draft.values[key] = None
            draft.issues.append(f"{key}: {reason}; cleared")

    def number(section, key, *, integer=False, low=0., high=None, unit=None):
        value = group(section).get(key)
        try:
            if unit and group("units").get(unit[0]) != unit[1]:
                raise ValueError("missing or ambiguous units")
            if integer:
                core._integer(value, key, low=low, high=high)
            else:
                core._number(value, key, low=low, high=high)
        except ValueError as exc:
            accept(key, value, False, str(exc))
        else:
            accept(key, value, True)

    name = group("item").get("name")
    accept("name", name, type(name) is str
           and re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}", name) is not None)
    task = group("model").get("declared_task")
    accept("model_task", task, type(task) is str and task in core.MODEL_TASKS)
    source = payload.get("geometry_source")
    accept("geometry_source", source, type(source) is str and source in ("mask", "obb", "none"))
    for key in core.MOTION_FIELDS:
        if key == "retract_height" and payload["schema_version"] < 6:
            accept(key, None, False, "old height was relative to pick; now extra above pre-pick")
        else:
            number("motion", key, unit=("distance", "mm"))
    for key in core.SPEED_FIELDS:
        number("speed", key, integer=True, low=1, high=100, unit=("speed", "%"))
        value = group("acceleration").get(key)
        valid = (group("units").get("acceleration") == "%"
                 and type(value) is int and 1 <= value <= 100)
        accept(f"acceleration_{key}", value, valid, "missing/invalid acceleration percentage")
    number("timing", "pick_settling", unit=("time", "s"))
    for key in core.GRIPPER_FIELDS:
        value = group("gripper").get(key)
        accept(key, value, type(value) is bool)
    for key in core.GEOMETRY_FIELDS:
        number("geometry", key, low=0 if key == "tolerance" else 0.000001,
               unit=("distance", "mm"))
    for key in ("confidence", "iou"):
        number("yolo", key, high=1)
    number("yolo", "max_detections", integer=True, low=1, high=1000)
    number("yolo", "image_size", integer=True, low=32, high=8192)
    size = draft.values["image_size"]
    if size is not None and size % 32:
        accept("image_size", size, False, "must be a multiple of 32")
    if draft.values["image_size"] is None:
        draft.issues.append("Browse a model explicitly to start with the new-profile 640 px size")
    ids = group("yolo").get("class_ids")
    valid_ids = (type(ids) is list and bool(ids)
                 and all(type(i) is int and i >= 0 for i in ids))
    accept("class_ids", ids, valid_ids and len(ids) == len(set(ids)))

    retry = group("retry")
    if payload["schema_version"] <= 3 and set(retry) == {"retry_limit"}:
        retry = {"pose_candidates": retry["retry_limit"]}
        draft.issues.append("Recovered retry_limit as pose_candidates; review the requested count")
    count = retry.get("pose_candidates")
    accept("pose_candidates", count, set(retry) == {"pose_candidates"}
           and type(count) is int and 1 <= count <= 1000)
    for key in core.QUALITY_DEFAULTS:
        if key == "minimum_depth_samples":
            number("quality", key, integer=True, low=3, high=100000)
        else:
            high = (1 if key == "minimum_depth_fraction" else
                    30 if key in ("request_timeout_sec", "result_max_age_sec") else None)
            # Quality field names explicitly encode seconds/mm; require corresponding units.
            unit = (("distance", "mm") if key.endswith("_mm") else
                    ("time", "s") if key.endswith("_sec") else None)
            number("quality", key, low=0.000001, high=high, unit=unit)
    for first, second, invalid in (
        ("height", "width", lambda a, b: a < b),
        ("depth_min_mm", "depth_max_mm", lambda a, b: a >= b),
        ("sync_tolerance_sec", "input_max_age_sec", lambda a, b: a > b),
        ("pose_candidates", "max_detections", lambda a, b: a > b),
    ):
        a, b = draft.values.get(first), draft.values.get(second)
        if a is not None and b is not None and invalid(a, b):
            for key in (first, second):
                accept(key, None, False, f"conflict between {first} and {second}")
    try:
        if group("units").get("home_joints") != "rad":
            raise ValueError("home joint units must be rad")
        core.validate_home(payload.get("home"))
        draft.home = copy.deepcopy(payload["home"])
    except (ValueError, TypeError, KeyError) as exc:
        draft.issues.append(f"Home cleared; record all six joints again: {exc}")

    model = group("model")
    try:
        model_path = path.with_suffix(".pt")
        digest = model.get("sha256")
        if (model.get("filename") != model_path.name
                or model_path.resolve().parent != path.parent
                or type(digest) is not str or re.fullmatch(r"[0-9a-f]{64}", digest) is None
                or not model_path.is_file() or model_path.stat().st_size == 0
                or core.file_sha256(model_path) != digest):
            raise ValueError("paired .pt filename/hash/file is missing or invalid")
        draft.model_path, draft.model_sha256 = model_path, digest
    except (ValueError, OSError) as exc:
        draft.issues.append(f"Model cleared; browse a trusted .pt explicitly: {exc}")
    return draft
