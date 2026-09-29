"""Editable teaching documents; incomplete drafts never satisfy runtime validation."""

import copy
from dataclasses import dataclass
import hashlib
import os
from pathlib import Path
import re
import shutil
import tempfile
import zipfile

import yaml

from item_perception_yolo.item_teach_core import (
    _fields, _sync_directory, _timestamp, _UniqueKeyLoader, file_sha256, utc_now,
    validate_home)
from .core import (
    load_profile, ORIGIN_CONVENTION, tray_directory, validate_plane, validate_profile,
    validate_session, validate_settings)


def validate_name(name):
    if type(name) is not str or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", name):
        raise ValueError("Enter a tray name: 1–64 letters, digits, underscores or hyphens")


def _path(path, root):
    path = Path(path).expanduser().absolute()
    directory = tray_directory(root)
    if (directory.is_symlink() or path.is_symlink() or path.with_suffix(".pt").is_symlink()
            or path.resolve().parent != directory.resolve() or path.suffix != ".yaml"):
        raise ValueError("Select a regular YAML directly inside offline_teach/tray_teach/")
    return path.resolve()


def _binding(value, suffix):
    if value is None:
        return
    _fields(value, ("filename", "sha256"), "Draft source")
    name, digest = value["filename"], value["sha256"]
    if (type(name) is not str or Path(name).name != name or not name.endswith(suffix)
            or type(digest) is not str or not re.fullmatch("[0-9a-f]{64}", digest)):
        raise ValueError("Invalid draft source filename or hash")


def validate_draft(document):
    _fields(document, ("schema_version", "artifact_type", "created_at_utc", "form",
                       "model", "camera_calibration", "tray_teach_position",
                       "reference_plane"), "Tray Teach draft")
    if type(document["schema_version"]) is not int or document["schema_version"] != 1:
        raise ValueError("Tray Teach draft schema must be exactly 1")
    if document["artifact_type"] != "tray_teach_draft":
        raise ValueError("Expected a Tray Teach draft")
    _timestamp(document["created_at_utc"], "created_at_utc")
    validate_session(document["form"])
    if document["form"]["schema_version"] != 2:
        raise ValueError("Tray Teach draft requires text form schema 2")
    validate_name(document["form"]["draft"]["name"].strip())
    _binding(document["model"], ".pt")
    _binding(document["camera_calibration"], ".yaml")
    if document["tray_teach_position"] is not None:
        validate_home(document["tray_teach_position"])
    if document["reference_plane"] is not None:
        validate_plane(document["reference_plane"])
        if document["camera_calibration"] is None:
            raise ValueError("A saved plane requires its calibration binding")


def load_document(path, root):
    """GUI-only reader; runtime continues to use core.load_profile exclusively."""
    path = _path(path, root)
    try:
        document = yaml.load(path.read_bytes(), Loader=_UniqueKeyLoader)
        if not isinstance(document, dict) or document.get("artifact_type") != "tray_teach_draft":
            return load_profile(path, root)
        validate_draft(document)
        model = document["model"]
        if model is not None and (
                model["filename"] != path.with_suffix(".pt").name or
                file_sha256(path.with_suffix(".pt")) != model["sha256"]):
            raise ValueError("Draft requires its unchanged same-stem model")
        return document
    except (OSError, yaml.YAMLError, TypeError, KeyError) as exc:
        raise ValueError(f"Invalid Tray Teach document: {exc}") from exc


def detection_profile(document, model_task):
    """Validate saved detection data, including old GUI drafts, without rewriting files.

    The verified paired model supplies the task omitted from the draft format.
    Unsaved GUI values must never fill missing fields in this saved-data view.
    """
    if document["artifact_type"] != "tray_teach_draft":
        validate_profile(document)
        return document
    validate_draft(document)
    for key, message in (("model", "Browse and save a segmentation or OBB model"),
                         ("camera_calibration", "Browse and save matching calibration"),
                         ("reference_plane", "Create and save a reference plane")):
        if document[key] is None:
            raise ValueError(message)
    if model_task not in ("segment", "obb"):
        raise ValueError("Tray pose requests require a verified segmentation or OBB model")
    form = document["form"]["draft"]
    try:
        settings = {"name": form["name"].strip(), "model_task": model_task,
                    "geometry_source": "mask" if model_task == "segment" else "obb",
                    "geometry": {key: float(form[key]) for key in
                                 ("length_mm", "width_mm", "tolerance_mm")},
                    "yolo": {"confidence": float(form["confidence"]), "iou": float(form["iou"]),
                             "max_detections": int(form["max_detections"]),
                             "image_size": form["image_size"], "class_ids": form["class_ids"]}}
    except ValueError as exc:
        raise ValueError(
            "Complete and save numeric size/confidence/IoU/detection settings") from exc
    profile = {key: copy.deepcopy(document[key]) for key in (
        "schema_version", "created_at_utc", "model", "camera_calibration",
        "tray_teach_position", "reference_plane")}
    profile.update(artifact_type="tray_teach", settings=settings,
                   origin_convention=ORIGIN_CONVENTION, frame_id="base_link",
                   units={"geometry": "mm", "plane": "m", "tray_teach_position": "rad"})
    validate_profile(profile)
    return profile


@dataclass(frozen=True)
class SaveTarget:
    path: Path
    name: str
    yaml_sha256: str
    model_sha256: str | None
    created_at_utc: str


def save_target(path, document, root, expected_digest):
    path = _path(path, root)
    if file_sha256(path) != expected_digest:
        raise ValueError("Tray Teach file changed while loading; reload it")
    name = (document["form"]["draft"]["name"].strip()
            if document["artifact_type"] == "tray_teach_draft" else document["settings"]["name"])
    weights = path.with_suffix(".pt")
    return SaveTarget(path, name, expected_digest,
                      file_sha256(weights) if weights.exists() else None,
                      document["created_at_utc"])


def open_document(path, root):
    digest = file_sha256(_path(path, root))
    document = load_document(path, root)
    return document, save_target(path, document, root, digest)


def _check_target(target, root):
    path = _path(target.path, root)
    weights = path.with_suffix(".pt")
    if (file_sha256(path) != target.yaml_sha256 or
            (file_sha256(weights) if weights.exists() else None) != target.model_sha256):
        raise ValueError("Loaded YAML/model changed externally; reload before overwriting")


def _replace(target, artifact, weights, digest, root):
    """Publish YAML last, retain one previous version, and roll back failed commits."""
    _check_target(target, root)
    output, model = target.path, target.path.with_suffix(".pt")
    previous = output.with_name(f".{output.stem}.previous.zip")
    if previous.is_symlink():
        raise ValueError("Previous-version backup must not be a symlink")
    backup = artifact.parent / "previous.zip"
    with zipfile.ZipFile(backup, "w", compression=zipfile.ZIP_STORED) as archive:
        archive.write(output, output.name)
        if target.model_sha256 is not None:
            archive.write(model, model.name)
    with backup.open("rb") as stream:
        os.fsync(stream.fileno())
    with zipfile.ZipFile(backup) as archive:
        for name, expected in ((output.name, target.yaml_sha256),
                               (model.name, target.model_sha256)):
            if expected is not None:
                with archive.open(name) as stream:
                    actual = hashlib.sha256()
                    for block in iter(lambda: stream.read(1024 * 1024), b""):
                        actual.update(block)
                if actual.hexdigest() != expected:
                    raise ValueError("Tray Teach files changed during backup")
    _check_target(target, root)
    os.replace(backup, previous)
    _sync_directory(output.parent)
    changed = digest != target.model_sha256
    rollback = artifact.parent / "original.pt"
    if changed and target.model_sha256 is not None:
        os.link(model, rollback)
    if changed and weights is not None:
        os.replace(weights, model)
    try:
        os.replace(artifact, output)
    except OSError:
        if changed and weights is not None:
            if file_sha256(model) != digest:
                raise ValueError(f"Concurrent model change; previous version is in {previous}")
            if target.model_sha256 is None:
                model.unlink()
            else:
                os.replace(rollback, model)
            _sync_directory(output.parent)
        raise
    if changed and weights is None:
        model.unlink()
    _sync_directory(output.parent)


def save_document(form, settings, position, plane, camera, model, root, *, target=None,
                  readiness_error="", plane_camera_calibration=None):
    """Save any named form; only fully validated data becomes a runtime profile."""
    validate_session(form)
    name = form["draft"]["name"].strip()
    validate_name(name)
    directory = tray_directory(root)
    if directory.is_symlink():
        raise ValueError("Tray teach directory must not be a symlink")
    directory.mkdir(parents=True, exist_ok=True)
    stamp = utc_now()
    token = stamp.replace("-", "").replace(":", "").replace(".", "_")
    output = directory / f"tray_teach_{name}_{token}.yaml"
    overwrite = target is not None and name == target.name
    if overwrite:
        _check_target(target, root)
        output, stamp = target.path, target.created_at_utc
    source = Path(model["path"]).resolve(strict=True) if model is not None else None
    digest = model["sha256"] if model is not None else None
    if source is not None and (source.suffix != ".pt" or not source.stat().st_size or
                               file_sha256(source) != digest):
        raise ValueError("Loaded model changed; reload before saving")
    if camera is not None and file_sha256(camera.path) != camera.sha256:
        raise ValueError("Loaded calibration changed; reload before saving")
    common = {"schema_version": 1, "created_at_utc": stamp,
              "model": {"filename": output.with_suffix(".pt").name, "sha256": digest}
              if source is not None else None,
              "camera_calibration": (
                  copy.deepcopy(plane_camera_calibration)
                  if plane is not None and plane_camera_calibration is not None else
                  {"filename": camera.path.name, "sha256": camera.sha256}
                  if camera is not None else None),
              "tray_teach_position": copy.deepcopy(position),
              "reference_plane": copy.deepcopy(plane)}
    document = {**common, "artifact_type": "tray_teach", "settings": copy.deepcopy(settings),
                "origin_convention": ORIGIN_CONVENTION, "frame_id": "base_link",
                "units": {"geometry": "mm", "plane": "m", "tray_teach_position": "rad"}}
    try:
        if readiness_error:
            raise ValueError(readiness_error)
        if settings is None:
            raise ValueError("Complete the model, selected classes and detection/size settings")
        validate_settings(settings)
        if plane is None or camera is None:
            raise ValueError("Load matching calibration and create a reference plane")
        validate_profile(document)
        reason = ""
    except ValueError as exc:
        reason = str(exc)
        document = {**common, "artifact_type": "tray_teach_draft", "form": copy.deepcopy(form)}
        # A paired draft remains portable even when its original source moves.
        if source is not None:
            document["form"]["model_path"] = output.with_suffix(".pt").name
        validate_draft(document)
    with tempfile.TemporaryDirectory(prefix=".tray_teach_", dir=directory) as temporary:
        artifact = Path(temporary) / "profile.yaml"
        weights = Path(temporary) / "model.pt" if source is not None else None
        if weights is not None:
            shutil.copyfile(source, weights)
            if file_sha256(weights) != digest or file_sha256(source) != digest:
                raise ValueError("Loaded model changed during save")
            with weights.open("rb") as stream:
                os.fsync(stream.fileno())
        if camera is not None and file_sha256(camera.path) != camera.sha256:
            raise ValueError("Loaded calibration changed during save")
        with artifact.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        if overwrite:
            _replace(target, artifact, weights, digest, root)
        else:
            if output.with_suffix(".pt").exists() or output.with_suffix(".pt").is_symlink():
                raise FileExistsError(output.with_suffix(".pt"))
            if weights is not None:
                os.link(weights, output.with_suffix(".pt"))
            try:
                os.link(artifact, output)
            except BaseException:
                if weights is not None:
                    output.with_suffix(".pt").unlink()
                raise
            _sync_directory(directory)
    saved = save_target(output, document, root, file_sha256(output))
    return output, document, saved, reason
