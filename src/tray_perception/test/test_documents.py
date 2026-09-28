import copy
from pathlib import Path
from types import SimpleNamespace
import zipfile

import pytest

from tray_perception import core, documents
from test_core import artifact as saved_artifact
from test_core import draft_session, plane, position, settings


@pytest.fixture
def sources(tmp_path):
    path, camera, model = saved_artifact.__wrapped__(tmp_path)
    camera.settings = SimpleNamespace(camera_prefix="robot_camera")
    metadata = {"path": str(model), "sha256": core.file_sha256(model), "task": "segment"}
    return path, camera, metadata


def form():
    value = draft_session()
    value["draft"].update(name="tray", camera_prefix="robot_camera")
    return value


def ready_form(prefix="robot_camera"):
    value, configured = form(), settings()
    value["draft"].update(
        name=configured["name"], camera_prefix=prefix,
        **{key: str(number) for key, number in configured["geometry"].items()},
        **{key: str(configured["yolo"][key]) for key in
           ("confidence", "iou", "max_detections")},
        image_size=configured["yolo"]["image_size"], class_ids=configured["yolo"]["class_ids"])
    return value


def test_name_only_and_invalid_edits_roundtrip_without_any_inputs(tmp_path):
    state = form()
    path, document, target, reason = documents.save_document(
        state, None, None, None, None, None, tmp_path)
    assert path.parent == tmp_path / "offline_teach/tray_teach"
    assert not path.with_suffix(".pt").exists()
    assert document["form"] == state and reason
    assert documents.load_document(path, tmp_path) == document
    with pytest.raises(ValueError, match="Incomplete Tray Teach draft"):
        core.load_profile(path, tmp_path)
    state["draft"]["confidence"] = "unfinished text"
    updated, document, target, _ = documents.save_document(
        state, None, None, None, None, None, tmp_path, target=target)
    assert updated == path and document["form"] == state
    assert documents.open_document(path, tmp_path) == (document, target)


def test_progressive_save_copies_model_preserves_plane_and_promotes_same_file(tmp_path, sources):
    _, camera, model = sources
    state = form()
    path, _, target, _ = documents.save_document(state, None, None, None, None, None, tmp_path)
    first = path.read_bytes()
    path2, draft, target, reason = documents.save_document(
        state, settings(), None, plane(), camera, model, tmp_path, target=target)
    assert path2 == path and not reason
    assert draft["artifact_type"] == "tray_teach" and draft["tray_teach_position"] is None
    assert draft["reference_plane"] == plane()
    assert draft["model"]["filename"] == path.with_suffix(".pt").name
    assert path.with_suffix(".pt").read_bytes() == Path(model["path"]).read_bytes()
    assert documents.load_document(path, tmp_path) == draft
    with zipfile.ZipFile(path.with_name(f".{path.stem}.previous.zip")) as archive:
        assert archive.read(path.name) == first
        assert archive.namelist() == [path.name]
    before, inode = path.read_bytes(), path.with_suffix(".pt").stat().st_ino
    path3, profile, target, reason = documents.save_document(
        state, settings(), position(), plane(), camera, model, tmp_path, target=target)
    assert path3 == path and not reason
    assert core.load_profile(path, tmp_path) == profile
    assert profile["created_at_utc"] == draft["created_at_utc"]
    assert path.with_suffix(".pt").stat().st_ino == inode
    with zipfile.ZipFile(path.with_name(f".{path.stem}.previous.zip")) as archive:
        assert archive.read(path.name) == before
        assert archive.read(path.with_suffix(".pt").name) == Path(model["path"]).read_bytes()
    assert len(list(path.parent.glob("*.yaml"))) == 2  # Fixture plus the one edited file.


def test_detection_complete_old_draft_needs_no_position_or_rewrite(tmp_path, sources):
    _, camera, model = sources
    path, draft, _, _ = documents.save_document(
        ready_form(), None, None, plane(), camera, model, tmp_path)
    before = path.read_bytes()
    normalized = documents.detection_profile(documents.load_document(path, tmp_path), "segment")
    assert normalized["settings"] == settings()
    assert normalized["tray_teach_position"] is None and normalized["reference_plane"] == plane()
    assert draft["artifact_type"] == "tray_teach_draft" and path.read_bytes() == before
    with pytest.raises(ValueError, match="Incomplete Tray Teach draft"):
        core.load_profile(path, tmp_path)  # Deployment still requires an explicit complete save.


@pytest.mark.parametrize("missing", [
    "model", "camera_calibration", "reference_plane", "width_mm", "class_ids", "confidence"])
def test_draft_normalization_cannot_fill_missing_detection_fields(tmp_path, sources, missing):
    _, camera, model = sources
    _, draft, _, _ = documents.save_document(
        ready_form(), None, None, plane(), camera, model, tmp_path)
    if missing in ("model", "camera_calibration", "reference_plane"):
        draft[missing] = None
    else:
        draft["form"]["draft"][missing] = [] if missing == "class_ids" else ""
    with pytest.raises(ValueError):
        documents.detection_profile(draft, "segment")


def test_loaded_complete_profile_updates_in_place_renaming_creates_new_pair(tmp_path, sources):
    original, camera, model = sources
    profile, target = documents.open_document(original, tmp_path)
    original_bytes = original.read_bytes()
    edited = settings()
    edited["geometry"]["tolerance_mm"] = 5.
    path, profile, target, reason = documents.save_document(
        form(), edited, position(), plane(), camera, model, tmp_path, target=target)
    assert path == original and not reason
    assert core.load_profile(path, tmp_path)["settings"] == edited
    with zipfile.ZipFile(path.with_name(f".{path.stem}.previous.zip")) as archive:
        assert archive.read(path.name) == original_bytes
    before = path.read_bytes()
    state = form()
    state["draft"]["name"] = edited["name"] = "renamed"
    renamed, _, _, _ = documents.save_document(
        state, edited, position(), plane(), camera, model, tmp_path, target=target)
    assert renamed != path and "renamed" in renamed.name
    assert path.read_bytes() == before and core.load_profile(renamed, tmp_path)


@pytest.mark.parametrize("change", ["yaml", "model"])
def test_external_changes_cannot_be_overwritten(tmp_path, sources, change):
    path, camera, model = sources
    _, target = documents.open_document(path, tmp_path)
    changed = path if change == "yaml" else path.with_suffix(".pt")
    changed.write_bytes(changed.read_bytes() + b"\nexternal change")
    before = changed.read_bytes()
    with pytest.raises(ValueError, match="changed externally"):
        documents.save_document(form(), settings(), position(), plane(), camera, model,
                                tmp_path, target=target)
    assert changed.read_bytes() == before


@pytest.mark.parametrize("existing_model", [False, True])
def test_failed_yaml_commit_restores_previous_pair(tmp_path, sources, monkeypatch, existing_model):
    _, camera, model = sources
    path, _, target, _ = documents.save_document(
        form(), None, None, None, None, model if existing_model else None, tmp_path)
    before = path.read_bytes()
    before_model = path.with_suffix(".pt").read_bytes() if existing_model else None
    replacement = tmp_path / "replacement.pt"
    replacement.write_bytes(b"different raw weights, never executed")
    model = {"path": str(replacement), "sha256": core.file_sha256(replacement), "task": "segment"}
    replace = documents.os.replace

    def fail_yaml(source, destination):
        if Path(destination) == path:
            raise OSError("synthetic full disk")
        return replace(source, destination)

    monkeypatch.setattr(documents.os, "replace", fail_yaml)
    with pytest.raises(OSError, match="full disk"):
        documents.save_document(form(), settings(), position(), plane(), camera, model,
                                tmp_path, target=target)
    assert path.read_bytes() == before
    assert (path.with_suffix(".pt").read_bytes() if existing_model else
            None if not path.with_suffix(".pt").exists() else "unexpected model") == before_model
    assert documents.load_document(path, tmp_path)
    assert not [p for p in path.parent.glob(".tray_teach_*") if p.is_dir()]


def test_draft_downgrade_never_runs_and_redeployment_requires_complete_profile(tmp_path, sources):
    path, camera, model = sources
    _, target = documents.open_document(path, tmp_path)
    state = form()
    state["draft"]["width_mm"] = ""
    path, draft, _, reason = documents.save_document(
        state, None, position(), plane(), camera, model, tmp_path, target=target)
    assert reason and draft["reference_plane"] == plane()
    with pytest.raises(ValueError, match="Incomplete Tray Teach draft"):
        core.load_profile(path, tmp_path)


@pytest.mark.parametrize("name", ["", "../bad", "two words", "x" * 65])
def test_invalid_name_does_not_create_artifacts(tmp_path, name):
    state = form()
    state["draft"]["name"] = name
    with pytest.raises(ValueError, match="tray name"):
        documents.save_document(state, None, None, None, None, None, tmp_path)
    assert not core.tray_directory(tmp_path).exists()


def test_draft_validation_keeps_geometry_and_file_boundary_guards(tmp_path, sources):
    _, camera, model = sources
    path, draft, _, _ = documents.save_document(
        form(), None, position(), plane(), camera, model, tmp_path)
    bad = copy.deepcopy(draft)
    bad["reference_plane"]["max_error_mm"] = 100.
    with pytest.raises(ValueError):
        documents.validate_draft(bad)
    symlink = path.parent / "tray_teach_link.yaml"
    symlink.symlink_to(path)
    with pytest.raises(ValueError, match="regular YAML"):
        documents.load_document(symlink, tmp_path)
    deployed = tmp_path / "runtime_teach/tray_teach.yaml"
    deployed.parent.mkdir()
    deployed.write_bytes(path.read_bytes())
    with pytest.raises(ValueError, match="offline_teach"):
        documents.load_document(deployed, tmp_path)
    with pytest.raises(ValueError, match="Incomplete Tray Teach draft"):
        core.load_profile(deployed, tmp_path, deployment=True)
