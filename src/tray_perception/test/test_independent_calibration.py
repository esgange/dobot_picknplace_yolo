"""Recalibration changes live projection without rewriting taught base-frame geometry."""

import copy
import json
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from tray_perception import calibration, core, documents, node as node_module
from tray_perception.node import TrayTeachNode
from test_core import plane, position, settings
from test_documents import ready_form, sources as saved_sources
from test_requests import artifact as saved_artifact, backend as saved_backend, arm, trigger


@pytest.fixture
def sources(tmp_path):
    return saved_sources.__wrapped__(tmp_path)


@pytest.fixture
def backend(tmp_path):
    return saved_backend.__wrapped__(tmp_path, saved_artifact.__wrapped__(tmp_path))


def test_default_camera_uses_shared_active_choice_and_never_teaching_filename(
        tmp_path, monkeypatch):
    (tmp_path / ".env").write_text("ITEM_TEACH_ROBOT_CAMERA_CALIBRATION=new.yaml\n")
    monkeypatch.setattr(calibration, "load_robot_lan1_ip", lambda _root: "192.0.2.1")
    selected = MagicMock(return_value=object())
    monkeypatch.setattr(calibration, "selected_robot_camera", selected)
    assert calibration.active_robot_camera(tmp_path) is selected.return_value
    selected.assert_called_once_with(tmp_path / "calibration/new.yaml", root=tmp_path)
    (tmp_path / ".env").write_text("ITEM_TEACH_ROBOT_CAMERA_CALIBRATION=\n")
    with pytest.raises(ValueError, match="Select the active"):
        calibration.active_robot_camera(tmp_path)


def test_apply_replacement_preserves_plane_pose_and_capture_history(tmp_path, monkeypatch):
    camera = SimpleNamespace(path=tmp_path / "new.yaml",
                             settings=SimpleNamespace(camera_prefix="robot_camera"))
    monkeypatch.setattr(node_module, "load_camera_calibration", lambda *_a, **_k: camera)
    node = SimpleNamespace(root=tmp_path, lock=threading.RLock(), generation=3,
                           plane=plane(), position=position(), plane_camera_calibration="old",
                           invalidate=MagicMock(), connect_camera=MagicMock(), events=MagicMock())
    before = copy.deepcopy((node.plane, node.position, node.plane_camera_calibration))
    TrayTeachNode.apply_camera(node, camera.path)
    assert before == (node.plane, node.position, node.plane_camera_calibration)
    node.invalidate.assert_called_once()
    node.connect_camera.assert_called_once_with("robot_camera")
    assert node.camera is camera


@pytest.mark.parametrize("draft", [False, True])
def test_save_preserves_original_plane_camera_after_recalibration(tmp_path, sources, draft):
    _, camera, model = sources
    original = calibration.camera_provenance(camera)
    camera.path.unlink()
    replacement = tmp_path / "current.yaml"
    replacement.write_text("new calibrated mounting transform")
    camera = SimpleNamespace(path=replacement, sha256=core.file_sha256(replacement))
    path, saved, _, _ = documents.save_document(
        ready_form(), None if draft else settings(), position(), plane(), camera, model,
        tmp_path, plane_camera_calibration=original)
    assert saved["camera_calibration"] == original
    assert saved["reference_plane"] == plane() and saved["tray_teach_position"] == position()
    assert documents.load_document(path, tmp_path) == saved


def test_requests_use_replacement_camera_and_live_intrinsics_pin_current_file(backend):
    node, path, digest = backend
    original = copy.deepcopy(node.plane), path.read_bytes()
    old = node.camera.path
    new = old.with_name("replacement.yaml")
    new.write_text("new mounting calibration")
    node.camera = SimpleNamespace(path=new, sha256=core.file_sha256(new),
                                  settings=node.camera.settings)
    node.validate_sources = lambda: TrayTeachNode.validate_sources(node)
    old.unlink()
    snapshot = node.snapshot.side_effect

    def new_intrinsics():
        view = snapshot()
        view["camera_context"]["camera"]["k"][0] += 20.
        return view

    node.snapshot.side_effect = new_intrinsics
    api = arm(backend)
    response = trigger(backend)
    assert response.success and response.found
    assert json.loads(response.diagnostics_json)["camera_sha256"] == node.camera.sha256
    local = api.simulate(path, settings(), digest, (100_000_000_000, time.monotonic()))
    assert local["response"].found
    assert original == (node.plane, path.read_bytes())
    new.write_text("changed while armed")
    assert not trigger(backend).success


def test_controller_binds_active_camera_without_opening_teach_time_file(
        tmp_path, sources, monkeypatch):
    from robot_controller import tray

    path, old, _model = sources
    original = core.load_profile(path, tmp_path)
    old.path.unlink()
    replacement = tmp_path / "active.yaml"
    replacement.write_text("new calibration")
    current = SimpleNamespace(path=replacement, sha256=core.file_sha256(replacement))
    monkeypatch.setattr(tray, "active_robot_camera", lambda _root: current)
    config = tray.load_tray_configuration(
        path, tmp_path, SimpleNamespace(forward=lambda _joints: np.eye(4)))
    assert config.camera_sha256 == current.sha256
    assert config.profile == original
    assert config.detect_joints == tuple(original["tray_teach_position"]["positions_rad"])
    replacement.write_text("changed while configured")
    with pytest.raises(ValueError, match="camera calibration changed"):
        config.validate_sources(tmp_path)
