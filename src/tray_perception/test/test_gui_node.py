import ast
import copy
from concurrent.futures import Future
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from python_qt_binding import QtTest
from rclpy.time import Time

from tray_perception import gui
from tray_perception.node import TrayTeachNode
from test_core import camera_info, plane, position, preview_settings, settings


@pytest.fixture
def window(tmp_path):
    app = gui.QtWidgets.QApplication.instance() or gui.QtWidgets.QApplication([])
    node = SimpleNamespace(root=tmp_path, invalidate=MagicMock(), events=MagicMock(),
                           model=None, model_metadata=None, camera=None, plane=None,
                           position=None, selected=None, native=SimpleNamespace(failed=False),
                           fatal_error="", close=MagicMock(), generation=1,
                           work_lock=threading.RLock(), requests=SimpleNamespace(
                               service=None, busy=False, status="Disarmed", disarm=MagicMock()),
                           camera_prefix=None, camera_status="No camera connected",
                           accept_view=MagicMock(), get_clock=lambda: SimpleNamespace(
                               now=lambda: Time(seconds=100)), rviz=SimpleNamespace(
                               status=lambda: {"status": "waiting", "point_count": 0,
                                               "reason": "No camera"}))
    widget = gui.TrayTeachWindow(node)
    widget.timer.stop()
    yield widget
    widget.close()
    app.processEvents()


def test_gui_starts_unapplied_and_has_no_motion_client(window):
    assert window.settings is None and not window.preview_toggle.isChecked()
    assert window.node.model is None and window.node.camera is None
    for module in (gui, __import__("tray_perception.node", fromlist=["node"])):
        tree = ast.parse(Path(module.__file__).read_text())
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)]
        assert not any(isinstance(n.func, ast.Attribute) and n.func.attr in
                       ("create_client", "create_service") for n in calls)


def test_record_detect_pose_displays_degrees_and_saves_joint_radians(window, monkeypatch):
    from tray_perception.documents import load_document

    pose = position()
    pose["positions_rad"] = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    window.node.capture_detect_pose = MagicMock(return_value=pose)
    window.name.setText("tray")
    question = MagicMock()
    monkeypatch.setattr(gui.QtWidgets.QMessageBox, "question", question)
    window.node.invalidate.reset_mock()
    window.position_button.click()
    assert window.node.position == pose
    assert "Recorded on robot: 192.0.2.1" in window.position_label.text()
    assert "J1: +5.730°" in window.position_label.text()
    assert "J6: -34.377°" in window.position_label.text()
    assert "Saved in radians, joint1 through joint6" in window.position_label.text()
    assert "Save Tray Teach to store it" in window.status.text()
    question.assert_not_called()
    window.node.invalidate.assert_called_once()
    window.node.validate_sources = MagicMock(side_effect=ValueError("No calibration yet"))
    window._save()
    finish_jobs(window)
    assert load_document(window.profile_path, window.node.root)["tray_teach_position"] == pose


def test_detect_pose_replacement_requires_confirmation_and_keeps_old_on_failure(
        window, monkeypatch):
    original, replacement = position(), position()
    replacement["positions_rad"] = [.2] * 6
    window.node.position = original
    window.node.capture_detect_pose = MagicMock(return_value=replacement)
    window._show_position()
    label = window.position_label.text()
    window.node.invalidate.reset_mock()
    question = MagicMock(return_value=gui.QtWidgets.QMessageBox.No)
    monkeypatch.setattr(gui.QtWidgets.QMessageBox, "question", question)
    window.position_button.click()
    assert window.node.position == original and window.position_label.text() == label
    window.node.invalidate.assert_not_called()
    window.node.capture_detect_pose.side_effect = ValueError("No fresh /joint_states feedback")
    window.position_button.click()
    assert window.node.position == original and window.position_label.text() == label
    assert "No fresh /joint_states feedback" in window.status.text()
    assert question.call_count == 1
    window.node.capture_detect_pose.side_effect = None
    question.return_value = gui.QtWidgets.QMessageBox.Yes
    window.position_button.click()
    assert window.node.position == replacement
    assert window.position_label.text() != label
    window.node.invalidate.assert_called_once()


def automatic_sources(window):
    camera = window.node.root / "calibration/camera_to_hand_calibration_test.yaml"
    camera.parent.mkdir(exist_ok=True)
    camera.write_text("synthetic calibration; loader is mocked")
    model = window.node.root / "model.pt"
    model.write_bytes(b"synthetic weights; inspector is mocked")
    calls = []

    def load_camera(path):
        calls.append(("camera", path))
        window.node.camera = SimpleNamespace(
            path=path, settings=SimpleNamespace(camera_prefix="robot_camera"))
        return window.node.camera

    def load_model(path):
        calls.append(("model", path))
        window.node.model = {"path": str(path), "task": "segment", "sha256": "1" * 64}
        window.node.model_metadata = {"task": "segment", "classes": {"0": "tray", "1": "brown"}}
        return window.node.model_metadata

    window.node.apply_camera = MagicMock(side_effect=load_camera)
    window.node.inspect_model = MagicMock(side_effect=load_model)
    return camera, model, calls


def test_restored_available_sources_load_once_in_order_and_keep_classes(window, monkeypatch):
    from test_core import draft_session
    camera, model, calls = automatic_sources(window)
    state = draft_session()
    state.update(camera_filename=camera.name, model_path=str(model))
    state["draft"].update(name="brown_tray", confidence="0.25", class_ids=[1])
    gui.write_session(window.node.root, state)
    question = MagicMock(side_effect=AssertionError("No model confirmation needed"))
    monkeypatch.setattr(gui.QtWidgets.QMessageBox, "question", question)
    restored = gui.TrayTeachWindow(window.node)
    restored.timer.stop()
    try:
        assert not calls
        restored._tick()
        finish_jobs(restored)
        assert calls == [("camera", camera), ("model", model)]
        assert restored.camera_prefix.text() == "robot_camera"
        assert restored._class_ids() == [1] and restored.image_size == 320
        assert restored.preview_toggle.isChecked() and not restored.armed_toggle.isChecked()
        assert restored.node.plane is None and restored.node.position is None
        for _ in range(3):
            restored._tick()
        assert len(calls) == 2
        question.assert_not_called()
    finally:
        restored.close()


def test_browse_loads_sources_and_cancel_preserves_them(window, monkeypatch):
    camera, model, calls = automatic_sources(window)
    buttons = [button.text() for button in window.findChildren(gui.QtWidgets.QPushButton)]
    assert buttons.count("Browse…") == 2 and "Load…" not in buttons
    monkeypatch.setattr(window, "_choose", lambda *_a: camera)
    window._load_camera()
    finish_jobs(window)
    monkeypatch.setattr(window, "_choose", lambda *_a: model)
    window._load_model()
    finish_jobs(window)
    assert calls == [("camera", camera), ("model", model)]
    monkeypatch.setattr(window, "_choose", lambda *_a: None)
    window._load_camera()
    window._load_model()
    assert len(calls) == 2 and window.node.model["path"] == str(model)


def test_missing_or_invalid_camera_does_not_block_model_or_repeat_load(window):
    camera, model, calls = automatic_sources(window)
    window.camera_path.setText("missing.yaml")
    window.model_path.setText(str(model))
    window._tick()
    finish_jobs(window)
    assert calls == [("model", model)]
    window.node.apply_camera.side_effect = ValueError("Invalid calibration")
    window.camera_path.setText(camera.name)
    window._tick()
    finish_jobs(window)
    assert "Invalid calibration" in window.status.text()
    for _ in range(3):
        window._tick()
    window.node.apply_camera.assert_called_once()
    assert window.node.model is not None and window.node.camera is None


def test_available_sources_wait_for_preview_and_explicit_work(window):
    camera, model, calls = automatic_sources(window)
    window.camera_path.setText(camera.name)
    window.model_path.setText(str(model))
    window.future, window.job_kind, window.completion = Future(), "preview", MagicMock()
    window._tick()
    assert not calls and window.pending_job is None
    window.future.set_result(None)
    window._tick()
    finish_jobs(window)
    assert calls == [("camera", camera), ("model", model)]


def test_canvas_maps_scaled_pixels_and_rejects_letterboxing(window):
    window._corner_evidence = MagicMock()
    canvas = window.canvas
    canvas.resize(800, 800)
    canvas.show_frame(bytes(640 * 480 * 3), 640, 480)
    assert canvas.image_point(gui.QtCore.QPointF(400, 400)) == pytest.approx((320, 240))
    assert canvas.image_point(gui.QtCore.QPointF(100, 50)) is None
    window.plane_view = {"generation": window.node.generation}
    window._plane_click(10, 10)
    window._plane_click(11, 11)  # Duplicate point refused.
    assert len(window.points) == 1
    for x, y in ((100, 10), (100, 100), (10, 100), (200, 200)):
        window._plane_click(x, y)
    assert len(window.points) == 4
    window._undo()
    assert len(window.points) == 3


def test_settings_and_state_restore_without_loading_weights(window):
    window.node.model = {"task": "segment", "path": "/synthetic.pt"}
    window.node.model_metadata = {"task": "segment", "classes": {"0": "tray"}}
    window._model_loaded(window.node.model_metadata)
    profile_settings = settings()
    profile_settings["yolo"]["image_size"] = 320
    window._fill_settings(profile_settings)
    window._refresh_preview_settings()
    window.session_due = 0
    window._tick()
    assert window.settings == profile_settings
    assert window.preview_toggle.isChecked()
    node = SimpleNamespace(**vars(window.node))
    node.model = node.camera = None
    restored = gui.TrayTeachWindow(node)
    restored.timer.stop()
    try:
        assert restored.name.text() == "tray"
        assert restored.settings is None and not restored.preview_toggle.isChecked()
        assert restored.node.model is None and restored.node.camera is None
        restored.node.model = {"task": "segment", "path": "/synthetic.pt"}
        restored._model_loaded({"task": "segment", "classes": {"0": "tray"}})
        assert restored._form_settings() == profile_settings
        assert restored.preview_toggle.isChecked()  # Explicit trusted model load enables preview.
    finally:
        restored.close()


def test_incomplete_draft_autosaves_during_busy_preview_and_restores_unapplied(window):
    window.camera_prefix.setText("alternate_camera")
    window.name.setText("unfinished_tray")
    window.dimensions["length_mm"].setText("300.50")
    window.confidence.setText("0.")
    window.camera_path.setText("camera_to_hand_calibration_test.yaml")
    window.model_path.setText("/synthetic.pt")
    # Neither a complete profile nor a loaded model/camera is needed to remember edits.
    assert window.session_due is not None
    assert gui.read_session(window.node.root) is None
    window.future, window.job_kind = Future(), "preview"
    window.session_due = 0
    window._tick()
    assert gui.read_session(window.node.root)["draft"]["length_mm"] == "300.50"
    window.future = None
    restored = gui.TrayTeachWindow(window.node)
    restored.timer.stop()
    try:
        assert restored.camera_prefix.text() == "alternate_camera"
        assert restored.name.text() == "unfinished_tray"
        assert restored.dimensions["length_mm"].text() == "300.50"
        assert restored.dimensions["width_mm"].text() == ""
        assert restored.confidence.text() == "0."
        assert restored.camera_path.text() == "camera_to_hand_calibration_test.yaml"
        assert restored.model_path.text() == "/synthetic.pt"
        assert restored.settings is None and restored.preview_settings is None
        assert not restored.preview_toggle.isChecked() and not restored.armed_toggle.isChecked()
        assert restored.node.model is None and restored.node.camera is None
        assert restored.node.plane is None and restored.node.position is None
    finally:
        restored.close()


def test_close_flushes_latest_invalid_edit_without_applying_it(window):
    window.confidence.setText("1.5")
    window.dimensions["width_mm"].setText("still typing")
    window.close()  # Before the debounce expires, including with YOLO OFF.
    state = gui.read_session(window.node.root)
    assert state["draft"]["confidence"] == "1.5"
    assert state["draft"]["width_mm"] == "still typing"
    restored = gui.TrayTeachWindow(window.node)
    restored.timer.stop()
    try:
        enable_model(restored)
        assert restored.preview_settings is None and restored.settings is None
        assert "confidence" in restored.preview_error
    finally:
        restored.close()


def test_remembered_classes_survive_relaunch_and_revalidate_against_explicit_model(window):
    enable_model(window)
    window.classes.item(1).setCheckState(gui.QtCore.Qt.Checked)
    window.image_size = 320
    window.close()
    node = SimpleNamespace(**vars(window.node))
    node.model = node.model_metadata = None
    restored = gui.TrayTeachWindow(node)
    restored.timer.stop()
    try:
        assert restored._class_ids() == [1]
        assert "load model to verify" in restored.classes.item(0).text()
        assert restored.image_size == 320
        enable_model(restored)
        assert restored._class_ids() == [1]
        restored.classes.item(1).setCheckState(gui.QtCore.Qt.Unchecked)
        restored._model_loaded(node.model_metadata)
        assert restored._class_ids() == []  # Re-loading cannot resurrect deselected IDs.
        restored.classes.item(0).setCheckState(gui.QtCore.Qt.Checked)
        node.model = {"task": "segment", "path": "/different.pt"}
        restored._model_loaded(node.model_metadata)
        assert restored._class_ids() == []  # Class IDs belong to the remembered model.
    finally:
        restored.close()


def test_legacy_complete_session_still_restores_without_executing_model(window):
    gui.write_session(window.node.root, {
        "schema_version": 1, "profile_filename": "tray_teach_saved.yaml",
        "camera_filename": "camera.yaml", "item_filename": "item.yaml",
        "model_path": "/synthetic.pt", "settings": settings()})
    restored = gui.TrayTeachWindow(window.node)
    restored.timer.stop()
    try:
        assert restored.name.text() == "tray" and restored._class_ids() == [0]
        assert restored.profile_filename == "tray_teach_saved.yaml"
        assert restored.node.model is None and restored.settings is None
        assert restored.profile_path is None and not restored.armed_toggle.isChecked()
        restored.close()
        assert gui.read_session(window.node.root)["schema_version"] == 2
        assert gui.read_session(window.node.root)["item_filename"] == ""
    finally:
        restored.close()


def test_draft_write_failure_is_visible_and_preserves_previous_session(window, monkeypatch):
    window.name.setText("saved")
    window._remember()
    window.name.setText("new")
    monkeypatch.setattr(gui, "write_session", MagicMock(side_effect=OSError("disk full")))
    window.session_due = 0
    window._tick()
    assert "Could not remember Tray Teach draft: disk full" in window.status.text()
    assert gui.read_session(window.node.root)["draft"]["name"] == "saved"
    assert window.name.isEnabled()


def finish_jobs(window):
    deadline = time.monotonic() + 3
    while window.future is not None or window.pending_job is not None:
        assert time.monotonic() < deadline
        QtTest.QTest.qWait(10)
        window._tick()


def restorable_sources(window, monkeypatch):
    from tray_perception import node as module
    from test_documents import sources
    path, camera, model = sources.__wrapped__(window.node.root)
    node = window.node
    node.lock, node.deployment = threading.RLock(), False

    def read_camera(*_a, **_k):
        return SimpleNamespace(**{**vars(camera), "sha256": gui.file_sha256(camera.path)})
    monkeypatch.setattr(module, "load_camera_calibration", read_camera)

    def inspect(path, expected_digest):
        assert gui.file_sha256(path) == expected_digest
        node.model = {**model, "path": str(path)}
        node.model_metadata = {"task": "segment", "classes": {"0": "tray"}}
        return node.model_metadata

    def apply(path):
        assert path == camera.path
        node.camera, node.camera_prefix = camera, camera.settings.camera_prefix
        node.plane = None
        return camera

    node.inspect_model = MagicMock(side_effect=inspect)
    node.apply_camera = MagicMock(side_effect=apply)
    node.load_saved = MagicMock(side_effect=lambda path: TrayTeachNode.load_saved(node, path))
    node.load_draft = MagicMock(side_effect=lambda path, digest: TrayTeachNode.load_draft(
        node, path, digest))
    return path, camera, model


@pytest.mark.parametrize("draft", [False, True])
def test_startup_reopens_exact_saved_pair_plane_pose_and_save_target(window, monkeypatch, draft):
    from tray_perception import documents
    from test_documents import ready_form
    _, camera, model = restorable_sources(window, monkeypatch)
    path, _, target, _ = documents.save_document(
        ready_form(), None if draft else settings(), position(), plane(), camera, model,
        window.node.root)
    # A later file and stale session form must not replace the remembered saved document.
    documents.save_document(ready_form(), settings(), None, plane(), camera, model,
                            window.node.root)
    state = ready_form()
    state.update(profile_filename=path.name, camera_filename="missing.yaml",
                 model_path="/missing.pt")
    state["draft"].update(name="unsaved_change", confidence="unfinished", class_ids=[])
    gui.write_session(window.node.root, state)
    before = path.read_bytes(), path.with_suffix(".pt").read_bytes()
    restored = gui.TrayTeachWindow(window.node)
    restored.timer.stop()
    restored.next_preview = float("inf")
    try:
        assert not restored.name.isEnabled()  # Lock only while the saved file is reopening.
        finish_jobs(restored)
        assert restored.profile_path == path and restored.save_target == target
        assert restored.profile_digest == target.yaml_sha256
        assert restored.settings == settings() and restored._class_ids() == [0]
        assert restored.node.plane == restored.saved_plane == plane()
        assert restored.node.position == position()
        assert restored.camera_prefix.text() == "robot_camera"
        assert restored.preview_toggle.isChecked() and not restored.armed_toggle.isChecked()
        assert restored.node.requests.service is None and restored.name.isEnabled()
        assert restored._trigger_settings() == settings()
        assert before == (path.read_bytes(), path.with_suffix(".pt").read_bytes())
        assert not path.with_name(f".{path.stem}.previous.zip").exists()
        assert gui.read_session(window.node.root)["profile_filename"] == path.name
        for _ in range(3):
            restored._tick()
        window.node.inspect_model.assert_called_once_with(path.with_suffix(".pt"), model["sha256"])
        window.node.apply_camera.assert_called_once_with(camera.path)
        restored.node.validate_sources = MagicMock()
        restored.dimensions["tolerance_mm"].setText("3")
        restored._save()
        finish_jobs(restored)
        assert restored.profile_path == path
        assert documents.load_document(path, window.node.root)["settings"]["geometry"][
            "tolerance_mm"] == 3.
    finally:
        restored.close()


def test_startup_reopens_name_only_draft_without_sources(window):
    from tray_perception import documents
    from test_documents import form
    state = form()
    state["draft"]["confidence"] = "unfinished"
    path, _, target, _ = documents.save_document(
        state, None, None, None, None, None, window.node.root)
    state["profile_filename"] = path.name
    gui.write_session(window.node.root, state)
    window.node.lock = threading.RLock()
    window.node.load_draft = lambda path, digest: TrayTeachNode.load_draft(
        window.node, path, digest)
    restored = gui.TrayTeachWindow(window.node)
    restored.timer.stop()
    try:
        finish_jobs(restored)
        assert restored.profile_path == path and restored.save_target == target
        assert restored.name.text() == "tray" and restored.confidence.text() == "unfinished"
        assert restored.node.model is None and restored.node.plane is None
        assert not restored.preview_toggle.isChecked() and not restored.armed_toggle.isChecked()
        assert "Loaded draft" in restored.status.text() and restored.save_button.isEnabled()
    finally:
        restored.close()


@pytest.mark.parametrize("failure", ["missing_yaml", "invalid_yaml", "model_hash", "camera_hash"])
def test_startup_rejects_bad_remembered_file_once_without_source_fallback(
        window, monkeypatch, failure):
    from test_documents import ready_form
    path, camera, model = restorable_sources(window, monkeypatch)
    state = ready_form()
    state.update(profile_filename=path.name, camera_filename=camera.path.name,
                 model_path=model["path"])
    gui.write_session(window.node.root, state)
    changed = path if failure.endswith("yaml") else (
        path.with_suffix(".pt") if failure == "model_hash" else camera.path)
    before = changed.read_bytes()
    if failure == "missing_yaml":
        changed.unlink()
    else:
        changed.write_bytes(b"invalid or changed source")
    restored = gui.TrayTeachWindow(window.node)
    restored.timer.stop()
    restored.next_preview = float("inf")
    try:
        finish_jobs(restored)
        assert restored.profile_path is None and restored.save_target is None
        assert "Could not restore " + path.name in restored.status.text()
        assert restored.node.model is None and restored.node.plane is None
        assert not restored.node.fatal_error and restored.name.isEnabled()
        for _ in range(3):
            restored._tick()
        restored.node.inspect_model.assert_not_called()
        restored.node.apply_camera.assert_not_called()
        # Correcting the source alone cannot silently retry or adopt another profile.
        changed.write_bytes(before)
        restored._tick()
        assert restored.profile_path is None
        monkeypatch.setattr(restored, "_choose", lambda *_: path)
        restored._load_tray()
        finish_jobs(restored)
        assert restored.profile_path == path and restored.saved_plane == plane()
        assert not restored.armed_toggle.isChecked()
    finally:
        restored.close()


def test_save_only_needs_name_and_reopens_partial_form_without_trusting_model(window, monkeypatch):
    window.node.lock = threading.RLock()
    window.node.validate_sources = MagicMock(side_effect=ValueError("Load a camera calibration"))
    window.node.load_draft = lambda path, digest: TrayTeachNode.load_draft(
        window.node, path, digest)
    window._trust = MagicMock(side_effect=AssertionError("No model should execute"))
    window._update_controls()
    assert not window.save_button.isEnabled()
    window.name.setText("brown_tray")
    window.confidence.setText("unfinished")
    window._update_controls()
    assert window.save_button.isEnabled()
    window._save()
    finish_jobs(window)
    path = window.profile_path
    assert path.parent == window.node.root / "offline_teach/tray_teach"
    assert "Saved draft" in window.status.text() and window.draft_reason
    assert not path.with_suffix(".pt").exists()
    window.dimensions["width_mm"].setText("200")
    window._save()
    finish_jobs(window)
    assert window.profile_path == path and window.save_target.path == path
    assert "Update" in window.save_button.toolTip()
    window.name.setText("unsaved_change")
    window.dimensions["width_mm"].clear()
    monkeypatch.setattr(window, "_choose", lambda *_a: path)
    window._load_tray()
    finish_jobs(window)
    assert window.name.text() == "brown_tray"
    assert window.dimensions["width_mm"].text() == "200"
    assert window.confidence.text() == "unfinished"
    assert window.save_target.path == path
    assert not window.preview_toggle.isChecked() and window.node.plane is None
    with pytest.raises(ValueError, match="YOLO"):
        window._trigger_settings()
    window._trust.assert_not_called()
    window.dimensions["width_mm"].setText("205")
    window._save()
    finish_jobs(window)
    assert window.profile_path == path
    document, target = gui.open_document(path, window.node.root)
    assert document["form"]["draft"]["width_mm"] == "205"
    assert target.yaml_sha256 == window.profile_digest
    assert len(list(path.parent.glob("*.yaml"))) == 1


def test_save_complete_form_without_position_preserves_plane_as_complete_profile(window):
    from test_documents import sources
    _, camera, model = sources.__wrapped__(window.node.root)
    window.node.camera = camera
    window.node.camera_prefix = camera.settings.camera_prefix
    window.node.plane = plane()
    window.node.model = model
    window.node.model_metadata = {"task": "segment", "classes": {"0": "tray"}}
    window.node.validate_sources = MagicMock()
    window._model_loaded(window.node.model_metadata)
    window.camera_prefix.setText(camera.settings.camera_prefix)
    window._fill_settings(settings())
    window.next_preview = float("inf")
    window._save()
    finish_jobs(window)
    assert not window.draft_reason and "Saved complete profile" in window.status.text()
    assert window.saved_plane == plane() and window.profile_path.exists()
    path = window.profile_path
    window.node.position = position()
    window._save()
    finish_jobs(window)
    assert window.profile_path == path and not window.draft_reason
    assert "Saved complete profile" in window.status.text()
    window.preview_toggle.setChecked(True)
    window._refresh_preview_settings()
    assert window._trigger_settings() == settings()


def test_loading_detection_complete_old_draft_allows_arm_without_save(window, monkeypatch):
    from tray_perception import documents
    from test_documents import ready_form, sources
    _, camera, model = sources.__wrapped__(window.node.root)
    path, document, _, _ = documents.save_document(
        ready_form(), None, None, plane(), camera, model, window.node.root)
    before = path.read_bytes()

    def load(*_):
        window.node.camera, window.node.plane = camera, plane()
        window.node.camera_prefix = camera.settings.camera_prefix
        window.node.model = {**model, "path": str(path.with_suffix(".pt"))}
        window.node.model_metadata = {"task": "segment", "classes": {"0": "tray"}}
        return document

    window.node.load_draft = MagicMock(side_effect=load)
    monkeypatch.setattr(window, "_choose", lambda *_: path)
    window._load_tray()
    window.next_preview = float("inf")
    finish_jobs(window)
    assert window.node.position is None and not window.draft_reason
    assert window._trigger_settings() == settings()
    assert "without another Save" in window.status.text()

    def arm(*_):
        window.node.requests.service = object()
    window.node.requests.arm = MagicMock(side_effect=arm)
    window.armed_toggle.setChecked(True)
    finish_jobs(window)
    assert window.armed_toggle.isChecked() and path.read_bytes() == before
    window.node.requests.arm.assert_called_once_with(path, settings(), gui.file_sha256(path))


def test_save_during_corner_capture_keeps_uncreated_corners(window):
    window.node.validate_sources = MagicMock(side_effect=ValueError("Load a camera calibration"))
    window.name.setText("new_tray")
    window.plane_view = {"generation": window.node.generation,
                         "rgb": {"stamp_ns": 100_000_000_000}}
    window.points = [(10, 10), (20, 20)]
    window.future, window.job_kind = Future(), "preview"
    window.node.invalidate.reset_mock()
    window._save()
    window.node.invalidate.assert_not_called()
    assert window.pending_job is not None
    window.future.set_result(None)  # Obsolete preview must not be displayed.
    finish_jobs(window)
    assert window.profile_path.exists()
    assert window.plane_view is not None and window.points == [(10, 10), (20, 20)]
    assert window.saved_plane is None


def test_file_dialogs_remember_choices_and_cancel_preserves_draft(window, monkeypatch):
    chooser = MagicMock(return_value=("", ""))
    monkeypatch.setattr(gui.QtWidgets.QFileDialog, "getOpenFileName", chooser)
    window.camera_path.setText("camera.yaml")
    window.model_path.setText("/synthetic.pt")
    window.profile_filename = "tray_teach_saved.yaml"
    for action, expected in (
            (window._load_camera, window.node.root / "calibration/camera.yaml"),
            (window._load_model, Path("/synthetic.pt")),
            (window._load_tray,
             window.node.root / "offline_teach/tray_teach/tray_teach_saved.yaml")):
        action()
        assert chooser.call_args.args[2] == str(expected)
        assert window.future is None
    assert window.model_path.text() == "/synthetic.pt"


def test_draft_autosave_waits_until_typing_pauses_with_yolo_off(window):
    window.timer.start(20)
    QtTest.QTest.keyClicks(window.name, "tray")
    QtTest.QTest.qWait(180)
    assert gui.read_session(window.node.root) is None
    QtTest.QTest.keyClicks(window.name, "_large")
    QtTest.QTest.qWait(180)
    assert gui.read_session(window.node.root) is None
    QtTest.QTest.qWait(200)
    assert gui.read_session(window.node.root)["draft"]["name"] == "tray_large"
    assert window.name.isEnabled() and not window.preview_toggle.isChecked()


def test_changed_settings_discard_inflight_view(window):
    window._show_view({"generation": 0})  # Must not access pixels on a stale response.
    assert window.canvas.image is None
    window._edited()
    assert not window.preview_toggle.isChecked() and window.settings is None


def enable_model(window):
    window.node.model = {"task": "segment", "path": "/synthetic.pt"}
    window.node.model_metadata = {"task": "segment", "classes": {"0": "tray", "1": "other"}}
    window._model_loaded(window.node.model_metadata)


def test_model_preview_needs_no_profile_geometry_or_selected_classes(window):
    enable_model(window)
    assert window.preview_toggle.isChecked()
    assert window.preview_settings["geometry"] is None
    assert window.preview_settings["accepted_class_ids"] == []
    assert window.preview_settings["yolo"]["class_ids"] == [0, 1]
    assert not window.name.text() and window.node.camera is None and window.node.position is None
    assert window.settings is None


def test_live_settings_debounce_invalid_values_and_keep_manual_size(window, monkeypatch):
    now = [10.]
    monkeypatch.setattr(gui.time, "monotonic", lambda: now[0])
    enable_model(window)
    window.confidence.setText("0.")
    window._tick()
    assert window.preview_settings is None and window.preview_toggle.isChecked()
    now[0] += .2
    window.confidence.setText("0.45")
    now[0] += .2
    window._tick()
    assert window.preview_settings is None
    now[0] += .11
    window._tick()
    assert window.preview_settings["yolo"]["confidence"] == .45
    window.iou.setText("bad")
    now[0] += .31
    window._tick()
    assert window.preview_settings is None and window.preview_error
    window.iou.setText("0.4")
    window.dimensions["length_mm"].setText("200")
    window.dimensions["width_mm"].setText("100")
    window.dimensions["tolerance_mm"].setText("5")
    window.classes.item(0).setCheckState(gui.QtCore.Qt.Checked)
    now[0] += .31
    window._tick()
    assert not window.preview_error
    assert window.preview_settings["geometry"]["tolerance_mm"] == 5
    assert window.preview_settings["accepted_class_ids"] == [0]
    assert window.preview_settings["yolo"]["class_ids"] == [0, 1]
    window.dimensions["width_mm"].clear()
    now[0] += .31
    window._tick()
    assert window.preview_settings["geometry"] is None and window.geometry_error


def live_view(pixel=0):
    return {"generation": 1, "camera_context": None, "metric_error": "", "depth_error": "",
            "rgb": {"width": 2, "height": 2, "stamp_ns": 100_000_000_000,
                    "rgb": bytes([pixel]) * 12},
            "overlay": bytes([pixel]) * 12, "depth_overlay": bytes([pixel]) * 12, "cloud": None,
            "result": {"selected": None, "reason": "No eligible tray", "detections": []}}


def test_click_inspection_keeps_live_updates_without_overwriting_dimensions(window):
    enable_model(window)
    window.dimensions["length_mm"].setText("222")
    item = {"polygon": [[10, 10], [100, 10], [100, 100], [10, 100]],
            "source_index": 1, "confidence": .9, "class_name": "tray",
            "length_mm": 200., "width_mm": 100., "reason": "Unselected tray class"}
    window.last_view = {**live_view(), "result": {"detections": [item]}}
    window._click(50, 50)
    assert window.detail_sample is not None and window.plane_view is None
    assert window.dimensions["length_mm"].text() == "222"
    assert "X/width 100.0 mm × Y/length 200.0 mm" in window.detail_label.text()
    view = live_view(45)
    window._show_view(view)
    window.node.accept_view.assert_called_once_with(view)
    assert window.last_view is view and not window.canvas.highlight
    assert window.canvas.image.pixelColor(0, 0).red() == 45
    assert "Last clicked tray" in window.detail_label.text()


def test_prefix_edit_stops_old_preview_and_connect_runs_explicitly(window):
    window.node.camera_prefix = "camera_a"
    window.node.connect_camera = MagicMock(return_value="camera_b")
    window.camera_prefix.setText("camera_b")
    window.node.preview = MagicMock()
    window._tick()
    window.node.preview.assert_not_called()
    window._connect_camera()
    window.future.result(timeout=2)
    window._tick()
    window.node.connect_camera.assert_called_once_with("camera_b")


def test_yolo_off_keeps_background_camera_preview(window):
    window.node.camera_prefix = "cam"
    window.camera_prefix.setText("cam")
    window.node.preview = MagicMock(return_value={"generation": -1})
    window._tick()
    window.future.result(timeout=2)
    window._tick()
    window.node.preview.assert_called_once_with(None, generation=window.node.generation)


def test_slow_preview_keeps_typing_focus_and_controls_available(window):
    release = threading.Event()
    callback = MagicMock()
    window.show()
    field = window.dimensions["length_mm"]
    field.setFocus()
    gui.QtWidgets.QApplication.processEvents()
    heartbeats = []
    heartbeat = gui.QtCore.QTimer(window)
    heartbeat.timeout.connect(lambda: heartbeats.append(True))
    heartbeat.start(10)
    window.timer.start(10)
    try:
        window._job(lambda: release.wait(2), callback, "preview")
        window._tick()
        assert all(widget.isEnabled() for widget in window.form_fields)
        assert field.hasFocus()
        QtTest.QTest.keyClicks(field, "200")
        QtTest.QTest.qWait(100)
        assert field.text() == "200"
        assert field.hasFocus() and len(heartbeats) >= 3
        assert window.load_teach_button.isEnabled()
        assert window.snapshot_button.isEnabled()
    finally:
        window.timer.stop()
        heartbeat.stop()
        release.set()
        window.future.result(timeout=2)
    window._tick()
    callback.assert_not_called()  # The edited preview cannot replace current UI state.


def test_plane_snapshot_waits_for_preview_then_runs_off_gui_thread(window):
    release = threading.Event()
    thread_ids = []
    view = live_view()

    def freeze():
        thread_ids.append(threading.get_ident())
        release.wait(2)
        return view

    window.node.freeze_for_plane = freeze
    window.future = preview = Future()
    window.job_kind, window.completion = "preview", MagicMock()
    stale_callback = window.completion
    window._snapshot_plane()
    assert window.pending_job is not None
    assert not window.name.isEnabled()  # Only this explicit operation locks edits.
    window._job(MagicMock(), MagicMock(), "preview")
    assert not thread_ids and window.future is preview
    preview.set_result({"obsolete": True})
    window._tick()
    try:
        assert window.pending_job is None and window.job_kind == "snapshot"
        assert window.plane_view is None and not window.name.isEnabled()
    finally:
        release.set()
        window.future.result(timeout=2)
    window._tick()
    stale_callback.assert_not_called()
    assert thread_ids and thread_ids[0] != threading.get_ident()
    assert window.plane_view is view
    assert not window.findChildren(gui.QtWidgets.QDialog)
    assert window.name.isEnabled()
    assert window.canvas.image is not None
    assert "CAPTURED" in window.rgb_status.text()


def test_inline_corners_hold_one_observation_while_background_preview_continues(window):
    captured = live_view(10)
    window.node.freeze_for_plane = MagicMock(return_value=captured)
    window._snapshot_plane()
    window.future.result(timeout=2)
    window._tick()
    # Background preview/RViz continues, but cannot replace the clicked observation.
    window._show_view(live_view(90))
    assert window.canvas.image.pixelColor(0, 0).red() == 10
    assert window.last_view["rgb"]["rgb"][0] == 90
    window.node.corner_preview = MagicMock(return_value={
        "corner_overlay": bytes([20]) * 12, "depth_overlay": bytes([30]) * 12,
        "samples": [{"index": 1, "accepted": 49, "median_mm": 600, "reason": "Accepted"}]})
    window.future = preview = Future()
    window.job_kind, window.completion = "preview", MagicMock()
    window.node.invalidate.reset_mock()
    window._click(.5, .5)  # Main RGB clicks select corners while capture is active.
    assert window.pending_job[2] == "corners"
    window.node.invalidate.assert_not_called()  # Corner work must not invalidate its own source.
    preview.set_result(live_view(120))
    window._tick()
    window.future.result(timeout=2)
    window._tick()
    window.node.corner_preview.assert_called_once_with(captured, [[.5, .5]])
    assert window.canvas.image.pixelColor(0, 0).red() == 20
    assert window.depth_canvas.image.pixelColor(0, 0).red() == 30
    assert "49/49 valid" in window.detail_label.text()
    # The normal timer still schedules preview without another operator action.
    window.camera_prefix.blockSignals(True)
    window.camera_prefix.setText("cam")
    window.camera_prefix.blockSignals(False)
    window.node.camera_prefix = "cam"
    window.node.preview = MagicMock(return_value=live_view(150))
    window.next_preview = 0
    window._tick()
    window.future.result(timeout=2)
    window._tick()
    assert window.canvas.image.pixelColor(0, 0).red() == 20
    assert window.last_view["rgb"]["rgb"][0] == 150
    assert window.plane_view is captured and window.points == [[.5, .5]]
    window.cancel_plane_button.click()
    assert window.plane_view is None and not window.canvas.points
    assert window.canvas.image.pixelColor(0, 0).red() == 150


@pytest.mark.parametrize("invalidate", [False, True])
def test_discarded_or_invalidated_corner_work_cannot_restore_capture(window, invalidate):
    view = live_view()
    window.plane_view = view
    window._show_plane_view(view)
    window.points.append([.5, .5])
    window.node.corner_preview = MagicMock(return_value={
        "corner_overlay": bytes(12), "depth_overlay": bytes(12), "samples": []})
    window._corner_evidence()
    window.future.result(timeout=2)
    if invalidate:
        window.node.generation += 1
    else:
        window._discard_plane_draft()
    window._tick()
    assert window.plane_view is None and not window.points
    assert not window.canvas.points
    window.node.accept_view.assert_not_called()


def test_plane_create_queues_original_snapshot_and_keeps_existing_plane_until_commit(window):
    original = plane()
    view = live_view()
    window.node.plane = original
    window.plane_view = view
    window.points[:] = [[10, 10], [20, 10], [20, 20], [10, 20]]
    window._show_view(live_view(80))
    window.future = preview = Future()
    window.job_kind, window.completion = "preview", MagicMock()
    window.node.capture_plane = MagicMock(return_value={"max_error_mm": 1.2})
    window._capture()
    assert window.node.plane is original
    window.node.invalidate.assert_not_called()
    preview.set_result(live_view(100))
    window._tick()
    window.future.result(timeout=2)
    window._tick()
    window.node.capture_plane.assert_called_once_with(
        view, [[10, 10], [20, 10], [20, 20], [10, 20]])
    assert window.plane_view is None and not window.canvas.points
    assert "Save Tray Teach" in window.status.text()
    assert window.canvas.image.pixelColor(0, 0).red() == 80


def test_created_plane_readiness_distinguishes_unsaved_from_saved(window):
    window.node.plane = plane()
    window._tick()
    assert "GREEN / ready" in window.plane_label.text()
    assert "not saved" in window.plane_label.text()
    window.saved_plane = copy.deepcopy(window.node.plane)
    window._tick()
    assert "saved in teach file" in window.plane_label.text()
    window.node.plane = None
    window._tick()
    assert window.plane_label.text() == "Reference plane: not taught"
    assert not window.plane_label.styleSheet()


@pytest.mark.parametrize("copied_position", [None, position()])
def test_snapshot_acquisition_preserves_existing_plane_and_rejects_invalidated_sources(
        copied_position):
    original, view = plane(), live_view()
    node = SimpleNamespace(position=copied_position, plane=original,
                           snapshot=MagicMock(return_value=view),
                           visuals=MagicMock(return_value={}), _check_snapshot=MagicMock())
    TrayTeachNode.freeze_for_plane(node)
    node.snapshot.assert_called_once_with(depth_required=True)
    assert node.plane is original
    node._check_snapshot.assert_called_once_with(view)
    node._check_snapshot.side_effect = ValueError("source changed during snapshot")
    with pytest.raises(ValueError, match="source changed"):
        TrayTeachNode.freeze_for_plane(node)
    assert node.plane is original


@pytest.mark.parametrize("dimension_text", ["", "invalid"])
def test_measurement_inspection_needs_no_size_filter_or_copied_position(window, dimension_text):
    enable_model(window)
    for field in window.dimensions.values():
        field.setText(dimension_text)
    window._refresh_preview_settings()
    assert window.preview_settings["geometry"] is None and window.node.position is None
    item = {"polygon": [[10, 10], [100, 10], [100, 100], [10, 100]],
            "source_index": 1, "confidence": .9, "class_name": "tray", "valid": False,
            "length_mm": 200., "width_mm": 100., "reason": "Measured; size filter inactive"}
    view = live_view()
    view["result"].update(detections=[item], reason="1 tray(s) measured | Size filter inactive")
    window._show_view(view)
    window._click(50, 50)
    assert "X/width 100.0 mm × Y/length 200.0 mm" in window.detail_label.text()
    assert "measured" in window.result_label.text()
    assert all(field.text() == dimension_text for field in window.dimensions.values())
    with pytest.raises(ValueError, match="complete Tray Teach profile"):
        window._trigger_settings()


@pytest.mark.parametrize("terminal", [False, True])
def test_obsolete_preview_errors_do_not_hide_worker_failures(window, monkeypatch, terminal):
    window.future = failed = Future()
    window.job_kind, window.completion = "preview", MagicMock()
    window._edited()
    error = RuntimeError("worker failed") if terminal else ValueError("view invalidated")
    failed.set_exception(error)
    critical = MagicMock()
    monkeypatch.setattr(gui.QtWidgets.QMessageBox, "critical", critical)
    action = MagicMock()
    if terminal:
        window._job(action, MagicMock(), "save")
    window.node.events.reset_mock()
    window._tick()
    if terminal:
        critical.assert_called_once()
        assert window.node.fatal_error == "worker failed"
        assert window.pending_job is None
        action.assert_not_called()
    else:
        critical.assert_not_called()
        window.node.events.record.assert_not_called()
        assert "Updating detection settings" in window.result_label.text()
        assert not window.node.fatal_error and window.name.isEnabled()


def test_closing_cancels_waiting_action(window):
    window.future = preview = Future()
    window.job_kind, window.completion = "preview", MagicMock()
    completion = window.completion
    action = MagicMock()
    window._job(action, MagicMock(), "model")
    window.close()
    assert window.pending_job is None
    preview.set_result(None)
    window._tick()  # A queued timer/dialog callback cannot resume work after close.
    window._job(action, MagicMock(), "model")
    assert window.pending_job is None
    completion.assert_not_called()
    action.assert_not_called()


def snapshot_node():
    info = camera_info()
    rgb = {"width": 640, "height": 480, "stamp_ns": 100_000_000_000,
           "received_at": time.monotonic(), "rgb": bytes(640 * 480 * 3)}
    camera = SimpleNamespace(settings=SimpleNamespace(camera_link_frame="cam_link",
                                                      optical_frame="cam_color_optical_frame"),
                             calibration_mode="camera_to_hand")
    node = SimpleNamespace(validate_sources=MagicMock(), lock=threading.RLock(), rgb=rgb,
                           depth=None, color_info=info, depth_info=None, camera=camera,
                           generation=4, connection=1, tf_buffer=MagicMock(),
                           get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100)))
    node.raw_snapshot = lambda **kwargs: TrayTeachNode.raw_snapshot(node, **kwargs)
    return node


def test_detection_snapshot_has_no_depth_dependency(monkeypatch):
    import numpy as np
    from tray_perception import node as module
    node = snapshot_node()
    monkeypatch.setattr(module, "transform_matrix", lambda *_: np.eye(4))
    monkeypatch.setattr(module, "resolve_base_from_camera_link", lambda *_: np.eye(4))
    result = TrayTeachNode.snapshot(node)
    assert result["depth"] is None and result["generation"] == 4
    node.tf_buffer.lookup_transform.assert_called_once()
    with pytest.raises(ValueError, match="depth"):
        TrayTeachNode.snapshot(node, depth_required=True)
    node.rgb["stamp_ns"] -= 1_000_000_000
    with pytest.raises(ValueError, match="fresh"):
        TrayTeachNode.snapshot(node)


def test_on_hand_requires_fresh_timestamped_robot_tf(monkeypatch):
    import numpy as np
    from tray_perception import node as module
    node = snapshot_node()
    node.camera.calibration_mode = "camera_on_hand"
    transform = SimpleNamespace(header=SimpleNamespace(stamp=Time(seconds=98).to_msg()))
    node.tf_buffer.lookup_transform.return_value = transform
    monkeypatch.setattr(module, "transform_matrix", lambda *_: np.eye(4))
    monkeypatch.setattr(module, "resolve_base_from_camera_link", lambda *_: np.eye(4))
    with pytest.raises(ValueError, match="robot TF"):
        TrayTeachNode.snapshot(node)


@pytest.mark.parametrize("arrives", [True, False])
def test_snapshot_waits_briefly_for_exact_rgb_time_tf(monkeypatch, arrives):
    import numpy as np
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import Buffer, TransformException
    from tray_perception import node as module
    node = snapshot_node()
    node.camera.calibration_mode = "camera_on_hand"
    buffer = Buffer()

    def transform(parent, child, seconds):
        message = TransformStamped()
        message.header.frame_id, message.child_frame_id = parent, child
        message.header.stamp = Time(seconds=seconds).to_msg()
        message.transform.rotation.w = 1.
        return message

    buffer.set_transform_static(transform("cam_link", "cam_color_optical_frame", 0), "test")
    buffer.set_transform(transform("base_link", "Link6", 99.99), "test")
    node.tf_buffer = SimpleNamespace(lookup_transform=MagicMock(wraps=buffer.lookup_transform))
    monkeypatch.setattr(module, "resolve_base_from_camera_link", lambda *_: np.eye(4))
    publish = threading.Timer(
        .03, lambda: buffer.set_transform(transform("base_link", "Link6", 100.01), "test"))
    started = time.monotonic()
    if arrives:
        publish.start()
    try:
        if arrives:
            view = TrayTeachNode.snapshot(node)
            assert view["rgb"]["stamp_ns"] == 100_000_000_000
        else:
            with pytest.raises(TransformException):
                TrayTeachNode.snapshot(node)
        assert time.monotonic() - started < .5
        calls = node.tf_buffer.lookup_transform.call_args_list
        assert len(calls) == 2
        for call in calls:
            assert call.args[2].nanoseconds == 100_000_000_000
            assert 0 < call.kwargs["timeout"].nanoseconds <= 100_000_000
    finally:
        if arrives:
            publish.join(timeout=1)


def test_snapshot_rechecks_freshness_after_tf_wait(monkeypatch):
    import numpy as np
    from tray_perception import node as module
    node = snapshot_node()
    clock = SimpleNamespace(now=MagicMock(side_effect=[
        Time(seconds=100), Time(seconds=100), Time(seconds=100.6)]))
    node.get_clock = lambda: clock
    monkeypatch.setattr(module, "transform_matrix", lambda *_: np.eye(4))
    monkeypatch.setattr(module, "resolve_base_from_camera_link", lambda *_: np.eye(4))
    with pytest.raises(ValueError, match="expired while waiting"):
        TrayTeachNode.snapshot(node)


def test_bad_depth_does_not_invalidate_a_depth_free_tray_pose():
    camera = SimpleNamespace(settings=SimpleNamespace(camera_prefix="cam"))
    node = SimpleNamespace(camera=camera, lock=threading.RLock(), selected="existing pose",
                           depth="previous depth", events=MagicMock(), connection=1,
                           get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100)))
    invalid = SimpleNamespace(header=SimpleNamespace(frame_id="wrong_frame",
                                                     stamp=Time(seconds=100).to_msg()))
    TrayTeachNode._receive(node, invalid, "depth", "cam", 1)
    assert node.depth is None and node.selected == "existing pose"
    TrayTeachNode._receive(node, invalid, "rgb", "cam", 1)
    assert node.rgb is None and node.selected is None


def test_reconnecting_changes_exact_topics_and_rejects_retired_callbacks():
    from sensor_msgs.msg import Image
    node = snapshot_node()
    node.camera_prefix, node.plane = "old", plane()
    node.subscriptions_owned = [object()]
    retired = node.subscriptions_owned[0]
    node.destroy_subscription, node.events = MagicMock(), MagicMock()
    node.create_subscription = MagicMock(side_effect=lambda *args: args[2])
    node.invalidate = MagicMock()
    node._receive = lambda *args: TrayTeachNode._receive(node, *args)
    TrayTeachNode.connect_camera(node, "new")
    assert node.plane is None and node.rgb is None and node.connection == 2
    assert [call.args[1] for call in node.create_subscription.call_args_list] == [
        "/new/color/image_raw", "/new/depth/image_raw",
        "/new/color/camera_info", "/new/depth/camera_info"]
    node.destroy_subscription.assert_called_once_with(retired)
    message = Image()
    message.header.stamp = Time(seconds=100).to_msg()
    message.header.frame_id = "new_color_optical_frame"
    message.encoding, message.width, message.height, message.step = "rgb8", 2, 2, 6
    message.data = bytes(12)
    callbacks = list(node.subscriptions_owned)
    callbacks[0](message)
    assert node.rgb["width"] == 2
    TrayTeachNode.connect_camera(node, "new")  # Even same-prefix old connections are retired.
    callbacks[0](message)
    assert node.rgb is None
    with pytest.raises(ValueError, match="prefix"):
        TrayTeachNode.connect_camera(node, "/bad/name")


def test_calibration_prefix_mismatch_is_not_remapped():
    node = SimpleNamespace(fatal_error="", native=SimpleNamespace(failed=False),
                           camera_prefix="other", camera=SimpleNamespace(
                               settings=SimpleNamespace(camera_prefix="calibrated")))
    with pytest.raises(ValueError, match="does not match"):
        TrayTeachNode.validate_sources(node)


def test_teaching_tf_keeps_source_stamp_without_republishing_old_poses():
    pose = {"position": [.1, .2, .3], "quaternion": [1., 0., 0., 0.]}
    node = SimpleNamespace(native=SimpleNamespace(process=None, failed=False),
                           lock=threading.RLock(), generation=4, _published_key=None,
                           selected=(4, pose, 100_000_000_000, time.monotonic()),
                           fatal_error="", validate_sources=MagicMock(), broadcaster=MagicMock(),
                           requests=SimpleNamespace(tick=MagicMock()),
                           rviz=SimpleNamespace(displayed=None, tick=MagicMock(),
                                                show_pose=MagicMock(), clear_pose=MagicMock()))
    TrayTeachNode._tick(node)
    TrayTeachNode._tick(node)
    node.broadcaster.sendTransform.assert_called_once()
    message = node.broadcaster.sendTransform.call_args.args[0]
    assert message.header.frame_id == "base_link" and message.header.stamp.sec == 100
    assert message.transform.rotation.x == 1. and message.transform.rotation.w == 0.
    assert pose["quaternion"] == [1., 0., 0., 0.]
    node.rviz.show_pose.assert_called_once_with(message)
    node.selected = (4, pose, 100_000_000_000, time.monotonic() - 3)
    TrayTeachNode._tick(node)
    assert node.selected is None
    node.rviz.clear_pose.assert_called_once()


def test_corrupt_native_pose_is_terminal():
    view = {"generation": 4, "camera_context": {},
            "rgb": {"width": 2, "height": 2, "rgb": bytes(12), "stamp_ns": 100_000_000_000}}
    response = {"state": "ok", "width": 2, "height": 2, "count": 1,
                "selected": {"valid": True}, "detections": [{"valid": True}],
                "reason": "malformed synthetic pose", "inference_ms": 1.}
    node = preview_node(view, response)
    with pytest.raises(RuntimeError, match="Invalid native selected"):
        TrayTeachNode.preview(node, preview_settings(), generation=4)


def preview_node(view, response):
    view.update(depth=None, info=None, depth_info=None, metric_error="", depth_error="")
    return SimpleNamespace(
        lock=threading.RLock(), model={"task": "segment"}, plane=None, generation=4, selected=None,
        deployment=False,
        raw_snapshot=lambda: view, snapshot=lambda **_: view,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100)),
        native=SimpleNamespace(call=MagicMock(return_value=(response, bytes(12)))),
        visuals=MagicMock(return_value={"cloud": None, "depth_overlay": b"", "samples": []}),
        _check_snapshot=MagicMock())


@pytest.mark.parametrize("missing", ["calibration", "TF", "depth"])
def test_missing_geometry_inputs_do_not_stop_rgb_yolo(missing):
    from tf2_ros import TransformException
    view = {"generation": 4, "camera_context": None,
            "rgb": {"width": 2, "height": 2, "rgb": bytes(12), "stamp_ns": 100}}
    response = {"state": "ok", "width": 2, "height": 2, "count": 0,
                "selected": None, "detections": [], "reason": "No tray", "inference_ms": 1.}
    node = preview_node(view, response)
    if missing != "depth":
        error = ValueError("No calibration") if missing == "calibration" else TransformException(
            "No exact-time TF")
        node.snapshot = MagicMock(side_effect=error)
    config = preview_settings()
    config["geometry"], config["accepted_class_ids"] = None, []
    result = TrayTeachNode.preview(node, config, generation=4)
    assert result["overlay"] == bytes(12) and result["result"]["count"] == 0
    node.native.call.assert_called_once()
    assert node.selected is None  # Worker computation alone never publishes a pose.


def test_missing_depth_keeps_plane_pose_and_only_display_commits_it():
    view = {"generation": 4, "camera_context": {"camera": camera_info()},
            "rgb": {"width": 2, "height": 2, "rgb": bytes(12), "stamp_ns": 100}}
    selected = {"valid": True, "position": [0., 0., .2], "quaternion": [0., 0., 0., 1.],
                "polygon": [[0., 0.], [1., 0.], [1., 1.]], "source_index": 0,
                "class_name": "tray", "confidence": .9, "size_status": "pass", "reason": "OK"}
    response = {"state": "ok", "width": 2, "height": 2, "count": 1,
                "selected": selected, "detections": [selected], "reason": "", "inference_ms": 1.}
    node = preview_node(view, response)
    node.plane = {"camera": camera_info()}
    node.rviz = MagicMock()
    result = TrayTeachNode.preview(node, preview_settings(), generation=4)
    assert result["depth"] is None and result["result"]["selected"] == selected
    assert node.native.call.call_args.args[0]["plane"] == node.plane
    assert node.selected is None
    node.rviz.accept.assert_not_called()
    TrayTeachNode.accept_view(node, result)
    assert node.selected[1] == selected
    node.rviz.hold.assert_called_once()


@pytest.mark.parametrize("when", ["before_snapshot", "before_commit"])
def test_preview_cannot_publish_old_settings_after_edit(when):
    view = {"generation": 4, "camera_context": {},
            "rgb": {"width": 2, "height": 2, "rgb": bytes(12), "stamp_ns": 100}}
    selected = {"valid": True, "position": [0., 0., 0.], "quaternion": [0., 0., 0., 1.],
                "polygon": [[0., 0.], [1., 0.], [1., 1.]], "source_index": 0,
                "class_name": "tray", "confidence": .9, "size_status": "pass", "reason": "OK"}
    response = {"state": "ok", "width": 2, "height": 2, "count": 1,
                "selected": selected, "detections": [selected], "reason": "", "inference_ms": 1.}
    node = preview_node(view, response)
    if when == "before_snapshot":
        view["generation"] = node.generation = 5
        node._check_snapshot = MagicMock()
    else:
        node._check_snapshot = lambda _view, **_: setattr(node, "generation", 5)
    with pytest.raises(ValueError, match="invalidated"):
        TrayTeachNode.preview(node, preview_settings(), generation=4)
    assert node.selected is None
    if when == "before_snapshot":
        node.native.call.assert_not_called()


def test_loaded_profile_restores_position_without_opening_item_file(tmp_path, monkeypatch):
    from tray_perception import node as module
    source = {"settings": settings(), "model": {"sha256": "1" * 64},
              "camera_calibration": {"filename": "camera.yaml", "sha256": "2" * 64},
              "tray_teach_position": position(), "reference_plane": plane()}
    monkeypatch.setattr(module, "load_profile", lambda *_a, **_k: copy.deepcopy(source))
    monkeypatch.setattr(module, "load_camera_calibration", lambda *_a, **_k: SimpleNamespace(
        path=tmp_path / "calibration/camera.yaml", sha256="2" * 64))
    node = SimpleNamespace(root=tmp_path, lock=threading.RLock(), apply_camera=MagicMock(),
                           deployment=False,
                           inspect_model=MagicMock(return_value={"task": "segment",
                                                                 "classes": {"0": "tray"}}))
    result = TrayTeachNode.load_saved(node, tmp_path / "tray.yaml")
    assert node.position == position() and node.plane == plane() and result == source
    assert not (tmp_path / "offline_teach/item_teach").exists()


def test_loaded_draft_restores_optional_sources_and_plane_without_position(tmp_path, monkeypatch):
    from tray_perception import documents, node as module
    from test_documents import form, sources
    _, camera, model = sources.__wrapped__(tmp_path)
    path, _, target, _ = documents.save_document(
        form(), None, None, plane(), camera, model, tmp_path)
    monkeypatch.setattr(module, "load_camera_calibration", lambda *_a, **_k: camera)
    node = SimpleNamespace(root=tmp_path, lock=threading.RLock(), apply_camera=MagicMock(),
                           invalidate=MagicMock(), inspect_model=MagicMock())
    profile = TrayTeachNode.load_draft(node, path, target.yaml_sha256)
    assert profile["artifact_type"] == "tray_teach_draft"
    assert node.position is None and node.plane == plane()
    node.inspect_model.assert_called_once_with(path.with_suffix(".pt"), model["sha256"])
    node.apply_camera.assert_called_once_with(camera.path)
    camera.sha256 = "0" * 64
    node.inspect_model.reset_mock()
    with pytest.raises(ValueError, match="calibration changed"):
        TrayTeachNode.load_draft(node, path, target.yaml_sha256)
    node.inspect_model.assert_not_called()


def test_launch_is_local_only_and_does_not_launch_dependencies(monkeypatch):
    import runpy
    launch = Path(__file__).parents[1] / "launch/tray_teach.launch.py"
    factory = runpy.run_path(str(launch))["generate_launch_description"]
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "0")
    with pytest.raises(RuntimeError):
        factory()
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    description = factory()
    assert len(description.entities) == 1


def ready_trigger_window(window):
    enable_model(window)
    window._fill_settings(settings())
    window._refresh_preview_settings()
    window.profile_path = window.node.root / "saved.yaml"
    window.profile_digest = "1" * 64
    window.node.requests.validate_view = MagicMock()


def test_trigger_row_requires_saved_profile_and_arming_stays_explicit(window):
    assert window.preview_toggle.text() == "YOLO Detect: OFF"
    assert window.simulate_button.text() == "Simulate Trigger"
    assert window.armed_toggle.text() == "Armed: OFF"
    window._simulate_trigger()
    assert "Save or load" in window.status.text() and window.future is None
    ready_trigger_window(window)
    assert not window.armed_toggle.isChecked()

    def arm(*_):
        window.node.requests.service = object()
    window.node.requests.arm = MagicMock(side_effect=arm)
    window.armed_toggle.setChecked(True)
    window.future.result(timeout=2)
    window._tick()
    assert window.armed_toggle.isChecked() and "#b51f24" in window.armed_toggle.styleSheet()
    assert window.node.requests.arm.call_args.args[1] == settings()

    def disarm():
        window.node.requests.service = None
    window.node.invalidate.side_effect = disarm
    window.confidence.setText("invalid")
    window._tick()
    assert not window.armed_toggle.isChecked() and not window.node.yolo_enabled


def test_simulation_shows_shared_result_and_continues_live_without_arming(window):
    ready_trigger_window(window)
    view = {"generation": 1, "camera_context": None, "metric_error": "", "depth_error": "",
            "rgb": {"width": 2, "height": 2, "stamp_ns": 100_000_000_000},
            "overlay": bytes(12), "depth_overlay": b"", "cloud": None,
            "result": {"selected": None, "reason": "No eligible tray", "detections": []}}
    response = SimpleNamespace(success=True, status="NO_VALID_TRAY", message="No eligible tray")
    window.node.requests.simulate = MagicMock(return_value={"response": response, "view": view})
    before = {key: field.text() for key, field in window.dimensions.items()}
    window._simulate_trigger()
    window.future.result(timeout=2)
    window._tick()
    assert window.last_view is view
    assert "Last simulated request: NO_VALID_TRAY" in window.detail_label.text()
    assert not window.armed_toggle.isChecked()
    assert before == {key: field.text() for key, field in window.dimensions.items()}
    window.camera_prefix.blockSignals(True)
    window.camera_prefix.setText("cam")
    window.camera_prefix.blockSignals(False)
    window.node.camera_prefix = "cam"
    window.node.preview = MagicMock(return_value=live_view(60))
    window.next_preview = 0
    window._tick()
    window.future.result(timeout=2)
    window._tick()
    assert window.canvas.image.pixelColor(0, 0).red() == 60
    assert window.node.preview.call_count == 1
    assert "Last simulated request" in window.detail_label.text()


def test_simulated_detail_source_change_revokes_old_target_without_stopping_preview(window):
    view = {**live_view(), "trigger_binding": {}, "trigger_epoch": 1}
    window.node.requests.validate_view = MagicMock()
    window._show_detail(view, "Last simulated request: OK", trigger=True)
    window.node.requests.validate_view.side_effect = ValueError("profile changed")
    window._tick()
    assert window.detail_sample is None
    assert "invalidated: profile changed" in window.detail_label.text()
    window.node.invalidate.assert_called_with("profile changed")
    replacement = live_view(70)
    window._show_view(replacement)
    assert window.last_view is replacement and window.canvas.image.pixelColor(0, 0).red() == 70
