"""Read-only teaching stages and image selection; no ROS or physical camera launch."""

from pathlib import Path
import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from item_perception_yolo import item_teach_gui as gui
from item_perception_yolo import item_teach_core as core
from item_perception_yolo.item_detector import ItemDetectNode


@pytest.fixture
def window(tmp_path, monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    app = gui.QtWidgets.QApplication.instance() or gui.QtWidgets.QApplication([])
    monkeypatch.setattr(gui, "load_package_ui_state", lambda _: None)
    monkeypatch.setattr(gui, "ui_state_path", lambda: tmp_path / "last_session.json")
    monkeypatch.setattr(gui, "workspace_root", lambda: tmp_path)
    monkeypatch.setattr(gui.QtWidgets.QMessageBox, "warning", MagicMock())
    node = SimpleNamespace(
        events=MagicMock(), disarm=MagicMock(), arm=MagicMock(), close_runtime=MagicMock(),
        clear_selected_pose=MagicMock(), show_selected_pose=MagicMock(), clicked_pose=MagicMock(),
        native=SimpleNamespace(failed=False), service=None, yolo_enabled=False,
        preview_source="mask", preview_mode="all", last_view=None, settings=None,
        model_config={"path": str(tmp_path / "model.pt"), "task": "segment"},
        model_metadata={"task": "segment", "classes": {"1": "part", "4": "other"},
                        "geometry_sources": ["mask"]},
        camera_prefix="bin_camera", applied=None,
        camera_snapshot=lambda: (None, "Synthetic camera"),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100_100_000_000)),
    )
    node.enable_preview = lambda source, yolo, **kw: ItemDetectNode.enable_preview(
        node, source, yolo, **kw)
    node.enable_yolo = lambda settings: ItemDetectNode.enable_yolo(node, settings)
    widget = gui.ItemTeachWindow(node)
    widget.timer.stop()
    widget.model.setText(node.model_config["path"])
    widget.camera_prefix.setText("bin_camera")
    widget.task.setCurrentIndex(widget.task.findData("segment"))
    widget._populate_classes(node.model_metadata["classes"], [])
    widget._populate_sources(["mask"], "mask")
    yield widget
    widget.close()
    app.processEvents()


def test_single_view_starts_with_blank_dimensions_and_no_production_profile(window):
    assert not window._selected_classes()
    assert window.home is None and window.saved_path is None
    window.yolo_toggle.setChecked(True)
    assert window.node.yolo_enabled
    assert window.node.settings is None  # No production parser needed.
    assert "image_size" not in window.inputs
    assert window.profile_image_size == 640
    assert window.node.preview_geometry is None  # Explicit gray/unchecked size, no guess.
    assert window.node.preview_quality == gui.QUALITY_DEFAULTS
    window.node.arm.assert_not_called()
    assert window.node.service is None


def test_arming_from_all_view_still_requires_production_settings(window):
    window.yolo_toggle.setChecked(True)
    window.saved_path = Path("explicit_profile.yaml")
    window.armed_toggle.setChecked(True)
    assert not window.armed_toggle.isChecked()
    window.node.arm.assert_not_called()
    assert "Setting 'height' is required" in window.status.toPlainText()
    assert window.yolo_toggle.isChecked()


def test_single_view_has_no_mode_controls_and_never_autostarts(window):
    for name in ("preview_stage", "resume_live", "filtered_toggle", "detect_all_toggle"):
        assert not hasattr(window, name)
    assert not window.node.yolo_enabled and not window.yolo_toggle.isChecked()


def test_visual_first_layout_has_one_settings_column_and_persistent_actions(window):
    window.show()
    gui.QtWidgets.QApplication.processEvents()
    sidebar, preview = window.workspace_split.sizes()
    assert preview > sidebar * 2
    assert isinstance(window.settings_column, gui.QtWidgets.QVBoxLayout)
    groups = [window.settings_column.itemAt(i).widget()
              for i in range(window.settings_column.count() - 1)]
    assert len(groups) == 9
    assert all(isinstance(g, gui.QtWidgets.QGroupBox) for g in groups)
    assert "Camera" in groups[0].title() and "Item / model" in groups[1].title()
    assert "YOLO" in groups[2].title() and "Item size" in groups[3].title()
    assert all(a.geometry().bottom() < b.geometry().top()
               for a, b in zip(groups, groups[1:]))
    assert window.video.height() > window.depth_video.height()
    assert not window.status.isVisible()
    window.activity_toggle.setChecked(True)
    assert window.status.isVisible()
    window._message("Synthetic feedback")
    assert window.activity_summary.text() == "Synthetic feedback"
    window.settings_scroll.verticalScrollBar().setValue(10000)
    save = next(button for button in window.findChildren(gui.QtWidgets.QPushButton)
                if button.text() == "Save YAML + Model Copy…")
    assert not window.settings_scroll.isAncestorOf(save)
    assert save.isVisible()
    assert not window.node.yolo_enabled and window.node.service is None


def test_live_yolo_edits_debounce_apply_exact_values_and_disarm(window, monkeypatch):
    clock = [10.0]
    monkeypatch.setattr(gui.time, "monotonic", lambda: clock[0])
    window.yolo_toggle.setChecked(True)
    window.saved_path = Path("saved.yaml")
    window.frozen_view = window.node.last_view = {"old": "result"}
    window.inputs["confidence"].setText("0.9")
    assert window.yolo_toggle.isChecked() and not window.node.yolo_enabled
    assert window.preview_settings_paused and window.frozen_view is None
    assert window.node.last_view is None and window.saved_path is None
    window.node.disarm.assert_called()
    assert not window.armed_toggle.isChecked()
    clock[0] += .2
    window.inputs["iou"].setText("0.35")
    window.inputs["max_detections"].setText("20")
    clock[0] += .2
    window._refresh_video()
    assert not window.node.yolo_enabled  # Debounce restarts with the last edit.
    clock[0] += .11
    window._refresh_video()
    assert window.node.yolo_enabled and not window.preview_settings_paused
    assert window.node.preview_yolo == {"confidence": .9, "iou": .35,
                                        "max_detections": 20, "image_size": 640,
                                        "class_ids": [1, 4]}
    assert window.node.settings is None  # No production/depth settings required in All.
    assert "confidence 0.9, IoU 0.35" in window.preview_help.text()
    window.node.arm.assert_not_called()


@pytest.mark.parametrize("key,bad", [("confidence", ""), ("confidence", "40"),
                                     ("iou", "nan"), ("max_detections", "1.5")])
def test_invalid_live_settings_pause_and_correcting_resumes(window, key, bad):
    window.yolo_toggle.setChecked(True)
    original = window.inputs[key].text()
    window.inputs[key].setText(bad)
    window._apply_live_detection_settings()
    assert window.yolo_toggle.isChecked() and not window.node.yolo_enabled
    assert window.preview_settings_paused and window.preview_update_due is None
    assert "correct settings" in window.preview_status
    gui.QtWidgets.QMessageBox.warning.assert_not_called()  # No dialog while typing.
    window.inputs[key].setText(original)
    window._apply_live_detection_settings()
    assert window.node.yolo_enabled and not window.preview_settings_paused
    window.yolo_toggle.setChecked(False)
    window.inputs[key].setText(bad)
    window._refresh_video()
    assert not window.node.yolo_enabled and window.preview_update_due is None


def test_live_size_quality_class_edits_keep_all_detections_and_clear_selection(window):
    for key, value in (("height", "65"), ("width", "55"), ("tolerance", "10")):
        window.inputs[key].setText(value)
    window.classes.item(0).setCheckState(gui.QtCore.Qt.Checked)
    window.yolo_toggle.setChecked(True)
    assert window.node.yolo_enabled and window.node.preview_mode == "all"
    before = window.node.preview_geometry
    window.inputs["confidence"].setText("0.9")
    window._apply_live_detection_settings()
    assert window.yolo_toggle.isChecked() and window.node.yolo_enabled
    assert window.node.preview_yolo["confidence"] == .9
    assert window.node.preview_geometry == before
    assert window.node.preview_yolo["class_ids"] == [1, 4]
    window.classes.item(0).setCheckState(gui.QtCore.Qt.Unchecked)
    window._apply_live_detection_settings()
    assert window.node.yolo_enabled  # Production class checks do not hide teaching detections.
    window.classes.item(1).setCheckState(gui.QtCore.Qt.Checked)
    window._apply_live_detection_settings()
    assert window.node.yolo_enabled and window.node.preview_yolo["class_ids"] == [1, 4]
    assert window._inference_settings()["yolo"]["class_ids"] == [4]
    window.frozen_view = {"old": "selection"}
    window.inputs["height"].setText("75")
    assert window.frozen_view is None
    window.node.clear_selected_pose.assert_called()
    window._apply_live_detection_settings()
    assert window.node.preview_geometry["height"] == 75
    window.node.arm.assert_not_called()


def test_live_edit_discards_already_completed_old_preview_reply(window):
    window.yolo_toggle.setChecked(True)
    old = {"preview_mode": "all", "metadata": {"count": 999, "geometry_sources": ["obb"]}}
    window._job("preview", lambda: old)
    completed = window.job_results.get(timeout=3)
    window.job_results.put(completed)
    window.inputs["confidence"].setText("0.9")
    window.node.last_view = old  # Model a late old-generation assignment as well.
    window._refresh_video()
    assert window.node.last_view is None and window.displayed_view is None
    assert window.geometry_source.currentData() == "mask"
    assert "999" not in window.preview_status
    window._apply_live_detection_settings()
    assert window.node.yolo_enabled


def test_yolo_off_cancels_pending_live_update(window):
    window.yolo_toggle.setChecked(True)
    window.inputs["confidence"].setText("0.9")
    assert window.preview_update_due is not None
    window.yolo_toggle.setChecked(False)
    window._refresh_video()
    assert not window.node.yolo_enabled and window.preview_update_due is None


def test_selection_circle_size_comes_from_projected_points_and_live_diameter(window):
    import math
    window.yolo_toggle.setChecked(True)
    assert window.node.preview_depth_diameter == 30.
    assert window.node.preview_geometry is None  # Circle does not require filled size filters.
    window.inputs["pickdepth_radius"].setText("60")
    assert not window.node.yolo_enabled
    window._apply_live_detection_settings()
    assert window.node.preview_depth_diameter == 60. and window.node.yolo_enabled
    for radius in (18.75, 37.5):
        points = [[320+radius*math.cos(i*2*math.pi/96),
                   240+radius*math.sin(i*2*math.pi/96)] for i in range(96)]
        image = gui.QtGui.QImage(640, 480, gui.QtGui.QImage.Format_RGB888)
        image.fill(gui.QtGui.QColor("black"))
        painter = gui.QtGui.QPainter(image)
        gui.draw_sampling_circle(painter, points)
        painter.end()
        assert image.pixelColor(round(320+radius), 240) == gui.QtGui.QColor("cyan")
        assert image.pixelColor(329, 240) == gui.QtGui.QColor("black")  # No old 9 px ring.
        assert image.pixelColor(320, 240) == gui.QtGui.QColor("black")  # Never filled.
    painter = MagicMock()
    gui.draw_sampling_circle(painter, None)
    painter.drawPolygon.assert_not_called()  # No guessed/fixed ring when calibration is missing.


@pytest.mark.parametrize("bad", ["", "0", "-10", "nan"])
def test_invalid_pickdepth_diameter_pauses_even_with_blank_size_fields(window, bad):
    window.yolo_toggle.setChecked(True)
    window.inputs["pickdepth_radius"].setText(bad)
    window._apply_live_detection_settings()
    assert window.preview_settings_paused and not window.node.yolo_enabled
    window.inputs["pickdepth_radius"].setText("45")
    window._apply_live_detection_settings()
    assert window.node.yolo_enabled and window.node.preview_depth_diameter == 45.


@pytest.mark.parametrize("outcome", ["valid", "size", "depth", "error", "cancel"])
def test_click_pose_uses_displayed_snapshot_and_publishes_only_valid_tf(window, outcome):
    for key, value in (("height", "80"), ("width", "32"), ("tolerance", "1")):
        window.inputs[key].setText(value)
    window.classes.item(0).setCheckState(gui.QtCore.Qt.Checked)
    window.yolo_toggle.setChecked(True)
    item = {"source_index": 0, "class_id": 1, "class_name": "part", "confidence": .8,
            "polygon": [[270, 220], [370, 220], [370, 260], [270, 260]],
            "rectangle": [[270, 220], [370, 220], [370, 260], [270, 260]],
            "measurement": {"length_mm": 80., "width_mm": 32.}, "measurement_error": "",
            "size_valid": outcome != "size", "size_reason": "synthetic size check"}
    view = {"rgb": bytes(640*480*3), "width": 640, "height": 480,
            "stamp_ns": 100_000_000_000, "depth_stamp_ns": 100_000_000_000,
            "sequence": 1, "preview_mode": "all", "metadata": {"detections": [item]}}
    candidate = {"position": [.01, .02, .1], "quaternion": [0., 0., 0., 1.]}
    result = {"candidate": candidate if outcome in ("valid", "cancel") else None,
              "reason": "insufficient accepted depth samples/fraction",
              "depth_rgb": bytes(640*480*3), "stamp_ns": view["stamp_ns"], "epoch": 1}
    window.node.clicked_pose.return_value = result
    if outcome == "error":
        window.node.clicked_pose.side_effect = ValueError("Snapshot expired")
    window.node.last_view = view
    window._refresh_video()
    center = gui.QtCore.QPointF(window.video.contentsRect().center())
    window._select_detection(center)
    assert window.frozen_view is view
    window.node.last_view = {**view, "sequence": 2}
    window._refresh_video()
    if outcome == "size":
        window.node.clicked_pose.assert_not_called()
        assert "Size outside tolerance" in window.selected_pose_status
    else:
        completed = window.job_results.get(timeout=3)
        window.job_results.put(completed)
        if outcome == "cancel":
            window._resume_live()
        window._refresh_video()
        assert window.node.clicked_pose.call_args.args[0] is view
        assert window.node.clicked_pose.call_args.args[1] is item
    if outcome == "valid":
        window.node.show_selected_pose.assert_called_once_with(candidate, view["stamp_ns"], 1)
        assert "platform_reference XYZ" in window.video_status.text()
        window._select_detection(center)
        assert window.selected_pose_result is None and window.frozen_view is None
        window.node.clear_selected_pose.assert_called()
    else:
        window.node.show_selected_pose.assert_not_called()
    window.node.arm.assert_not_called()


def test_clicked_tf_preserves_platform_tilt_and_stops_on_invalidation():
    import numpy as np
    from builtin_interfaces.msg import Time
    angle = .15
    c, s = np.cos(angle), np.sin(angle)
    base = np.array([[c, 0, s, .2], [0, 1, 0, .3], [-s, 0, c, .4], [0, 0, 0, 1.]])
    candidate = {"position": [.01, .02, .1], "quaternion": [0., 0., 0., 1.]}
    message = gui.build_selected_pose_transform(base, candidate, Time(sec=123))
    assert message.header.frame_id == "base_link"
    assert message.child_frame_id == "item_teach_selected_item"
    expected = base.copy()
    expected[:3, 3] = (base @ np.array([.01, .02, .1, 1]))[:3]
    assert np.allclose(gui.transform_matrix(message), expected)
    node = SimpleNamespace(selection_lock=threading.RLock(), selected_pose=(1, message),
                           arm_epoch=1, yolo_enabled=True, native=SimpleNamespace(failed=False),
                           fatal_error=None, _validate_sources=MagicMock(), events=MagicMock(),
                           get_clock=lambda: SimpleNamespace(
                               now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=124))),
                           selected_pose_broadcaster=MagicMock())
    gui.ItemTeachNode._broadcast_selected_pose(node)
    assert node.selected_pose_broadcaster.sendTransform.call_count == 1
    for failure in ("epoch", "off", "native", "source"):
        node.selected_pose = (1, message)
        node.arm_epoch = 2 if failure == "epoch" else 1
        node.yolo_enabled = failure != "off"
        node.native.failed = failure == "native"
        node._validate_sources.side_effect = ValueError("Source changed") if failure == "source" else None
        gui.ItemTeachNode._broadcast_selected_pose(node)
        assert node.selected_pose is None
    assert node.selected_pose_broadcaster.sendTransform.call_count == 1


@pytest.mark.parametrize("running_kind", ["roi", "preview"])
def test_model_load_takes_next_slot_from_continuous_preview(window, monkeypatch, running_kind):
    node = window.node
    node.model_config = None
    node.applied = object()
    frame = {"width": 2, "height": 2, "rgb": bytes(12), "sequence": 1,
             "stamp_ns": 100_000_000_000}
    node.camera_snapshot = lambda: (frame, "Synthetic continuous RGB")
    node.roi_once = MagicMock(return_value=None)
    node.preview_once = MagicMock(return_value=None)
    window.job_busy = True  # A native preview is already in flight.

    def trust_dialog(*_):
        # Exercise the nested Qt-event-loop timing, including preview completion
        # before the operator finishes answering the trust question.
        window.job_results.put((running_kind, None, None))
        window._refresh_video()
        assert not window.job_busy
        assert not window.load_model_button.isEnabled()
        node.roi_once.assert_not_called()
        node.preview_once.assert_not_called()
        window.job_busy = True  # Also exercise an outstanding result at confirmation.
        return gui.QtWidgets.QMessageBox.Yes

    monkeypatch.setattr(gui.QtWidgets.QMessageBox, "question", trust_dialog)
    release = threading.Event()
    entered = threading.Event()
    path = window.model.text()
    metadata = node.model_metadata

    def inspect(selected):
        entered.set()
        assert release.wait(3), "Test did not release the synthetic model operation"
        node.model_config = {"path": selected, "task": "segment"}
        node.model_metadata = metadata
        return node.model_metadata

    node.inspect_model = MagicMock(side_effect=inspect)
    try:
        window._load_model()
        assert window.pending_model_path == path and window.model_load_reserved
        assert not window.yolo_toggle.isEnabled()
        assert not window.armed_toggle.isEnabled()
        node.inspect_model.assert_not_called()
        window._load_model()  # Repeated clicks cannot queue extra loads.
        window._refresh_video()
        node.roi_once.assert_not_called()
        window.job_results.put((running_kind, None, None))
        window._refresh_video()
        assert entered.wait(2)
        assert window.pending_model_path is None and window.job_busy
        assert window.model_load_reserved
        node.roi_once.assert_not_called()
        release.set()
        completed = window.job_results.get(timeout=3)
        window.job_results.put(completed)
        window._refresh_video()
        node.inspect_model.assert_called_once_with(path)
        assert not window.model_load_reserved
        assert window.load_model_button.isEnabled() and window.yolo_toggle.isEnabled()
        assert window.classes.item(0).text() == "1: part"
        assert not window.yolo_toggle.isChecked() and not node.yolo_enabled
        # Drain the automatically resumed ROI job before enabling detection.
        completed = window.job_results.get(timeout=3)
        window.job_results.put(completed)
        window._refresh_video()
        node.roi_once.assert_called_once()
        window.yolo_toggle.setChecked(True)
        assert node.yolo_enabled
        assert "Model load busy" not in window.status.toPlainText()
    finally:
        release.set()


def test_declining_model_trust_resumes_roi_without_loading(window, monkeypatch):
    window.node.applied = object()
    frame = {"width": 2, "height": 2, "rgb": bytes(12), "sequence": 1,
             "stamp_ns": 100_000_000_000}
    window.node.camera_snapshot = lambda: (frame, "Synthetic RGB")
    window.node.roi_once = MagicMock(return_value=None)
    window.node.inspect_model = MagicMock()

    def decline(*_):
        window._refresh_video()
        window.node.roi_once.assert_not_called()
        return gui.QtWidgets.QMessageBox.No

    monkeypatch.setattr(gui.QtWidgets.QMessageBox, "question", decline)
    window._load_model()
    assert not window.model_load_reserved and window.pending_model_path is None
    assert window.load_model_button.isEnabled() and window.yolo_toggle.isEnabled()
    window._refresh_video()
    completed = window.job_results.get(timeout=3)
    window.job_results.put(completed)
    window._refresh_video()
    window.node.roi_once.assert_called_once()
    window.node.inspect_model.assert_not_called()


@pytest.mark.parametrize("outcome", ["changed_selection", "close", "worker_failure"])
def test_pending_model_is_cancelled_on_invalidation(window, monkeypatch, outcome):
    monkeypatch.setattr(gui.QtWidgets.QMessageBox, "question",
                        lambda *_: gui.QtWidgets.QMessageBox.Yes)
    window.node.inspect_model = MagicMock()
    window.node.get_logger = MagicMock()
    window.job_busy = True
    window._load_model()
    assert window.pending_model_path is not None
    if outcome == "changed_selection":
        window.model.setText("changed.pt")
    elif outcome == "close":
        window.close()
    else:
        window.node.native.failed = True
    window.job_results.put(("roi", None, None))
    window._refresh_video()
    window.node.inspect_model.assert_not_called()
    assert window.pending_model_path is None


def test_failed_model_load_unlocks_controls_without_retry(window, monkeypatch):
    monkeypatch.setattr(gui.QtWidgets.QMessageBox, "question",
                        lambda *_: gui.QtWidgets.QMessageBox.Yes)
    window.node.inspect_model = MagicMock(side_effect=ValueError("Select a non-empty .pt model"))
    window._load_model()
    completed = window.job_results.get(timeout=3)
    window.job_results.put(completed)
    window._refresh_video()
    assert not window.model_load_reserved and window.pending_model_path is None
    assert window.load_model_button.isEnabled() and window.yolo_toggle.isEnabled()
    assert window.node.model_config is None
    assert "Select a non-empty .pt model" in window.status.toPlainText()
    window._refresh_video()
    window.node.inspect_model.assert_called_once()


@pytest.fixture
def paired_teach(window, tmp_path, monkeypatch):
    settings = {
        "item": {"name": "paired_part"}, "model_task": "segment", "geometry_source": "mask",
        "quality": dict(core.QUALITY_DEFAULTS),
        "motion": dict(zip(core.MOTION_FIELDS, [90., 50., 60., 100.])),
        "timing": {"pick_settling": .5}, "gripper": {"use_grip": True, "grip_onpick": True},
        "retry": {"retry_limit": 3},
        "geometry": {"height": 80., "width": 40., "tolerance": 5., "pickdepth_radius": 45.},
        "yolo": {"confidence": .63, "iou": .35, "max_detections": 17,
                 "image_size": 1280, "class_ids": [1]},
    }
    home = core.record_home(core.JOINT_NAMES, [.1]*6, 100, 0, now_ns=100_100_000_000,
                            robot_ip="192.168.20.204", publisher="/dobot_bringup_ros2")
    source = tmp_path / "trusted-test.pt"
    source.write_bytes(b"Synthetic pair; never deserialize these bytes")
    path, profile = core.save_item_profile(settings, home, source, root=tmp_path)
    metadata = {**window.node.model_metadata, "sha256": profile["model"]["sha256"]}
    monkeypatch.setattr(gui, "load_item_profile", lambda p: core.load_item_profile(p, root=tmp_path))
    monkeypatch.setattr(gui.QtWidgets.QFileDialog, "getOpenFileName", lambda *_: (str(path), ""))
    question = MagicMock(return_value=gui.QtWidgets.QMessageBox.Yes)
    monkeypatch.setattr(gui.QtWidgets.QMessageBox, "question", question)
    monkeypatch.setattr(gui, "write_item_ui_state", MagicMock())

    def inspect(selected, *, expected_sha256):
        assert selected == str(path.with_suffix(".pt"))
        assert expected_sha256 == profile["model"]["sha256"]
        window.node.model_config = {"path": selected, "sha256": expected_sha256,
                                    "task": metadata["task"]}
        window.node.model_metadata = metadata
        return metadata
    window.node.inspect_model = MagicMock(side_effect=inspect)
    return path, profile, settings, metadata, question


def finish_model_job(window):
    result = window.job_results.get(timeout=3)
    window.job_results.put(result)
    window._refresh_video()


@pytest.mark.parametrize("busy_preview", [False, True])
def test_explicit_teach_load_automatically_loads_exact_pair(window, paired_teach, busy_preview):
    path, profile, settings, _, question = paired_teach
    window.job_busy = busy_preview
    window._load_dialog()
    assert window.model_load_reserved
    assert not window.load_teach_button.isEnabled()
    assert not window.yolo_toggle.isChecked() and not window.armed_toggle.isChecked()
    window._load_dialog()
    window._load_model()  # Neither entry point can enqueue a duplicate load.
    question.assert_called_once()  # Combined replacement/trust dialog, no second Load Model click.
    assert "execute code" in question.call_args.args[2]
    if busy_preview:
        window.node.inspect_model.assert_not_called()
        window.job_results.put(("roi", None, None))
        window._refresh_video()
    finish_model_job(window)
    window.node.inspect_model.assert_called_once_with(
        str(path.with_suffix(".pt")), expected_sha256=profile["model"]["sha256"])
    assert window.model.text() == str(path.with_suffix(".pt"))
    assert window._selected_classes() == [1] and window.classes.item(0).text() == "1: part"
    assert window._settings() == settings
    assert window.saved_path == path and window.send.isEnabled()
    assert not window.node.yolo_enabled and not window.armed_toggle.isChecked()
    assert not window.model_load_reserved and window.load_teach_button.isEnabled()
    assert "Item teach and paired model loaded" in window.status.toPlainText()


def test_teach_prefill_never_loads_weights(window, paired_teach):
    path, _, settings, _, question = paired_teach
    window._load(path, prefill=True)
    assert window._settings() == settings
    assert window.saved_path is None and not window.send.isEnabled()
    window.node.inspect_model.assert_not_called()
    question.assert_not_called()
    assert not window.model_load_reserved and not window.node.yolo_enabled


def test_declining_pair_confirmation_preserves_form_and_does_not_load(window, paired_teach):
    _, _, _, _, question = paired_teach
    before = window.model.text(), window.name.text(), window.home
    question.return_value = gui.QtWidgets.QMessageBox.No
    window._load_dialog()
    assert (window.model.text(), window.name.text(), window.home) == before
    window.node.inspect_model.assert_not_called()
    assert not window.model_load_reserved and window.load_teach_button.isEnabled()


@pytest.mark.parametrize("stage", ["before", "queued", "completion"])
def test_changed_or_missing_pair_blocks_model_load_without_retry(window, paired_teach, stage):
    path, _, _, _, _ = paired_teach
    if stage == "before":
        path.with_suffix(".pt").unlink()
    window.job_busy = stage != "completion"
    window._load_dialog()
    if stage == "queued":
        path.with_suffix(".pt").write_bytes(b"changed model")
        window.job_results.put(("roi", None, None))
        window._refresh_video()
    if stage == "completion":
        result = window.job_results.get(timeout=3)
        path.write_text(path.read_text() + "\n# changed after model inspection\n")
        window.job_results.put(result)
        window._refresh_video()
    elif stage == "queued":
        finish_model_job(window)
    if stage != "completion":
        window.node.inspect_model.assert_not_called()
    else:
        assert window.node.model_config is None and window.saved_path is None
    assert not window.model_load_reserved and window.load_teach_button.isEnabled()
    assert not window.node.yolo_enabled and not window.armed_toggle.isChecked()


@pytest.mark.parametrize("mismatch", ["task", "classes", "geometry_sources"])
def test_paired_model_must_match_saved_task_classes_and_geometry(window, paired_teach, mismatch):
    _, _, _, metadata, _ = paired_teach
    metadata[mismatch] = {"task": "obb", "classes": {"4": "other"},
                          "geometry_sources": ["obb"]}[mismatch]
    window._load_dialog()
    finish_model_job(window)
    assert window.node.model_config is None
    assert window.saved_path is None and not window.send.isEnabled()
    assert "Model load failed" in window.status.toPlainText()
    window.node.inspect_model.assert_called_once()
    assert not window.node.yolo_enabled and not window.armed_toggle.isChecked()


def test_scaled_letterbox_click_and_frozen_measurement(window):
    item = {"source_index": 0, "class_id": 1, "class_name": "part", "confidence": .8,
            "polygon": [[270, 220], [370, 220], [370, 260], [270, 260]],
            "rectangle": [[270, 220], [370, 220], [370, 260], [270, 260]],
            "measurement": {"length_mm": 80., "width_mm": 32.}, "measurement_error": ""}
    view = {"rgb": bytes(640 * 480 * 3), "width": 640, "height": 480,
            "stamp_ns": 100_000_000_000, "sequence": 1, "preview_mode": "all",
            "metadata": {"detections": [item], "inference_ms": 10.}}
    window.yolo_toggle.setChecked(True)
    window.node.last_view = view
    window._refresh_video()
    center = gui.QtCore.QPointF(window.video.contentsRect().center())
    # Centering uses width/height rather than QRect's inclusive right/bottom edge.
    center += gui.QtCore.QPointF(.5, .5)
    mapped = gui.image_click(center, window.video, 640, 480)
    assert abs(mapped.x() - 320) < 2 and abs(mapped.y() - 240) < 2
    window._select_detection(center)
    assert window.frozen_view is view
    assert window.selected_detection is item
    assert window.inputs["height"].text() == ""  # No automatic field overwrite/tolerance guess.
    window.node.last_view = {**view, "sequence": 2, "metadata": {"detections": []}}
    window._refresh_video()
    assert window.displayed_view is view  # Frame-local ID cannot jump to a newer detection.
    assert "Frozen frame" in window.video_status.text()
    window.inputs["height"].setText("80")
    assert window.frozen_view is None and not window.node.yolo_enabled
    window._apply_live_detection_settings()
    assert window.node.yolo_enabled
    window.displayed_view = view
    window._select_detection(center)
    window._select_detection(center)  # Second image click replaces Resume Live.
    assert window.frozen_view is None and window.selected_detection is None
    assert not hasattr(window, "resume_live")
    window.yolo_toggle.setChecked(False)

    label = gui.QtWidgets.QLabel()
    label.resize(800, 600)
    label.setAlignment(gui.QtCore.Qt.AlignCenter)
    label.setPixmap(gui.QtGui.QPixmap(800, 400))
    assert gui.image_click(gui.QtCore.QPointF(400, 99), label, 1920, 960) is None
    point = gui.image_click(gui.QtCore.QPointF(400, 300), label, 1920, 960)
    assert point == gui.QtCore.QPointF(960, 480)
    label.close()


def test_unavailable_measurement_keeps_detection_and_explains_reason(window):
    item = {"source_index": 0, "class_name": "part", "confidence": .8,
            "polygon": [[10, 10], [630, 10], [630, 470], [10, 470]],
            "rectangle": [[10, 10], [630, 10], [630, 470], [10, 470]],
            "measurement": None, "measurement_error": "Apply station calibration"}
    view = {"rgb": bytes(640 * 480 * 3), "width": 640, "height": 480,
            "stamp_ns": 100_000_000_000, "sequence": 1, "preview_mode": "all",
            "metadata": {"detections": [item]}}
    window.node.last_view = view
    window.yolo_toggle.setChecked(True)
    window._refresh_video()
    window._select_detection(gui.QtCore.QPointF(window.video.contentsRect().center()))
    window._refresh_video()
    assert window.frozen_view is view
    assert "Apply station calibration" in window.video_status.text()


def test_roi_is_shown_without_yolo_and_hidden_when_stale(window):
    frame = {"rgb": bytes(640 * 480 * 3), "width": 640, "height": 480,
             "stamp_ns": 100_000_000_000, "sequence": 1}
    view = {**frame, "preview_mode": "roi",
            "metadata": {"roi_overlay": {"visible": True, "reason": ""}}}
    window.node.applied = object()
    window.node.camera_snapshot = lambda: (frame, "RGB live")
    window.node.roi_once = MagicMock()
    window.node.last_view = view
    window.last_preview_sequence = 1
    window._refresh_video()
    assert not window.yolo_toggle.isChecked()
    assert window.displayed_view is view
    assert "Loaded Bin ROI" in window.video_status.text()
    window.node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=101_000_000_000))
    window._refresh_video()
    assert window.displayed_view is frame
    assert "overlay frame is stale" in window.video_status.text()


@pytest.mark.parametrize("mode", ["all", "filtered"])
def test_slow_inference_keeps_annotated_snapshot_and_its_roi(window, mode):
    frame = {"rgb": bytes(640 * 480 * 3), "width": 640, "height": 480,
             "stamp_ns": 100_030_000_000, "sequence": 2}
    metadata = {"roi_overlay": {"visible": True, "reason": ""},
                "detections": [{}] * 23, "candidates": [{}] * 4, "count": 23,
                "inference_ms": 800., "geometry_sources": ["mask"]}
    annotated = {**frame, "rgb": bytes([100]) * (640 * 480 * 3),
                 "stamp_ns": 99_100_000_000, "sequence": 1,
                 "preview_mode": mode, "metadata": metadata}
    window.yolo_toggle.setChecked(True)
    window.node.camera_snapshot = lambda: (frame, "RGB live")
    window.node.last_view = annotated
    window.last_preview_sequence = 2
    window._refresh_video()
    assert window.displayed_view is annotated  # Not the newer, unannotated raw RGB.
    text = window.video_status.text()
    assert "RESULT SNAPSHOT" in text and "STALE Frame age 1.00s" in text
    assert "inference 800.0ms" in text
    assert "same result snapshot, not a live projection" in text
    assert ("DETECTIONS: 23" if mode == "all" else "FILTERED: 4 valid picks") in text
    window.job_results.put(("preview", None, RuntimeError("Synthetic TF unavailable")))
    window._refresh_video()
    assert window.displayed_view is annotated
    assert "Preview blocked: Synthetic TF unavailable" in window.video_status.text()
    # A newer completed result replaces the snapshot naturally; no freezing or retry.
    newer = {**annotated, "stamp_ns": 100_000_000_000, "sequence": 3}
    window.node.last_view = newer
    window.job_results.put(("preview", newer, None))
    window._refresh_video()
    assert window.displayed_view is newer
    assert "RESULT SNAPSHOT" not in window.video_status.text()
    assert "Preview blocked" not in window.video_status.text()
    window.node.arm.assert_not_called()
    assert window.node.service is None


def test_raw_rgb_never_claims_last_result_detection_count(window):
    frame = {"rgb": bytes(640 * 480 * 3), "width": 640, "height": 480,
             "stamp_ns": 100_000_000_000, "sequence": 1}
    window.yolo_toggle.setChecked(True)
    window.node.camera_snapshot = lambda: (frame, "RGB live")
    window.last_preview_sequence = 1
    window.preview_status = "ALL DETECTIONS: 23 | click to measure"
    window._refresh_video()
    assert window.displayed_view is frame
    assert "waiting for annotated result" in window.video_status.text()
    assert "DETECTIONS: 23" not in window.video_status.text()


def automatic_station_fixture(window, monkeypatch):
    """Mock validated artifacts/subscriptions, never run camera or model processes."""
    station_write, prefix_write = MagicMock(), MagicMock()
    monkeypatch.setattr(gui, "write_item_station_state", station_write)
    monkeypatch.setattr(gui, "write_item_preview_state", prefix_write)

    def apply(platform, bin_path):
        window.node.applied = SimpleNamespace(
            camera=SimpleNamespace(settings=SimpleNamespace(camera_prefix="station_camera")))
        window.node.bin_artifact = object()
        window.node.camera_prefix = "station_camera"
        window.node.yolo_enabled = False
    window.node.apply_station = MagicMock(side_effect=apply)
    return station_write, prefix_write


def test_station_files_automatically_enable_roi_when_stream_arrives(window, monkeypatch):
    station_write, prefix_write = automatic_station_fixture(window, monkeypatch)
    buttons = [button.text() for button in window.findChildren(gui.QtWidgets.QPushButton)]
    assert "Apply Station + Bin ROI" not in buttons
    window.platform_path.setText("/selected/platform_calibration_test.yaml")
    window.node.apply_station.assert_not_called()
    assert "select platform and bin teach" in window.station_status.text()
    window.bin_path.setText("/selected/bin_teach_test.yaml")
    window.node.apply_station.assert_called_once_with(
        "/selected/platform_calibration_test.yaml", "/selected/bin_teach_test.yaml")
    assert window.camera_prefix.text() == "station_camera"
    station_write.assert_called_once()
    prefix_write.assert_called_once()
    assert "automatically displayed" in window.station_status.text()
    window._job = MagicMock()
    window.node.roi_once = MagicMock()
    window._refresh_video()  # Artifacts ready, camera not publishing yet.
    window._job.assert_not_called()
    frame = {"rgb": bytes(640 * 480 * 3), "width": 640, "height": 480,
             "stamp_ns": 100_000_000_000, "sequence": 1}
    window.node.camera_snapshot = lambda: (frame, "RGB live")
    window._refresh_video()  # First image triggers ROI-only rendering without a button.
    window._job.assert_called_once_with("roi", window.node.roi_once)
    window._refresh_video()
    window.node.apply_station.assert_called_once()  # No timer-driven reapplication/reconnect.
    assert not window.node.yolo_enabled and not window.yolo_toggle.isChecked()
    window.node.arm.assert_not_called()


def test_station_change_clears_old_overlay_and_stops_yolo(window, monkeypatch):
    automatic_station_fixture(window, monkeypatch)
    window.platform_path.setText("/selected/platform_calibration_test.yaml")
    window.bin_path.setText("/selected/bin_teach_first.yaml")
    window.node.camera_prefix = "bin_camera"
    window.camera_prefix.setText("bin_camera")
    window.yolo_toggle.setChecked(True)
    window.node.last_view = window.frozen_view = {"previous": "overlay"}
    window.bin_path.setText("/selected/bin_teach_second.yaml")
    assert window.node.last_view is None and window.frozen_view is None
    assert not window.yolo_toggle.isChecked() and not window.node.yolo_enabled
    assert window.node.apply_station.call_count == 2
    window.node.disarm.assert_called()
    window.node.arm.assert_not_called()


@pytest.mark.parametrize("error", [
    FileNotFoundError("Missing YAML"), ValueError("Camera SHA-256 mismatch"),
])
def test_invalid_station_hides_roi_without_repeated_apply_or_modal(window, monkeypatch, error):
    station_write, prefix_write = automatic_station_fixture(window, monkeypatch)
    window.node.apply_station.side_effect = error
    window.node.last_view = {"previous": "overlay"}
    window.platform_path.setText("/selected/platform_calibration_test.yaml")
    window.bin_path.setText("/selected/bin_teach_test.yaml")
    assert str(error) in window.station_status.text()
    assert window.node.applied is None and window.node.bin_artifact is None
    assert window.node.last_view is None
    station_write.assert_not_called()
    prefix_write.assert_not_called()
    gui.QtWidgets.QMessageBox.warning.assert_not_called()
    for _ in range(3):
        window._refresh_video()
    window.node.apply_station.assert_called_once()
    window.node.arm.assert_not_called()


def test_restored_station_files_connect_readonly_preview_without_apply(window, monkeypatch):
    automatic_station_fixture(window, monkeypatch)
    monkeypatch.setattr(gui, "load_package_ui_state", lambda _: SimpleNamespace(
        item_platform_filename="platform_calibration_saved.yaml",
        item_bin_filename="bin_teach_saved.yaml", item_profile_filename=None,
        item_preview_camera_prefix="previous_prefix"))
    restored = gui.ItemTeachWindow(window.node)
    restored.timer.stop()
    try:
        window.node.apply_station.assert_called_once_with(
            str(gui.workspace_root() / "calibration/platform_calibration_saved.yaml"),
            str(gui.workspace_root() / "offline_teach/bin_teach/bin_teach_saved.yaml"))
        assert restored.camera_prefix.text() == "station_camera"
        assert restored.saved_path is None and restored.home is None
        assert not restored.yolo_toggle.isChecked() and not restored.armed_toggle.isChecked()
        window.node.arm.assert_not_called()
    finally:
        restored.close()


def test_explicit_reselection_can_revalidate_same_artifact(window, monkeypatch):
    automatic_station_fixture(window, monkeypatch)
    window.platform_path.setText("/selected/platform_calibration_test.yaml")
    window.bin_path.setText("/selected/bin_teach_test.yaml")
    monkeypatch.setattr(gui.QtWidgets.QFileDialog, "getOpenFileName",
                        lambda *args: (window.bin_path.text(), "YAML"))
    window._choose_station_file(window.bin_path, Path("/selected"))
    assert window.node.apply_station.call_count == 2
    assert window.node.applied is not None
