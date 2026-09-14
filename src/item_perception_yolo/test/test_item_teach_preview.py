"""Read-only teaching stages and image selection; no ROS or physical camera launch."""

from pathlib import Path
import threading
import zipfile
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
import yaml

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
        show_simulated_poses=MagicMock(),
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


def test_teach_has_no_controller_validation_action_or_request_state(window):
    import ast
    source = Path(gui.__file__).read_text()
    buttons = window.findChildren(gui.QtWidgets.QPushButton)
    assert all("Controller" not in button.text() for button in buttons)
    for name in ("send", "request", "request_started", "_send_controller", "_poll_request"):
        assert not hasattr(window, name)
    calls = [node for node in ast.walk(ast.parse(source)) if isinstance(node, ast.Call)]
    assert not any(isinstance(node.func, ast.Attribute) and node.func.attr == "create_client"
                   for node in calls)
    assert "/robot_controller/" not in source


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
    assert "pose_candidates" in window.inputs and "retry_limit" not in window.inputs
    assert window.simulate_button.text() == "Simulate Trigger"
    assert not window.simulate_button.isCheckable()
    layout = window.simulate_button.parent().layout().itemAt(0).layout()
    assert [layout.itemAt(i).widget() for i in range(3)] == [
        window.yolo_toggle, window.simulate_button, window.armed_toggle]


def simulation_setup(window):
    from item_perception_interfaces.srv import GetItemPoses
    from item_perception_interfaces.msg import ItemCandidate
    for name, value in (("height", "80"), ("width", "32"), ("tolerance", "2")):
        window.inputs[name].setText(value)
    window._populate_classes({1: "part"}, [1])
    window.yolo_toggle.setChecked(True)
    window.saved_path = Path("saved.yaml")
    window.node._validate_pose_profile = MagicMock(return_value=(
        {"retry": {"pose_candidates": 3}}, "a"*64))
    candidate = ItemCandidate(class_name="part", priority=1, length=.08, width=.032,
                              filtered_camera_depth=.7, accepted_depth_count=100,
                              rejected_depth_count=2)
    candidate.pose.position.z = .1
    candidate.pose.orientation.w = 1.
    response = GetItemPoses.Response(success=True, status="SHORTAGE", message="Returned 1 of 3 requested",
                                     valid_count=1, detected_count=20, candidates=[candidate])
    view = {"width": 200, "height": 100, "rgb": bytes([80, 100, 120])*20000,
            "depth_rgb": bytes([160, 50, 90])*20000, "stamp_ns": 100_100_000_000,
            "depth_stamp_ns": 100_100_000_000, "sequence": 10, "preview_mode": "filtered",
            "metadata": {"candidates": [], "count": 20, "inference_ms": 50.,
                         "roi_overlay": {"visible": True, "reason": ""}}}
    window.node.simulate_trigger = MagicMock(return_value={"response": response, "view": view})
    window.node.validate_simulation_view = MagicMock()
    window.node.clear_selected_pose.reset_mock()
    return response, view


def test_simulate_button_reserves_next_slot_and_freezes_only_returned_pair(window):
    response, view = simulation_setup(window)
    window.job_busy = True  # A running all-detection preview must yield the next slot.
    window.simulate_button.click()
    assert window.pending_simulation and not window.simulate_button.isEnabled()
    window.node.simulate_trigger.assert_not_called()
    window._simulate_trigger()  # Duplicate action is ignored, not a second request.
    window.job_results.put(("preview", None, None))
    window._refresh_video()
    assert window.simulation_busy
    finish_model_job(window)
    assert window.frozen_view["rgb"] == view["rgb"]
    assert window.frozen_view["depth_rgb"] == view["depth_rgb"]
    assert window.simulation_response is response
    assert "SIMULATED SHORTAGE" in window.rgb_feedback.text()
    assert "SIMULATED SHORTAGE" in window.depth_feedback.text()
    assert "P1 part" in window.rgb_feedback.text() and "100 accepted / 2 rejected" in window.depth_feedback.text()
    window.node.simulate_trigger.assert_called_once()
    window.node.arm.assert_not_called()
    window.node.show_selected_pose.assert_not_called()
    window.node.show_simulated_poses.assert_called_once_with(response, view)
    assert "RViz TF: base_link → item_teach_candidate_1" in window.rgb_feedback.text()
    assert window.node.service is None and not window.armed_toggle.isChecked()
    assert window.saved_path == Path("saved.yaml")
    old_rgb, old_depth = window.video.pixmap().toImage(), window.depth_video.pixmap().toImage()
    window.node.last_view = {**view, "rgb": bytes([3, 4, 5])*20000, "depth_rgb": bytes(60000)}
    window._refresh_video()
    assert window.video.pixmap().toImage() == old_rgb
    assert window.depth_video.pixmap().toImage() == old_depth
    # Status/margins are not a resume action, but a real image click releases the batch.
    window._select_detection(gui.QtCore.QPointF(-1, -1))
    assert window.frozen_view is not None
    window._select_detection(gui.QtCore.QPointF(window.video.contentsRect().center()))
    assert window.frozen_view is None and window.simulation_response is None


@pytest.mark.parametrize("invalidate", ["field", "off", "arming", "resume"])
def test_simulation_inflight_result_is_discarded_on_changes(window, invalidate):
    simulation_setup(window)
    window.simulate_button.click()
    window._refresh_video()
    completed = window.job_results.get(timeout=3)
    action = window.node.simulate_trigger.call_args.kwargs["cancelled"]
    assert not action()
    if invalidate == "field":
        window.inputs["pose_candidates"].setText("4")
    elif invalidate == "off":
        window.yolo_toggle.setChecked(False)
    elif invalidate == "arming":
        window._toggle_armed(False)
    else:
        window._resume_live()
    assert action()
    window.job_results.put(completed)
    window._refresh_video()
    assert window.simulation_response is None and window.frozen_view is None
    assert not window.simulation_busy and window.simulate_button.isEnabled()
    window.node.show_selected_pose.assert_not_called()
    window.node.show_simulated_poses.assert_not_called()


@pytest.mark.parametrize("failure", ["unsaved", "off", "fields", "profile", "busy", "no_items"])
def test_simulation_requirements_and_empty_or_failed_batch(window, failure):
    response, _view = simulation_setup(window)
    if failure == "unsaved":
        window.saved_path = None
    elif failure == "off":
        window.yolo_toggle.setChecked(False)
    elif failure == "fields":
        window.inputs["height"].setText("")
    elif failure == "profile":
        window.node._validate_pose_profile.side_effect = ValueError("Saved model hash changed")
    elif failure == "busy":
        response.success, response.status, response.message = False, "BUSY", "One request active"
        response.candidates = []
        window.node.simulate_trigger.return_value["view"] = None
    else:
        response.status, response.message = "NO_VALID_ITEMS", "Returned 0 of 3 requested"
        response.candidates, response.valid_count = [], 0
    window.simulate_button.click()
    if failure in ("busy", "no_items"):
        window._refresh_video()
        finish_model_job(window)
        if failure == "no_items":
            assert "NO_VALID_ITEMS" in window.rgb_feedback.text()
            assert window.frozen_view is not None and not window.simulation_response.candidates
        else:
            assert window.frozen_view is None and "One request active" in window.status.toPlainText()
    else:
        assert window.pending_simulation is None
        window.node.simulate_trigger.assert_not_called()
    assert window.simulate_button.isEnabled()
    window.node.arm.assert_not_called()


def test_simulation_freeze_clears_if_profile_or_station_changes(window):
    simulation_setup(window)
    window.simulate_button.click()
    window._refresh_video()
    finish_model_job(window)
    assert window.frozen_view is not None
    window.node.validate_simulation_view.side_effect = ValueError("Station source hash changed")
    window._refresh_video()
    assert window.frozen_view is None and window.simulation_response is None
    assert "Simulated batch cleared: Station source hash changed" in window.status.toPlainText()
    window.node.clear_selected_pose.assert_called()


def test_simulation_tf_rejection_never_keeps_old_preview(window):
    simulation_setup(window)
    window.node.show_simulated_poses.side_effect = ValueError("Simulated batch snapshot is stale")
    window.simulate_button.click()
    window._refresh_video()
    finish_model_job(window)
    assert window.frozen_view is None and window.simulation_response is None
    assert "Simulate Trigger preview rejected" in window.status.toPlainText()
    window.node.clear_selected_pose.assert_called()


def test_cancel_queued_simulation_does_not_run_it_after_preview_completion(window):
    simulation_setup(window)
    window.job_busy = True
    window.simulate_button.click()
    window._resume_live()
    window.job_results.put(("preview", None, None))
    window._refresh_video()
    window.node.simulate_trigger.assert_not_called()
    assert window.simulation_response is None and window.simulate_button.isEnabled()


def test_simulation_batch_feedback_does_not_expand_with_all_1000_candidates(window):
    response, _ = simulation_setup(window)
    from copy import deepcopy
    response.candidates = [deepcopy(response.candidates[0]) for _ in range(20)]
    for index, candidate in enumerate(response.candidates, 1):
        candidate.priority = index
    response.valid_count = 20
    response.message, response.status = "Returned 20 of 20 requested", "OK"
    window.simulate_button.click()
    window._refresh_video()
    finish_model_job(window)
    assert "P3 part" in window.rgb_feedback.text() and "P4 part" not in window.rgb_feedback.text()
    assert "17 more poses" in window.depth_feedback.text()
    assert "P20 part" in window.status.toPlainText()
    assert "item_teach_candidate_1…20" in window.rgb_feedback.text()
    window.node.show_simulated_poses.assert_called_once()
    assert len(window.node.show_simulated_poses.call_args.args[0].candidates) == 20


def test_visual_first_layout_has_one_settings_column_and_persistent_actions(window):
    window.show()
    gui.QtWidgets.QApplication.processEvents()
    sidebar, preview = window.workspace_split.sizes()
    assert preview > sidebar * 2
    assert isinstance(window.settings_column, gui.QtWidgets.QVBoxLayout)
    groups = [window.settings_column.itemAt(i).widget()
              for i in range(window.settings_column.count())]
    assert len(groups) == 9
    assert all(isinstance(g, gui.QtWidgets.QGroupBox) for g in groups)
    assert "Camera" in groups[0].title() and "Item / model" in groups[1].title()
    assert "YOLO" in groups[2].title() and "Item size" in groups[3].title()
    assert all(a.geometry().bottom() < b.geometry().top()
               for a, b in zip(groups, groups[1:]))
    assert window.image_split.orientation() == gui.QtCore.Qt.Horizontal
    assert abs(window.video.width() - window.depth_video.width()) <= 2
    assert not window.preview_help.isVisible()
    assert not window.status.isVisible()
    window.activity_toggle.setChecked(True)
    assert window.status.isVisible()
    window._message("Synthetic feedback")
    assert window.activity_summary.text() == "Synthetic feedback"
    window.settings_scroll.verticalScrollBar().setValue(10000)
    save = next(button for button in window.findChildren(gui.QtWidgets.QPushButton)
                if button.text() == "Save Item Teach…")
    assert not window.settings_scroll.isAncestorOf(save)
    assert save.isVisible()
    assert not window.node.yolo_enabled and window.node.service is None


@pytest.mark.parametrize("window_size", [(1600, 1000), (1200, 800)])
def test_feedback_uses_top_black_band_without_painting_camera_pixels(window, window_size):
    width, height = 1920, 1080
    view = {"rgb": bytes([40, 110, 70]) * (width * height),
            "depth_rgb": bytes([160, 80, 20]) * (width * height),
            "width": width, "height": height, "stamp_ns": 99_000_000_000,
            "depth_stamp_ns": 99_050_000_000, "sequence": 1, "preview_mode": "all",
            "metadata": {"detections": [], "inference_ms": 555.}}
    window.node.last_view = view
    window.resize(*window_size)
    window.show()
    # Let Qt lay out wrapped status bands, then render at the resulting image size.
    for _ in range(3):
        window._refresh_video()
        gui.QtWidgets.QApplication.processEvents()
    assert "RESULT SNAPSHOT — DETECTIONS: 0" in window.rgb_feedback.text()
    assert "STALE Frame age 1.10s | inference 555.0ms" in window.rgb_feedback.text()
    assert "conf 0.25 / IoU 0.7 / cap 100" in window.rgb_feedback.text()
    assert "Depth age: 1.05s" in window.depth_feedback.text()
    for feedback, image, pixels in (
        (window.rgb_feedback, window.video, view["rgb"]),
        (window.depth_feedback, window.depth_video, view["depth_rgb"]),
    ):
        assert feedback.isVisible() and feedback.wordWrap()
        assert feedback.textFormat() == gui.QtCore.Qt.PlainText
        assert feedback.parent() is image.parent()
        assert feedback.geometry().bottom() < image.geometry().top()
        assert feedback.height() >= feedback.heightForWidth(feedback.width())
        expected = gui.QtGui.QImage(pixels, width, height, width * 3,
                                   gui.QtGui.QImage.Format_RGB888)
        expected = gui.QtGui.QPixmap.fromImage(expected).scaled(
            image.size(), gui.QtCore.Qt.KeepAspectRatio, gui.QtCore.Qt.SmoothTransformation)
        assert image.pixmap().toImage() == expected.toImage()  # No burnt-in text/background.
    center = gui.QtCore.QPointF(window.video.contentsRect().center())
    mapped = gui.image_click(center, window.video, width, height)
    assert abs(mapped.x() - width / 2) <= 4 and abs(mapped.y() - height / 2) <= 4
    assert gui.image_click(gui.QtCore.QPointF(0, 0), window.video, width, height) is None
    # Header clicks are not image clicks and cannot select/release a frozen target.
    clicked = MagicMock()
    window.video.clicked.connect(clicked)
    event = gui.QtGui.QMouseEvent(gui.QtCore.QEvent.MouseButtonPress, gui.QtCore.QPointF(20, 10),
                                 gui.QtCore.Qt.LeftButton, gui.QtCore.Qt.LeftButton,
                                 gui.QtCore.Qt.NoModifier)
    gui.QtWidgets.QApplication.sendEvent(window.rgb_feedback, event)
    clicked.assert_not_called()
    window.node.last_view = {**view, "depth_rgb": None, "depth_error": "Synthetic missing pair"}
    window._refresh_video()
    assert window.depth_feedback.isHidden() and not window.depth_feedback.text()
    assert "Synthetic missing pair" in window.depth_video.text()
    window.node.last_view = None
    window._refresh_video()
    assert window.rgb_feedback.isHidden() and not window.rgb_feedback.text()


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
    candidate = {"position": [.01, .02, .1], "quaternion": [0., 0., 0., 1.],
                 "filtered_camera_depth": .7, "accepted_depth_count": 100,
                 "rejected_depth_count": 2}
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
        for feedback in (window.rgb_feedback, window.depth_feedback):
            assert "FROZEN SELECTION" in feedback.text()
            assert "X / height: 80.00 mm" in feedback.text()
            assert "platform_reference XYZ [mm]: +10.00, +20.00, +100.00" in feedback.text()
        assert "100 accepted / 2 rejected" in window.depth_feedback.text()
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
    node = SimpleNamespace(selection_lock=threading.RLock(), selected_pose=(1, (message,), None),
                           arm_epoch=1, yolo_enabled=True, native=SimpleNamespace(failed=False),
                           fatal_error=None, _validate_sources=MagicMock(), events=MagicMock(),
                           get_clock=lambda: SimpleNamespace(
                               now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=124))),
                           selected_pose_broadcaster=MagicMock())
    gui.ItemTeachNode._broadcast_selected_pose(node)
    assert node.selected_pose_broadcaster.sendTransform.call_count == 1
    for failure in ("epoch", "off", "native", "source"):
        node.selected_pose = (1, (message,), None)
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
        "retry": {"pose_candidates": 3},
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
    from item_perception_yolo.item_teach_recovery import recover_item_fields
    monkeypatch.setattr(gui, "recover_item_fields", lambda p: recover_item_fields(p, root=tmp_path))
    monkeypatch.setattr(gui.QtWidgets.QFileDialog, "getOpenFileName", lambda *_: (str(path), ""))
    question = MagicMock(return_value=gui.QtWidgets.QMessageBox.Yes)
    monkeypatch.setattr(gui.QtWidgets.QMessageBox, "question", question)
    monkeypatch.setattr(gui, "write_item_ui_state", MagicMock())

    def inspect(selected, *, expected_sha256=None):
        assert selected == str(path.with_suffix(".pt"))
        assert expected_sha256 in (None, profile["model"]["sha256"])
        window.node.model_config = {"path": selected, "sha256": metadata["sha256"],
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
    assert window.saved_path == path
    assert not window.node.yolo_enabled and not window.armed_toggle.isChecked()
    assert not window.model_load_reserved and window.load_teach_button.isEnabled()
    assert "Item teach and paired model loaded" in window.status.toPlainText()


def test_teach_prefill_never_loads_weights(window, paired_teach):
    path, _, settings, _, question = paired_teach
    window._load(path, prefill=True)
    assert window._settings() == settings
    assert window.saved_path == path
    window.node.inspect_model.assert_not_called()
    question.assert_not_called()
    assert not window.model_load_reserved and not window.node.yolo_enabled


@pytest.mark.parametrize("prefill", [False, True])
def test_valid_loaded_profile_can_simulate_and_arm_without_save(window, paired_teach, prefill):
    path, profile, _, _, _ = paired_teach
    if prefill:
        window._load(path, prefill=True)
        window._load_model()  # Startup restoration never implicitly trusts/executes weights.
    else:
        window._load_dialog()
    finish_model_job(window)
    assert window.saved_path == path
    window.yolo_toggle.setChecked(True)
    window.node._validate_pose_profile = MagicMock(return_value=(profile, core.file_sha256(path)))
    window._simulate_trigger()
    assert window.pending_simulation[0] == path
    window.node._validate_pose_profile.assert_called_once_with(path)
    window.node.arm.assert_not_called()
    window._resume_live()
    window.armed_toggle.setChecked(True)
    window.node.arm.assert_called_once_with(path)
    assert window.armed_toggle.isChecked()
    assert not list(path.parent.glob(".*.previous.zip"))  # No redundant write.


def test_form_edits_preserve_loaded_save_target_and_rename_switches_target(
        window, paired_teach, monkeypatch, tmp_path):
    path, profile, _, _, question = paired_teach
    monkeypatch.setattr(gui.QtWidgets.QMessageBox, "information", MagicMock())
    window._load(path, prefill=True)
    target = window.save_target
    window.inputs["confidence"].setText("0.8")
    assert window.saved_path is None and window.save_target == target
    window._save()
    assert window.saved_path == path and window.save_target.path == path
    assert "Overwrite loaded item teach" in question.call_args.args[2]
    updated, _ = core.load_item_profile(path, root=tmp_path)
    assert updated["yolo"]["confidence"] == 0.8
    window.name.setText("new_item_name")
    window._save()
    new_path = window.saved_path
    assert new_path != path and window.save_target.item_name == "new_item_name"
    assert "Save a NEW" in question.call_args.args[2]
    assert core.load_item_profile(path, root=tmp_path)[0] == updated
    assert Path(window.model.text()) == path.with_suffix(".pt")  # No implicit model switch.
    window.inputs["pose_candidates"].setText("4")
    window._save()
    assert window.saved_path == new_path
    assert core.load_item_profile(new_path, root=tmp_path)[0]["retry"]["pose_candidates"] == 4
    assert not window.armed_toggle.isChecked()
    window.node.arm.assert_not_called()
    window.node.inspect_model.assert_not_called()


def test_cancel_overwrite_or_loading_model_never_writes_pair(window, paired_teach):
    path, _, _, _, question = paired_teach
    original = path.read_bytes()
    window._load(path, prefill=True)
    window.inputs["confidence"].setText("0.8")
    question.return_value = gui.QtWidgets.QMessageBox.No
    window._save()
    assert path.read_bytes() == original and window.saved_path is None
    target = window.save_target
    question.reset_mock()
    window._reserve_model_load()
    assert not window.save_button.isEnabled()
    window._save()
    question.assert_not_called()
    assert path.read_bytes() == original and window.save_target == target
    window._finish_model_load()
    assert window.save_button.isEnabled()


def test_external_change_save_failure_keeps_target_and_disarms(window, paired_teach, monkeypatch):
    path, _, _, _, _ = paired_teach
    window._load(path, prefill=True)
    target = window.save_target
    path.write_bytes(b"operator replaced the file")
    error = MagicMock()
    monkeypatch.setattr(window, "_error", error)
    window._save()
    assert window.saved_path is None and window.save_target == target
    assert path.read_bytes() == b"operator replaced the file"
    assert "changed externally" in str(error.call_args.args[1])
    window.node.disarm.assert_called()


def test_old_teach_requires_review_then_overwrites_with_backup(
        window, paired_teach, tmp_path, monkeypatch):
    path, profile, settings, _, _ = paired_teach
    profile["schema_version"] = 3
    profile["retry"] = {"retry_limit": 3}
    path.write_text(yaml.safe_dump(profile))
    original, model_original = path.read_bytes(), path.with_suffix(".pt").read_bytes()
    window._load(path, prefill=True)
    assert window.recovered_draft and window._settings() == settings
    assert window.saved_path is None
    window.node.inspect_model.assert_not_called()
    window._load_dialog()
    finish_model_job(window)
    assert window.recovered_draft and window._settings() == settings
    assert window.saved_path is None
    window.armed_toggle.setChecked(True)
    window.node.arm.assert_not_called()
    assert not window.armed_toggle.isChecked()
    monkeypatch.setattr(gui.QtWidgets.QMessageBox, "information", MagicMock())
    window._save()
    assert not window.recovered_draft and window.saved_path == path
    assert window.recovery_notice.isHidden()
    saved, _ = core.load_item_profile(window.saved_path, root=tmp_path)
    assert saved["schema_version"] == 4 and saved["retry"] == {"pose_candidates": 3}
    assert saved["home"] == profile["home"]
    assert core.settings_from_profile(saved) == settings
    assert path.read_bytes() != original and path.with_suffix(".pt").read_bytes() == model_original
    with zipfile.ZipFile(path.with_name(f".{path.stem}.previous.zip")) as backup:
        assert backup.read(path.name) == original
        assert backup.read(path.with_suffix(".pt").name) == model_original
    assert not window.node.yolo_enabled and not window.armed_toggle.isChecked()


def test_partial_recovery_clears_previous_form_values_and_unknown_booleans(window, paired_teach):
    path, profile, _, _, _ = paired_teach
    window.inputs["height"].setText("999")
    profile["geometry"]["height"] = ""
    profile["gripper"]["grip_onpick"] = None
    profile["gripper"]["use_grip"] = False
    profile["home"] = None
    profile["yolo"]["confidence"] = 40
    profile["retry"] = {}
    profile["model"]["sha256"] = "b" * 64
    path.write_text(yaml.safe_dump(profile))
    window._load_dialog()
    assert window.recovered_draft and not window.recovery_notice.isHidden()
    for key in ("height", "confidence", "pose_candidates"):
        assert window.inputs[key].text() == ""
    assert window.home is None and window.model.text() == ""
    assert window.inputs["grip_onpick"].checkState() == gui.QtCore.Qt.PartiallyChecked
    assert window.inputs["grip_onpick"].isEnabled()  # Can resolve unknown even with use_grip OFF.
    assert window.inputs["width"].text() == "40.0"
    assert not window.model_load_reserved and window.saved_path is None
    with pytest.raises(ValueError, match="grip_onpick is unknown"):
        window._settings()
    window.node.inspect_model.assert_not_called()
    window.node.arm.assert_not_called()


def test_corrupt_profile_prefill_opens_with_no_model_or_old_values(window, paired_teach):
    path, _, _, _, _ = paired_teach
    path.write_text("broken: [")
    window._load(path, prefill=True)
    assert window.recovered_draft and window.model.text() == "" and window.home is None
    assert window.profile_image_size is None and not window._selected_classes()
    assert window.task.currentIndex() == window.geometry_source.currentIndex() == -1
    for widget in window.inputs.values():
        if isinstance(widget, gui.QtWidgets.QLineEdit):
            assert widget.text() == ""
    assert window.saved_path is None and not window.node.yolo_enabled
    window.node.inspect_model.assert_not_called()
    with pytest.raises(ValueError, match="image_size is unknown"):
        window._yolo_settings()


@pytest.mark.parametrize("corrupt", [False, True])
def test_window_startup_recovers_named_old_or_corrupt_profile_without_execution(
        window, paired_teach, monkeypatch, corrupt):
    path, profile, _, _, _ = paired_teach
    profile["schema_version"] = 3
    profile["retry"] = {"retry_limit": 3}
    path.write_text("broken: [" if corrupt else yaml.safe_dump(profile))
    monkeypatch.setattr(gui, "item_directory", lambda: path.parent)
    monkeypatch.setattr(gui, "load_package_ui_state", lambda _: SimpleNamespace(
        item_profile_filename=path.name, item_preview_camera_prefix=None,
        item_platform_filename=None, item_bin_filename=None))
    restored = gui.ItemTeachWindow(window.node)
    restored.timer.stop()
    try:
        assert restored.recovered_draft and restored.saved_path is None
        assert not hasattr(restored, "send") and not restored.yolo_toggle.isChecked()
        assert restored.inputs["pose_candidates"].text() == ("" if corrupt else "3")
        window.node.inspect_model.assert_not_called()
        window.node.arm.assert_not_called()
    finally:
        restored.close()


def test_recovery_pair_changes_while_queued_never_loads_model(window, paired_teach):
    path, profile, _, _, _ = paired_teach
    profile["schema_version"] = 3
    profile["retry"] = {"retry_limit": 3}
    path.write_text(yaml.safe_dump(profile))
    window.job_busy = True
    window._load_dialog()
    path.with_suffix(".pt").write_bytes(b"changed")
    window.job_results.put(("roi", None, None))
    window._refresh_video()
    finish_model_job(window)
    assert window.saved_path is None and window.node.model_config is None
    window.node.inspect_model.assert_not_called()


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
    assert window.saved_path is None
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
            "depth_rgb": bytes([20, 50, 80]) * (640 * 480),
            "depth_stamp_ns": 100_010_000_000,
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
    window.node.last_view = {**view, "sequence": 2, "metadata": {"detections": []},
                             "depth_rgb": bytes([200, 0, 0]) * (640 * 480)}
    window._refresh_video()
    assert window.displayed_view is view  # Frame-local ID cannot jump to a newer detection.
    depth_image = window.depth_video.pixmap().toImage()
    assert depth_image.pixelColor(depth_image.width()//2, depth_image.height()//2) == \
        gui.QtGui.QColor(20, 50, 80)  # Frozen depth is not replaced with newer live data.
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


def automatic_station_fixture(window, monkeypatch, source_sha="a" * 64):
    """Mock validated artifacts/subscriptions, never run camera or model processes."""
    station_write, prefix_write = MagicMock(), MagicMock()
    monkeypatch.setattr(gui, "write_item_station_state", station_write)
    monkeypatch.setattr(gui, "write_item_preview_state", prefix_write)
    latest = SimpleNamespace(
        platform=SimpleNamespace(path=Path("/selected/platform_calibration_test.yaml"),
                                 sha256="a" * 64),
        camera=SimpleNamespace(path=Path("/selected/camera_to_hand_calibration_test.yaml"),
                               sha256="c" * 64))
    monkeypatch.setattr(gui, "latest_station_calibration", MagicMock(return_value=latest))

    def apply(platform, bin_path, *, expected_station=None):
        window.node.applied = SimpleNamespace(
            platform=SimpleNamespace(path=Path(platform), sha256="a" * 64),
            camera=SimpleNamespace(settings=SimpleNamespace(camera_prefix="station_camera")))
        window.node.bin_artifact = SimpleNamespace(
            path=Path(bin_path), source_platform_calibration_sha256=source_sha,
            source_platform_calibration_filename="platform_calibration_source.yaml")
        window.node.camera_prefix = "station_camera"
        window.node.yolo_enabled = False
    window.node.apply_station = MagicMock(side_effect=apply)
    return station_write, prefix_write


def test_station_files_automatically_enable_roi_when_stream_arrives(window, monkeypatch):
    station_write, prefix_write = automatic_station_fixture(window, monkeypatch)
    buttons = [button.text() for button in window.findChildren(gui.QtWidgets.QPushButton)]
    assert "Apply Station + Bin ROI" not in buttons
    assert "Platform teach…" not in buttons
    window._update_station_preview()
    window.node.apply_station.assert_not_called()
    assert "Select a bin teach" in window.station_status.text()
    assert window.platform_path.isReadOnly() and window.calibration_camera_path.isReadOnly()
    window.bin_path.setText("/selected/bin_teach_test.yaml")
    window.node.apply_station.assert_called_once_with(
        "/selected/platform_calibration_test.yaml", "/selected/bin_teach_test.yaml",
        expected_station=gui.latest_station_calibration.return_value)
    assert window.camera_prefix.text() == "station_camera"
    station_write.assert_called_once()
    prefix_write.assert_called_once()
    assert "automatically displayed" in window.station_status.text()
    assert window.bin_platform_warning.isHidden()
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


def test_platform_mismatch_warns_once_without_blocking_preview(window, monkeypatch):
    station_write, prefix_write = automatic_station_fixture(window, monkeypatch, source_sha="b" * 64)
    window.platform_path.setText("/selected/platform_calibration_test.yaml")
    window.bin_path.setText("/selected/bin_teach_test.yaml")
    assert window.node.applied is not None and window.node.bin_artifact is not None
    assert not window.bin_platform_warning.isHidden()
    displayed_warning = window.bin_platform_warning.text()
    warning = displayed_warning.replace("\u200b", "")
    assert "WARNING: Bin/platform mismatch" in warning
    assert "platform_calibration_source.yaml" in warning
    assert "platform_calibration_test.yaml" in warning
    assert "Portable reuse is allowed" in warning and "X/Y directions" in warning
    assert "a" * 64 in window.bin_platform_warning.toolTip()
    assert "b" * 64 in window.bin_platform_warning.toolTip()
    assert warning in window.bin_platform_warning.toolTip()
    assert "automatically displayed" in window.station_status.text()
    station_write.assert_called_once()
    prefix_write.assert_called_once()
    window._job = MagicMock()
    window.node.roi_once = MagicMock()
    window.node.camera_snapshot = lambda: ({
        "rgb": bytes(640 * 480 * 3), "width": 640, "height": 480,
        "stamp_ns": 100_000_000_000, "sequence": 1}, "RGB live")
    for _ in range(3):
        window._refresh_video()
    window._job.assert_called_once_with("roi", window.node.roi_once)
    assert window.bin_platform_warning.text() == displayed_warning  # Not transient feedback.
    records = [call for call in window.node.events.record.call_args_list
               if call.args[1] == "item_bin_platform_mismatch"]
    assert len(records) == 1 and records[0].args[0] == "WARNING"
    assert records[0].kwargs["source_platform_sha256"] == "b" * 64
    assert records[0].kwargs["selected_platform_sha256"] == "a" * 64
    gui.QtWidgets.QMessageBox.warning.assert_not_called()
    assert not window.yolo_toggle.isChecked() and not window.armed_toggle.isChecked()
    window.node.arm.assert_not_called()


@pytest.mark.parametrize("replacement", ["matching_bin", "matching_platform", "empty", "invalid"])
def test_platform_warning_clears_on_reselection(window, monkeypatch, replacement):
    automatic_station_fixture(window, monkeypatch, source_sha="b" * 64)
    window.platform_path.setText("/selected/platform_calibration_test.yaml")
    window.bin_path.setText("/selected/bin_teach_test.yaml")
    assert not window.bin_platform_warning.isHidden()
    if replacement.startswith("matching"):
        automatic_station_fixture(window, monkeypatch)
        if replacement == "matching_bin":
            window.bin_path.setText("/selected/bin_teach_matching.yaml")
        else:
            window._update_station_preview()  # Reload latest, not a manual platform chooser.
        assert window.node.applied is not None
    elif replacement == "empty":
        window.bin_path.clear()
    else:
        window.node.apply_station.side_effect = ValueError("Camera SHA-256 mismatch")
        window._update_station_preview()
        assert "Bin ROI hidden" in window.station_status.text()
        assert window.node.applied is None
    assert window.bin_platform_warning.isHidden()
    assert window.bin_platform_warning.text() == ""
    assert window.bin_platform_warning.toolTip() == ""


def test_platform_warning_clears_when_camera_disconnects_station_binding(window, monkeypatch):
    automatic_station_fixture(window, monkeypatch, source_sha="b" * 64)
    window.platform_path.setText("/selected/platform_calibration_test.yaml")
    window.bin_path.setText("/selected/bin_teach_test.yaml")
    assert not window.bin_platform_warning.isHidden()
    window.node.connect_camera = lambda _: setattr(window.node, "applied", None)
    window.camera_prefix.setText("other_camera")
    window._connect_camera()
    assert window.bin_platform_warning.isHidden() and window.bin_platform_warning.text() == ""
    assert "not bound to the selected station" in window.station_status.text()


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


@pytest.mark.parametrize("source_sha", ["a" * 64, "b" * 64])
def test_restored_station_files_connect_readonly_preview_without_apply(window, monkeypatch, source_sha):
    automatic_station_fixture(window, monkeypatch, source_sha=source_sha)
    monkeypatch.setattr(gui, "load_package_ui_state", lambda _: SimpleNamespace(
        item_platform_filename="platform_calibration_saved.yaml",
        item_bin_filename="bin_teach_saved.yaml", item_profile_filename=None,
        item_preview_camera_prefix="previous_prefix"))
    restored = gui.ItemTeachWindow(window.node)
    restored.timer.stop()
    try:
        window.node.apply_station.assert_called_once_with(
            "/selected/platform_calibration_test.yaml",
            str(gui.workspace_root() / "offline_teach/bin_teach/bin_teach_saved.yaml"),
            expected_station=gui.latest_station_calibration.return_value)
        assert "saved.yaml" not in restored.platform_path.text()  # Old prefill is not authority.
        assert restored.camera_prefix.text() == "station_camera"
        assert restored.saved_path is None and restored.home is None
        assert restored.bin_platform_warning.isHidden() == (source_sha == "a" * 64)
        assert not restored.yolo_toggle.isChecked() and not restored.armed_toggle.isChecked()
        window.node.arm.assert_not_called()
    finally:
        restored.close()


def test_reload_latest_calibration_clears_old_preview_and_invalid_selection(window, monkeypatch):
    automatic_station_fixture(window, monkeypatch)
    window.bin_path.setText("/selected/bin_teach_test.yaml")
    window.yolo_toggle.setChecked(True)
    window.node.last_view = {"old": "snapshot"}
    window.node.disarm.reset_mock()
    window.node.clear_selected_pose.reset_mock()
    gui.latest_station_calibration.side_effect = ValueError("Newest calibration is invalid")
    reload_button = next(button for button in window.findChildren(gui.QtWidgets.QPushButton)
                         if button.text() == "Reload Latest Calibration")
    reload_button.click()
    assert window.node.applied is None and window.node.last_view is None
    assert not window.yolo_toggle.isChecked() and not window.armed_toggle.isChecked()
    window.node.disarm.assert_called()
    window.node.clear_selected_pose.assert_called()
    assert "Newest calibration is invalid" in window.station_status.text()
    assert not window.platform_path.text() and not window.calibration_camera_path.text()
    assert not window.platform_path.toolTip() and not window.calibration_camera_path.toolTip()
    gui.QtWidgets.QMessageBox.warning.assert_not_called()
    calls = gui.latest_station_calibration.call_count
    window._refresh_video()
    assert gui.latest_station_calibration.call_count == calls  # No periodic file selection.


def test_explicit_reselection_can_revalidate_same_artifact(window, monkeypatch):
    automatic_station_fixture(window, monkeypatch)
    window.platform_path.setText("/selected/platform_calibration_test.yaml")
    window.bin_path.setText("/selected/bin_teach_test.yaml")
    monkeypatch.setattr(gui.QtWidgets.QFileDialog, "getOpenFileName",
                        lambda *args: (window.bin_path.text(), "YAML"))
    window._choose_station_file(window.bin_path, Path("/selected"))
    assert window.node.apply_station.call_count == 2
    assert window.node.applied is not None
