"""Video-first platform/bin layout tests with synthetic state, never live nodes."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
from python_qt_binding import QtCore, QtWidgets

from item_perception_yolo import bin_teach_gui, platform_teach_gui


@pytest.mark.parametrize("kind", ["platform", "bin"])
def test_teach_layout_prioritizes_video_and_keeps_details_available(monkeypatch, kind):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    snapshot = {"fatal_error": None, "calibration": None, "capture": None, "saved_path": None,
                "target_ready": False, "gate": "No valid marker observation",
                "applied": None, "aruco_settings": None, "loaded_bin": None,
                "configured": False, "has_preview": False, "overlay": None}
    node = SimpleNamespace(load_last_session=lambda: None, status_snapshot=lambda: snapshot,
                           latest_overlay=lambda: None, robot_lan1_ip="192.168.20.204",
                           capture=MagicMock(), save=MagicMock())
    cls = platform_teach_gui.PlatformTeachWindow if kind == "platform" else bin_teach_gui.BinTeachWindow
    window = cls(node)
    window._timer.stop()
    window.show()
    app.processEvents()
    try:
        sidebar, video = window.workspace_split.sizes()
        assert video > sidebar*2
        assert window.workspace_split.orientation() == QtCore.Qt.Horizontal
        assert not window.details_panel.isVisible()
        window.details_toggle.setChecked(True)
        assert window.details_panel.isVisible() and window.output.isVisible()
        assert window.save_button.isVisible() and window.capture_button.isVisible()
        assert not window.settings_scroll.isAncestorOf(window.save_button)
        assert not window.settings_scroll.isAncestorOf(window.capture_button)
        assert not window.save_button.isEnabled() and not window.capture_button.isEnabled()
        assert window.apply_button.isVisible()
        # Fresh frames retain an explicit blocked-capture reason in the video.
        frame = np.zeros((480, 640, 3), np.uint8)
        if kind == "platform":
            node.latest_overlay = lambda: frame
        else:
            snapshot.update(overlay=frame, selected_pixels=None)
        window._refresh()
        assert not window.video.pixmap().isNull()
        node.capture.assert_not_called()
        node.save.assert_not_called()
    finally:
        window.close()
        app.processEvents()
