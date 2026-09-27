import ast
import copy
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from rclpy.time import Time

from tray_perception import gui
from tray_perception.node import TrayTeachNode
from test_core import camera_info, plane, position, settings


@pytest.fixture
def window(tmp_path):
    app = gui.QtWidgets.QApplication.instance() or gui.QtWidgets.QApplication([])
    node = SimpleNamespace(root=tmp_path, invalidate=MagicMock(), events=MagicMock(),
                           model=None, model_metadata=None, camera=None, plane=None,
                           position=None, selected=None, native=SimpleNamespace(failed=False),
                           fatal_error="", close=MagicMock(), generation=1)
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


def test_canvas_maps_scaled_pixels_and_rejects_letterboxing(window):
    canvas = window.canvas
    canvas.resize(800, 800)
    canvas.show_frame(bytes(640 * 480 * 3), 640, 480)
    assert canvas.image_point(gui.QtCore.QPointF(400, 400)) == pytest.approx((320, 240))
    assert canvas.image_point(gui.QtCore.QPointF(100, 50)) is None
    window.frozen = {"synthetic": True}
    window._click(10, 10)
    window._click(11, 11)  # Duplicate point refused.
    assert len(window.points) == 1
    for x, y in ((100, 10), (100, 100), (10, 100), (200, 200)):
        window._click(x, y)
    assert len(window.points) == 4
    window._undo()
    assert len(window.points) == 3


def test_settings_and_state_restore_without_loading_weights(window):
    window.node.model = {"task": "segment", "path": "/synthetic.pt"}
    window._model_loaded({"task": "segment", "classes": {"0": "tray"}})
    profile_settings = settings()
    profile_settings["yolo"]["image_size"] = 320
    window._fill_settings(profile_settings)
    window._apply()
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
        assert not restored.preview_toggle.isChecked()  # Explicit Apply is still required.
    finally:
        restored.close()


def test_changed_settings_discard_inflight_view(window):
    window._show_view({"generation": 0})  # Must not access pixels on a stale response.
    assert window.canvas.image is None
    window._edited()
    assert not window.preview_toggle.isChecked() and window.settings is None


def snapshot_node():
    info = camera_info()
    rgb = {"width": 640, "height": 480, "stamp_ns": 100_000_000_000,
           "received_at": time.monotonic(), "rgb": bytes(640 * 480 * 3)}
    camera = SimpleNamespace(settings=SimpleNamespace(camera_link_frame="cam_link",
                                                      optical_frame="cam_color_optical_frame"),
                             calibration_mode="camera_to_hand")
    node = SimpleNamespace(validate_sources=MagicMock(), lock=threading.RLock(), rgb=rgb,
                           depth=None, color_info=info, depth_info=None, camera=camera,
                           generation=4, tf_buffer=MagicMock(),
                           get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100)))
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
    with pytest.raises(ValueError, match="Fresh"):
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


def test_bad_depth_does_not_invalidate_a_depth_free_tray_pose():
    camera = SimpleNamespace(settings=SimpleNamespace(camera_prefix="cam"))
    node = SimpleNamespace(camera=camera, lock=threading.RLock(), selected="existing pose",
                           depth="previous depth", events=MagicMock())
    invalid = SimpleNamespace(header=SimpleNamespace(frame_id="wrong_frame",
                                                     stamp=Time(seconds=100).to_msg()))
    TrayTeachNode._receive(node, invalid, "depth", camera)
    assert node.depth is None and node.selected == "existing pose"
    TrayTeachNode._receive(node, invalid, "rgb", camera)
    assert node.rgb is None and node.selected is None


def test_teaching_tf_keeps_source_stamp_without_republishing_old_poses():
    pose = {"position": [.1, .2, .3], "quaternion": [0., 0., 0., 1.]}
    node = SimpleNamespace(native=SimpleNamespace(process=None, failed=False),
                           lock=threading.RLock(), generation=4, _published_key=None,
                           selected=(4, pose, 100_000_000_000, time.monotonic()),
                           fatal_error="", validate_sources=MagicMock(), broadcaster=MagicMock())
    TrayTeachNode._tick(node)
    TrayTeachNode._tick(node)
    node.broadcaster.sendTransform.assert_called_once()
    message = node.broadcaster.sendTransform.call_args.args[0]
    assert message.header.frame_id == "base_link" and message.header.stamp.sec == 100
    node.selected = (4, pose, 100_000_000_000, time.monotonic() - 3)
    TrayTeachNode._tick(node)
    assert node.selected is None


def test_corrupt_native_pose_is_terminal():
    view = {"generation": 4, "camera_context": {},
            "rgb": {"width": 2, "height": 2, "rgb": bytes(12), "stamp_ns": 100_000_000_000}}
    response = {"state": "ok", "width": 2, "height": 2, "count": 1,
                "selected": {"valid": True}, "detections": [{"valid": True}],
                "reason": "malformed synthetic pose", "inference_ms": 1.}
    node = SimpleNamespace(lock=threading.RLock(), model={"task": "segment"}, plane=None,
                           snapshot=lambda: view, native=SimpleNamespace(
                               call=MagicMock(return_value=(response, bytes(12)))),
                           _check_snapshot=MagicMock())
    with pytest.raises(RuntimeError, match="Invalid native selected"):
        TrayTeachNode.preview(node, settings())


def test_loaded_profile_restores_position_without_opening_item_file(tmp_path, monkeypatch):
    from tray_perception import node as module
    source = {"settings": settings(), "model": {"sha256": "1" * 64},
              "camera_calibration": {"filename": "camera.yaml", "sha256": "2" * 64},
              "tray_teach_position": position(), "reference_plane": plane()}
    monkeypatch.setattr(module, "load_profile", lambda *_: copy.deepcopy(source))
    monkeypatch.setattr(module, "load_camera_calibration", lambda *_a, **_k: SimpleNamespace(
        path=tmp_path / "calibration/camera.yaml", sha256="2" * 64))
    node = SimpleNamespace(root=tmp_path, lock=threading.RLock(), apply_camera=MagicMock(),
                           inspect_model=MagicMock(return_value={"task": "segment",
                                                                 "classes": {"0": "tray"}}))
    result = TrayTeachNode.load_saved(node, tmp_path / "tray.yaml")
    assert node.position == position() and node.plane == plane() and result == source
    assert not (tmp_path / "offline_teach/item_teach").exists()


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
