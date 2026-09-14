"""Portable Bin Teach loading through Item Teach's shared read-only backend."""

from dataclasses import replace
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
from rclpy.time import Time
from tf2_ros import TransformException

from camera_calibration_gui.calibration_core import CAMERA_ON_HAND, CAMERA_TO_HAND
from item_perception_yolo import bin_teach_core as core
from item_perception_yolo import item_detector as detector
from item_perception_yolo.platform_teach_core import rotation_matrix_to_quaternion
from test_bin_teach import _applied, _write_template
from test_bin_planar_geometry import pose


def tf_message(matrix, stamp_sec=100):
    message = TransformStamped()
    message.header.stamp = TimeMessage(sec=stamp_sec)
    translation = message.transform.translation
    translation.x, translation.y, translation.z = matrix[:3, 3].tolist()
    rotation = message.transform.rotation
    rotation.x, rotation.y, rotation.z, rotation.w = rotation_matrix_to_quaternion(matrix[:3, :3])
    return message


def transferred_node(tmp_path, monkeypatch, mode):
    source_path = _write_template(tmp_path / "source", CAMERA_TO_HAND)
    destination_root = tmp_path / "destination"
    destination_root.mkdir()
    applied = _applied(destination_root, mode)
    directory = core.bin_teach_directory(destination_root)
    directory.mkdir(parents=True)
    copied = directory / source_path.name
    copied.write_bytes(source_path.read_bytes())
    original = core.load_bin_teach(source_path, root=tmp_path / "source")
    # Remove the source station files from this temporary fixture. The bin must
    # be independently loadable on a station with a different platform and camera.
    for path in (tmp_path / "source" / "calibration").iterdir():
        path.unlink()
    source_path.unlink()

    platform = pose((.7, -.2, .32), (12, -7, 25))
    internal = pose((.012, -.006, .003), (-90, 0, -90))
    robot = pose((.3, .1, .55), (20, 15, -35))
    view = pose((.02, -.03, 1.2), (176, 5, 11))
    base_camera = platform @ view @ np.linalg.inv(internal)
    mounting = base_camera if mode == CAMERA_TO_HAND else np.linalg.inv(robot) @ base_camera
    applied = replace(
        applied, camera=replace(applied.camera, reference_from_camera_link=mounting),
        platform=replace(applied.platform, base_from_platform=platform,
                         calibration_reference_from_camera_link=mounting),
    )
    frame = {"stamp_ns": 100_000_000_000, "received_at": time.monotonic(),
             "width": 848, "height": 480, "sequence": 1, "rgb": bytes(848 * 480 * 3)}
    info = {"width": 848, "height": 480,
            "k": [461., 0., 424., 0., 461., 240., 0., 0., 1.], "d": [0.] * 5}
    node = SimpleNamespace(
        applied=None, bin_artifact=None, camera_prefix=None,
        disarm=MagicMock(), events=MagicMock(), _feedback_lock=threading.RLock(),
        _validate_sources=MagicMock(), _color_info=info, tf_buffer=MagicMock(),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=100_100_000_000)),
    )
    node.connect_camera = MagicMock(
        side_effect=lambda prefix: setattr(node, "camera_prefix", prefix))

    def lookup(parent, child, instant):
        assert instant.nanoseconds == frame["stamp_ns"]
        if (parent, child) == (applied.camera.settings.camera_link_frame,
                               applied.camera.settings.optical_frame):
            return tf_message(internal)
        assert (parent, child) == ("base_link", "Link6") and mode == CAMERA_ON_HAND
        return tf_message(robot)

    node.tf_buffer.lookup_transform.side_effect = lookup
    monkeypatch.setattr(detector, "load_bin_teach_calibration_context", lambda _: applied)
    monkeypatch.setattr(detector, "load_bin_teach",
                        lambda path: core.load_bin_teach(path, root=destination_root))
    detector.ItemDetectNode.apply_station(node, applied.platform.path, copied)
    return node, frame, view, original


@pytest.mark.parametrize("mode", [CAMERA_TO_HAND, CAMERA_ON_HAND])
def test_item_teach_transfers_bin_to_destination_plane_without_source_station(
        tmp_path, monkeypatch, mode):
    node, frame, expected, original = transferred_node(tmp_path, monkeypatch, mode)
    context = detector.ItemDetectNode._measurement_context(node, frame)
    assert node.bin_artifact.points == original.points
    assert context["roi"] == [[p.x_m, p.y_m] for p in original.points]
    assert np.allclose(context["platform_from_optical"], expected)
    assert not np.allclose(node.applied.platform.base_from_platform[:2, 2], 0)
    node.connect_camera.assert_called_once_with(node.applied.camera.settings.camera_prefix)
    node.disarm.assert_called_once()
    assert node.tf_buffer.lookup_transform.call_count == (2 if mode == CAMERA_ON_HAND else 1)
    # No model, marker detection or depth fields are needed to construct the ROI view.
    assert not hasattr(node, "model_config") and not hasattr(node, "_depth")


@pytest.mark.parametrize("invalid", ["camera", "info", "tf", "robot_age", "source_changed"])
def test_item_teach_rejects_invalid_destination_projection(tmp_path, monkeypatch, invalid):
    node, frame, _expected, _original = transferred_node(tmp_path, monkeypatch, CAMERA_ON_HAND)
    if invalid == "camera":
        node.camera_prefix = "other_camera"
    elif invalid == "info":
        node._color_info = None
    elif invalid == "tf":
        node.tf_buffer.lookup_transform.side_effect = TransformException("Missing optical TF")
    elif invalid == "robot_age":
        node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=102_000_000_000))
    else:
        node._validate_sources.side_effect = ValueError("Applied calibration changed")
    with pytest.raises((ValueError, TransformException)):
        detector.ItemDetectNode._measurement_context(node, frame)


def test_item_roi_projection_dependency_remains_native_import_free():
    import ast
    from item_perception_yolo import planar_bin_roi
    tree = ast.parse(Path(planar_bin_roi.__file__).read_text())
    assert not any(isinstance(node, (ast.Import, ast.ImportFrom)) for node in ast.walk(tree))
