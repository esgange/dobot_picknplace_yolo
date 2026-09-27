"""GUI-only ROS visualization lifecycle; no executor or hardware launched."""

import json
from types import SimpleNamespace
import threading
from unittest.mock import MagicMock

import numpy as np
import pytest
from rclpy.time import Time
from sensor_msgs.msg import PointField
from visualization_msgs.msg import Marker

from item_perception_yolo import item_teach_rviz as rviz
from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
from item_perception_yolo.item_teach_gui import build_selected_pose_transform


@pytest.fixture
def preview(monkeypatch):
    clock = [100.]
    monkeypatch.setattr(rviz.time, "monotonic", lambda: clock[0])
    info = {"width": 2, "height": 2, "k": [100., 0., 1., 0., 100., 1., 0., 0., 1.]}
    rgb = {"rgb": bytes(12), "width": 2, "height": 2,
           "stamp_ns": 100_000_000_000, "received_at": 100.}
    depth = {**rgb, "depth": bytes(8)}
    node = SimpleNamespace(
        create_publisher=MagicMock(side_effect=lambda *_: MagicMock()), create_timer=MagicMock(),
        _validate_sources=MagicMock(), get_clock=lambda: SimpleNamespace(
            now=lambda: Time(nanoseconds=int(clock[0] * 1e9))),
        _image=rgb, _depth=depth, _color_info=info, _depth_info=info,
        condition=threading.Condition(), request_lock=threading.Lock(),
        operation_lock=threading.Lock(), arm_epoch=3, _camera_generation=4, fatal_error="",
        native=SimpleNamespace(failed=False, call=MagicMock(), close=MagicMock()),
        applied=SimpleNamespace(platform=SimpleNamespace(base_from_platform=np.eye(4))),
        selected_pose_broadcaster=MagicMock())
    return rviz.TeachingRvizPreview(node, build_selected_pose_transform), node, clock


def snapshot():
    info = {"width": 2, "height": 2, "k": [100., 0., 1., 0., 100., 1., 0., 0., 1.]}
    return {"epoch": 3, "camera_generation": 4, "stamp_ns": 100_000_000_000,
            "camera": info, "depth_camera": info,
            "depth_stamp_ns": 100_000_000_000, "quality": dict(QUALITY_DEFAULTS),
            "base_from_platform": np.eye(4).tolist(), "point_count": 1,
            "data": bytes.fromhex("0000803f000000400000404078563400"), "pose_error": "",
            "candidates": [{"position": [.1 * i, 0., .1], "quaternion": [0., 0., 0., 1.],
                            "class_name": "part", "confidence": .9} for i in range(4)],
            "rejected": [{"source_index": 4, "reason": "synthetic depth rejection"}]}


def fresh(node, clock):
    for image in (node._image, node._depth):
        image.update(stamp_ns=int(clock[0] * 1e9), received_at=clock[0])


def test_all_frames_markers_cloud_and_diagnostics_share_snapshot(preview):
    visual, node, clock = preview
    visual.publish(snapshot())
    cloud = visual.cloud_publisher.publish.call_args.args[0]
    assert cloud.header.frame_id == "base_link" and cloud.header.stamp.sec == 100
    assert cloud.width == 1 and cloud.point_step == cloud.row_step == 16
    assert [field.name for field in cloud.fields] == ["x", "y", "z", "rgb"]
    assert cloud.fields[-1].datatype == PointField.UINT32
    assert bytes(cloud.data) == snapshot()["data"]
    frames = node.selected_pose_broadcaster.sendTransform.call_args.args[0]
    assert [f.child_frame_id for f in frames] == [
        f"item_teach_live_candidate_{i}" for i in range(1, 5)]
    assert frames[3].transform.translation.x == pytest.approx(.3)
    markers = visual.marker_publisher.publish.call_args.args[0].markers
    assert markers[0].action == Marker.DELETEALL
    assert len(markers) == 17  # 3 axes + label for each of all four candidates.
    assert all(m.lifetime.sec == 2 for m in markers[1:])
    diagnostic = json.loads(visual.diagnostic_publisher.publish.call_args.args[0].data)
    assert diagnostic["rejected"] == snapshot()["rejected"]
    assert len(diagnostic["candidates"]) == 4 and diagnostic["voxel_size_mm"] == 5
    clock[0] += .1
    visual.publish(snapshot())  # Completed job cannot bypass the 1 Hz publishing bound.
    visual.tick()
    assert visual.cloud_publisher.publish.call_count == 1
    clock[0] += .9
    fresh(node, clock)
    visual.tick()
    assert visual.cloud_publisher.publish.call_count == 2
    assert visual.cloud_publisher.publish.call_args.args[0].header.stamp.sec == 100
    assert "snapshot 1.0s" in visual.marker_publisher.publish.call_args.args[0].markers[-1].text


@pytest.mark.parametrize("invalid", ["epoch", "camera", "intrinsics", "source", "stale",
                                     "worker", "fatal"])
def test_invalidated_or_lost_inputs_clear_visualization(preview, invalid):
    visual, node, clock = preview
    visual.publish(snapshot())
    clock[0] += 1.
    fresh(node, clock)
    if invalid == "epoch":
        node.arm_epoch += 1
    elif invalid == "camera":
        node._camera_generation += 1
    elif invalid == "intrinsics":
        node._depth_info = {**node._depth_info, "d": [0.1, 0., 0., 0., 0.]}
    elif invalid == "source":
        node._validate_sources.side_effect = ValueError("Selected source changed")
    elif invalid == "stale":
        node._depth["stamp_ns"] = 90_000_000_000
    elif invalid == "worker":
        node.native.failed = True
    else:
        node.fatal_error = "Synthetic fatal feedback"
    visual.tick()
    assert visual.current is None
    assert visual.cloud_publisher.publish.call_args.args[0].width == 0
    assert visual.marker_publisher.publish.call_args.args[0].markers[0].action == Marker.DELETEALL
    assert node.selected_pose_broadcaster.sendTransform.call_count == 1


def test_exact_observation_used_without_second_yolo_or_production_request(preview):
    visual, node, _ = preview
    context = {"camera": node._color_info, "depth_camera": node._depth_info}
    view = {"metadata": {"detections": []}, "observation": {
        "epoch": 3, "camera_generation": 4, "rgb": node._image, "depth": node._depth,
        "context": context, "error": ""}}
    options = {"quality": dict(QUALITY_DEFAULTS), "settings": None,
               "planning": None, "pose_error": "Select item settings"}
    node.native.call.return_value = ({"state": "ok", "generation": 3, "point_count": 0,
                                      "candidates": [], "rejected": []}, b"")
    result = visual.compute(view, options)
    assert result["stamp_ns"] == node._image["stamp_ns"]
    assert result["pose_error"] == options["pose_error"]
    header, payload, timeout = node.native.call.call_args.args
    assert header["operation"] == "teaching_rviz" and "model" not in header
    assert payload == node._image["rgb"] + node._depth["depth"] and timeout == 10.
    assert "pick_planning" not in context  # Shared clicked-pose snapshot stays untouched.
    node.request_lock.acquire()
    try:
        assert "priority" in visual.compute(view, options)["error"]
        assert node.native.call.call_count == 1
    finally:
        node.request_lock.release()
    node.arm_epoch += 1
    assert "invalidated" in visual.compute(view, options)["error"]
    assert not node.operation_lock.locked()


def test_corrupt_native_cloud_is_terminal(preview):
    visual, node, _ = preview
    view = {"observation": {"epoch": 3, "camera_generation": 4, "rgb": node._image,
                            "depth": node._depth, "context": {}, "error": ""}}
    options = {"quality": dict(QUALITY_DEFAULTS), "settings": None,
               "planning": None, "pose_error": ""}
    node.native.call.return_value = ({"state": "ok", "generation": 3, "point_count": 1,
                                      "candidates": [], "rejected": []}, b"truncated")
    with pytest.raises(RuntimeError, match="Invalid native RViz"):
        visual.compute(view, options)
    assert node.native.failed and not node.operation_lock.locked()
    node.native.close.assert_called_once()
