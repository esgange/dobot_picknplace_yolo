"""GUI-only ROS visualization lifecycle; no executor or hardware launched."""

import json
from copy import deepcopy
from types import SimpleNamespace
import threading
from unittest.mock import MagicMock

import numpy as np
import pytest
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
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


def test_live_markers_reverse_only_downward_blue_guide_not_candidates_or_tf(preview):
    visual, node, _ = preview
    sample = snapshot()
    sample["base_from_platform"] = np.diag([1., -1., -1., 1.]).tolist()
    original = deepcopy(sample)
    visual.publish(sample)
    assert sample == original
    frames = node.selected_pose_broadcaster.sendTransform.call_args.args[0]
    markers = visual.marker_publisher.publish.call_args.args[0].markers[1:]
    for index, frame in enumerate(frames):
        assert frame.transform.rotation.x == 1.  # The real TF still points down.
        blue = markers[3 * index + 2]
        assert blue.points[1].z == -.2 and blue.ns.endswith("_up")
        assert markers[3 * index].points[1].x == .2
        assert markers[3 * index + 1].points[1].y == .2
    diagnostic = json.loads(visual.diagnostic_publisher.publish.call_args.args[0].data)
    assert diagnostic["candidates"] == original["candidates"]
    assert diagnostic["blue_guide"] == "upward_surface_normal_not_pose_z"


def test_all_frames_markers_cloud_and_diagnostics_share_snapshot(preview):
    visual, node, clock = preview
    qos = node.create_publisher.call_args_list[0].args[2]
    assert qos.depth == 1 and qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
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
    assert len(markers) == 13  # Three axes for each of all four candidates, with no text.
    assert all(m.type == Marker.ARROW and not m.text for m in markers[1:])
    assert all(m.lifetime.sec == 2 for m in markers[1:])
    diagnostic = json.loads(visual.diagnostic_publisher.publish.call_args.args[0].data)
    assert diagnostic["rejected"] == snapshot()["rejected"]
    assert len(diagnostic["candidates"]) == 4 and diagnostic["voxel_size_mm"] == 10
    clock[0] += .1
    visual.publish(snapshot())  # Completed job cannot bypass the 1 Hz publishing bound.
    visual.tick()
    assert visual.cloud_publisher.publish.call_count == 1
    clock[0] += .9
    fresh(node, clock)
    visual.tick()
    assert visual.cloud_publisher.publish.call_count == 1  # Same frame never resets its age.
    assert visual.cloud_publisher.publish.call_args.args[0].header.stamp.sec == 100
    diagnostic = json.loads(visual.diagnostic_publisher.publish.call_args.args[0].data)
    assert diagnostic["age_sec"] == 1.
    clock[0] += 3.99
    fresh(node, clock)
    visual.tick()
    assert visual.cloud_publisher.publish.call_count == 1  # Still colored before five seconds.
    assert node.selected_pose_broadcaster.sendTransform.call_count == 3
    clock[0] += 1.
    fresh(node, clock)
    visual.tick()
    assert visual.cloud_publisher.publish.call_count == 2 and visual.displayed_grey
    grey = visual.cloud_publisher.publish.call_args.args[0]
    assert bytes(grey.data)[:12] == snapshot()["data"][:12]
    assert int.from_bytes(bytes(grey.data)[12:], "little") == 0x808080
    assert grey.width == 1 and grey.header.stamp.sec == 100
    assert node.selected_pose_broadcaster.sendTransform.call_count == 3
    assert snapshot()["data"] != bytes(grey.data)  # Original colors are not mutated.
    diagnostic = json.loads(visual.diagnostic_publisher.publish.call_args.args[0].data)
    assert diagnostic["status"] == "stale_grey" and not diagnostic["candidates"]
    assert diagnostic["refresh_age_sec"] == pytest.approx(5.99)
    assert visual.marker_publisher.publish.call_args.args[0].markers[0].action == Marker.DELETEALL
    clock[0] += 30.
    fresh(node, clock)
    visual.publish(snapshot())  # Reprocessing the same frozen frame cannot restore color.
    assert visual.cloud_publisher.publish.call_count == 2  # Grey cloud remains indefinitely.
    clock[0] += 1.
    fresh(node, clock)
    visual.publish({**snapshot(), "stamp_ns": node._image["stamp_ns"],
                    "depth_stamp_ns": node._depth["stamp_ns"]})
    assert visual.cloud_publisher.publish.call_count == 3 and not visual.displayed_grey
    assert bytes(visual.cloud_publisher.publish.call_args.args[0].data) == snapshot()["data"]
    diagnostic = json.loads(visual.diagnostic_publisher.publish.call_args.args[0].data)
    assert diagnostic["refresh_age_sec"] == 0.


def test_clear_retains_geometry_without_poses_or_repeated_cloud_messages(preview):
    visual, node, clock = preview
    visual.publish(snapshot())
    visual.clear("Teaching preview closed")
    assert visual.current is None and visual.displayed_grey
    grey = visual.cloud_publisher.publish.call_args.args[0]
    assert grey.width == 1 and bytes(grey.data)[:12] == snapshot()["data"][:12]
    assert visual.cloud_publisher.publish.call_count == 2
    clock[0] += 60.
    visual.clear("Teaching preview closed")
    visual.tick()
    assert visual.cloud_publisher.publish.call_count == 2
    assert node.selected_pose_broadcaster.sendTransform.call_count == 1
    diagnostic = json.loads(visual.diagnostic_publisher.publish.call_args.args[0].data)
    assert diagnostic["status"] == "stale_grey"
    assert diagnostic["reason"] == "Teaching preview closed"
    assert diagnostic["source_stamp_ns"] == snapshot()["stamp_ns"]


@pytest.mark.parametrize("invalid", ["epoch", "camera", "intrinsics", "source", "worker", "fatal"])
def test_invalidated_inputs_keep_grey_cloud_and_clear_poses(preview, invalid):
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
    elif invalid == "worker":
        node.native.failed = True
    else:
        node.fatal_error = "Synthetic fatal feedback"
    visual.tick()
    assert visual.current is None
    assert visual.cloud_publisher.publish.call_args.args[0].width == 1
    assert visual.displayed_grey
    assert visual.marker_publisher.publish.call_args.args[0].markers[0].action == Marker.DELETEALL
    assert node.selected_pose_broadcaster.sendTransform.call_count == 1


@pytest.mark.parametrize("gap", ["sync", "stale", "missing", "busy", "no_result", "empty"])
def test_acquisition_gap_retains_cloud_until_valid_replacement(preview, gap):
    visual, node, clock = preview
    first = snapshot()
    visual.publish(first)
    clock[0] += 1.
    fresh(node, clock)
    if gap == "sync":
        node._depth["stamp_ns"] -= 200_000_000  # Fresh, but not synchronized with RGB.
    elif gap == "stale":
        node._depth["stamp_ns"] = 90_000_000_000
    elif gap == "missing":
        node._color_info = None
    elif gap == "busy":
        visual.publish({"error": "Pose request has priority"})
    elif gap == "no_result":
        visual.publish(None)
    else:
        visual.publish({**first, "point_count": 0, "data": b"", "candidates": []})
    visual.tick()
    assert visual.current is first and visual.waiting_reason
    # No empty cloud is sent during a slow/missing/rejected replacement.
    assert visual.cloud_publisher.publish.call_count == 1
    assert node.selected_pose_broadcaster.sendTransform.call_count == 1
    assert visual.marker_publisher.publish.call_args.args[0].markers[0].action == Marker.DELETEALL
    diagnostic = json.loads(visual.diagnostic_publisher.publish.call_args.args[0].data)
    assert diagnostic["status"] == "retained_cloud" and diagnostic["age_sec"] == 1.
    assert diagnostic["source_stamp_ns"] == first["stamp_ns"] and not diagnostic["candidates"]

    clock[0] += 10.
    fresh(node, clock)
    node._color_info = first["camera"]
    visual.tick()
    assert visual.current is first and visual.cloud_publisher.publish.call_count == 2
    assert visual.displayed_grey
    assert node.selected_pose_broadcaster.sendTransform.call_count == 1
    # Fresh input alone never relabels an old observation as a new result.
    clock[0] += 1.
    fresh(node, clock)
    second = {**snapshot(), "stamp_ns": node._image["stamp_ns"],
              "depth_stamp_ns": node._depth["stamp_ns"], "data": bytes(16)}
    visual.publish(second)
    assert visual.current is second and not visual.waiting_reason
    assert node.selected_pose_broadcaster.sendTransform.call_count == 2
    clouds = [call.args[0] for call in visual.cloud_publisher.publish.call_args_list]
    assert len(clouds) == 3 and all(cloud.width == 1 for cloud in clouds)
    assert bytes(clouds[-1].data) == second["data"]
    assert clouds[-1].header.stamp.sec == int(clock[0])
    visual.hold("Another acquisition gap")
    node.arm_epoch += 1
    visual.tick()
    assert visual.current is None  # Pose eligibility never survives a source/settings reset.
    assert visual.cloud_publisher.publish.call_args.args[0].width == 1
    assert visual.displayed_grey


def test_exact_observation_used_without_second_yolo_or_production_request(preview):
    visual, node, _ = preview
    context = {"camera": node._color_info, "depth_camera": node._depth_info}
    view = {"metadata": {"detections": []}, "observation": {
        "epoch": 3, "camera_generation": 4, "rgb": node._image, "depth": node._depth,
        "context": context, "error": ""}}
    options = {"quality": dict(QUALITY_DEFAULTS), "settings": None,
               "planning": None, "pose_error": "Select item settings"}
    node.native.call.return_value = ({"state": "ok", "generation": 3, "point_count": 0,
                                      "nearby_overlay": False,
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
                                      "nearby_overlay": False,
                                      "candidates": [], "rejected": []}, b"truncated")
    with pytest.raises(RuntimeError, match="Invalid native RViz"):
        visual.compute(view, options)
    assert node.native.failed and not node.operation_lock.locked()
    node.native.close.assert_called_once()


@pytest.mark.parametrize("invalidated", [False, True])
def test_nearby_overlay_updates_only_its_validated_source_view(preview, invalidated):
    visual, node, _ = preview
    view = {"preview_mode": "all", "rgb": bytes(12), "depth_rgb": bytes(12),
            "metadata": {"detections": []}, "observation": {
                "epoch": 3, "camera_generation": 4, "rgb": node._image,
                "depth": node._depth, "context": {
                    "camera": node._color_info, "depth_camera": node._depth_info}, "error": ""}}
    options = {"quality": dict(QUALITY_DEFAULTS), "settings": {"yolo": {"max_detections": 20}},
               "planning": {}, "pose_error": ""}
    rendered_rgb, rendered_depth = bytes([19]*12), bytes([23]*12)
    cloud = snapshot()["data"]

    def reply(*_):
        if invalidated:
            node.arm_epoch += 1
        return ({"state": "ok", "generation": 3, "point_count": 1, "nearby_overlay": True,
                 "candidates": [], "rejected": []}, cloud + rendered_rgb + rendered_depth)

    node.native.call.side_effect = reply
    result = visual.compute(view, options)
    header, payload, _ = node.native.call.call_args.args
    assert header["nearby_overlay"] is True
    assert payload == node._image["rgb"] + node._depth["depth"] + bytes(24)
    if invalidated:
        assert "invalidated" in result["error"]
        assert view["rgb"] == view["depth_rgb"] == bytes(12)
        assert "nearby_overlay" not in view
    else:
        assert result["data"] == cloud  # Image bytes must never enter PointCloud2.
        assert view["rgb"] == rendered_rgb and view["depth_rgb"] == rendered_depth
        assert view["nearby_overlay"] is True


def test_gap_retains_the_published_cloud_not_an_unpublished_result(preview):
    visual, node, clock = preview
    first = snapshot()
    visual.publish(first)
    clock[0] += .1
    second = {**snapshot(), "stamp_ns": 100_100_000_000}
    visual.publish(second)  # Waiting for the next publication slot.
    assert visual.current is second and visual.cloud_publisher.publish.call_count == 1
    clock[0] += .9
    fresh(node, clock)
    node._depth["stamp_ns"] -= 200_000_000
    visual.tick()
    assert visual.current is first
    assert visual.cloud_publisher.publish.call_count == 1
    diagnostic = json.loads(visual.diagnostic_publisher.publish.call_args.args[0].data)
    assert diagnostic["source_stamp_ns"] == first["stamp_ns"]
    assert diagnostic["age_sec"] == 1.
