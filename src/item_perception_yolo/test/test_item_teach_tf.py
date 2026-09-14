"""Teaching-only simulated/clicked TF batches; no cameras, models or robot execution."""

from copy import deepcopy
import os
from pathlib import Path
import subprocess
from types import MethodType, SimpleNamespace
from unittest.mock import MagicMock
import threading

import numpy as np
import pytest
from rclpy.time import Time

from item_perception_interfaces.msg import ItemCandidate
from item_perception_interfaces.srv import GetItemPoses
from item_perception_yolo import item_detector as detector
from item_perception_yolo import item_teach_gui as gui


def batch(count=5, batch_id="test-batch"):
    response = GetItemPoses.Response(success=True, status="OK", batch_id=batch_id)
    response.header.frame_id = "platform_reference"
    response.header.stamp = Time(nanoseconds=100_000_000_000).to_msg()
    response.detected_count, response.valid_count = 20, count
    for index in range(count):
        candidate = ItemCandidate(id=f"{batch_id}:{index}", priority=index+1, class_name="part")
        candidate.pose.position.x = .02 * index
        candidate.pose.position.y = -.01 * index
        candidate.pose.position.z = .1 + .01 * index
        candidate.pose.orientation.z = float(np.sin(.1 * index))
        candidate.pose.orientation.w = float(np.cos(.1 * index))
        response.candidates.append(candidate)
    view = {"stamp_ns": 100_000_000_000, "simulation_epoch": 7,
            "simulation_profile": ("test_profile.yaml", "a" * 64), "rgb": b"image-not-tf-state"}
    return response, view


@pytest.fixture
def teaching_node(monkeypatch):
    c, s = np.cos(.2), np.sin(.2)
    base = np.array([[c, 0, s, .2], [0, 1, 0, -.3], [-s, 0, c, .4], [0, 0, 0, 1.]])
    clock = SimpleNamespace(now=lambda: Time(nanoseconds=100_100_000_000))
    node = SimpleNamespace(
        selection_lock=threading.RLock(), selected_pose=None, arm_epoch=7, yolo_enabled=True,
        native=SimpleNamespace(failed=False), fatal_error=None, service=None, events=MagicMock(),
        get_clock=lambda: clock, selected_pose_broadcaster=MagicMock(),
        _validate_sources=MagicMock(), applied=SimpleNamespace(
            platform=SimpleNamespace(base_from_platform=base)),
        settings={"quality": {"result_max_age_sec": 2.}}, disarm=MagicMock())
    monkeypatch.setattr(detector, "file_sha256", lambda _: "a" * 64)
    for name in ("clear_selected_pose", "show_selected_pose", "show_simulated_poses",
                 "_broadcast_selected_pose"):
        setattr(node, name, MethodType(getattr(gui.ItemTeachNode, name), node))
    node.validate_simulation_view = MethodType(
        detector.ItemDetectNode.validate_simulation_view, node)
    return node


def test_simulated_tf_matches_every_ranked_pose_and_preserves_platform_tilt(teaching_node):
    node = teaching_node
    response, view = batch()
    original = deepcopy(response)
    node.show_simulated_poses(response, view)
    node._broadcast_selected_pose()
    transforms = node.selected_pose_broadcaster.sendTransform.call_args.args[0]
    assert len(transforms) == 5  # Not restricted to the three expanded text entries.
    for priority, (message, candidate) in enumerate(zip(transforms, response.candidates), 1):
        assert message.header.frame_id == "base_link"
        assert message.child_frame_id == f"item_teach_candidate_{priority}"
        assert message.header.stamp == node.get_clock().now().to_msg()
        p, q = candidate.pose.position, candidate.pose.orientation
        clicked = gui.build_selected_pose_transform(
            node.applied.platform.base_from_platform,
            {"position": [p.x, p.y, p.z], "quaternion": [q.x, q.y, q.z, q.w]}, message.header.stamp)
        assert np.allclose(gui.transform_matrix(message), gui.transform_matrix(clicked))
    assert not np.allclose(gui.transform_matrix(transforms[0])[:3, 2], [0., 0., 1.])
    assert response == original  # Neither platform poses nor priorities are modified.
    assert set(node.selected_pose[2]) == {"simulation_epoch", "simulation_profile"}
    assert node.service is None
    node.disarm.assert_not_called()


def test_frozen_batch_rebroadcasts_at_current_stamp_but_never_tracks_new_items(teaching_node):
    node = teaching_node
    response, view = batch(3)
    node.show_simulated_poses(response, view)
    node._broadcast_selected_pose()
    before = [gui.transform_matrix(t) for t in node.selected_pose[1]]
    response.candidates[0].pose.position.x = 500.  # External response mutation must not move TF.
    node.get_clock().now = lambda: Time(nanoseconds=200_000_000_000)
    node._broadcast_selected_pose()
    transforms = node.selected_pose_broadcaster.sendTransform.call_args.args[0]
    for old, message in zip(before, transforms):
        assert message.header.stamp.sec == 200
        assert np.allclose(gui.transform_matrix(message), old)


def test_batches_replace_clicked_tf_and_each_other_atomically(teaching_node):
    node = teaching_node
    node.show_selected_pose({"position": [.1, .2, .3], "quaternion": [0., 0., 0., 1.]},
                            100_000_000_000, 7)
    assert node.selected_pose[1][0].child_frame_id == "item_teach_selected_item"
    response, view = batch(5)
    node.show_simulated_poses(response, view)
    node._broadcast_selected_pose()
    assert len(node.selected_pose_broadcaster.sendTransform.call_args.args[0]) == 5
    response, view = batch(1, "second-batch")
    node.show_simulated_poses(response, view)
    node._broadcast_selected_pose()
    messages = node.selected_pose_broadcaster.sendTransform.call_args.args[0]
    assert [t.child_frame_id for t in messages] == ["item_teach_candidate_1"]
    node.show_selected_pose({"position": [.2, .1, .3], "quaternion": [0., 0., 0., 1.]},
                            100_000_000_000, 7)
    node._broadcast_selected_pose()
    messages = node.selected_pose_broadcaster.sendTransform.call_args.args[0]
    assert [t.child_frame_id for t in messages] == ["item_teach_selected_item"]
    node.clear_selected_pose()
    node.selected_pose_broadcaster.reset_mock()
    node._broadcast_selected_pose()
    node.selected_pose_broadcaster.sendTransform.assert_not_called()


def test_empty_simulation_clears_every_old_candidate(teaching_node):
    node = teaching_node
    node.show_simulated_poses(*batch(3))
    node.show_simulated_poses(*batch(0))
    assert node.selected_pose is None
    node._broadcast_selected_pose()
    node.selected_pose_broadcaster.sendTransform.assert_not_called()


@pytest.mark.parametrize("failure", ["epoch", "off", "native", "fatal", "source", "profile"])
def test_batch_timer_clears_on_invalidation_without_gui_progress(
        teaching_node, monkeypatch, failure):
    node = teaching_node
    node.show_simulated_poses(*batch(3))
    if failure == "epoch":
        node.arm_epoch += 1
    elif failure == "off":
        node.yolo_enabled = False
    elif failure == "native":
        node.native.failed = True
    elif failure == "fatal":
        node.fatal_error = "Synthetic fatal error"
    elif failure == "source":
        node._validate_sources.side_effect = ValueError("Station changed")
    else:
        monkeypatch.setattr(detector, "file_sha256", lambda _: "b" * 64)
    node._broadcast_selected_pose()
    assert node.selected_pose is None
    node.selected_pose_broadcaster.sendTransform.assert_not_called()


@pytest.mark.parametrize("failure", [
    "stale", "future", "stamp", "frame", "failed",
    "priority", "duplicate", "nonfinite", "rotation",
])
def test_invalid_simulated_batch_clears_old_tf_without_partial_publication(teaching_node, failure):
    node = teaching_node
    node.show_simulated_poses(*batch(3))
    response, view = batch(3)
    if failure == "stale":
        node.get_clock().now = lambda: Time(nanoseconds=103_000_000_000)
    elif failure == "future":
        node.get_clock().now = lambda: Time(nanoseconds=99_000_000_000)
    elif failure == "stamp":
        view["stamp_ns"] += 1
    elif failure == "frame":
        response.header.frame_id = "camera_optical_frame"
    elif failure == "failed":
        response.success = False
    elif failure == "priority":
        response.candidates[-1].priority = 1
    elif failure == "duplicate":
        response.candidates[-1].id = response.candidates[0].id
    elif failure == "nonfinite":
        response.candidates[-1].pose.position.z = float("nan")
    else:
        response.candidates[-1].pose.orientation.w = 2.
    with pytest.raises(ValueError):
        node.show_simulated_poses(response, view)
    node._broadcast_selected_pose()
    assert node.selected_pose is None
    node.selected_pose_broadcaster.sendTransform.assert_not_called()


def test_epoch_change_during_batch_validation_does_not_publish(teaching_node):
    node = teaching_node
    response, view = batch(3)
    node._validate_sources.side_effect = lambda: setattr(node, "arm_epoch", 8)
    with pytest.raises(ValueError, match="invalidated"):
        node.show_simulated_poses(response, view)
    node._broadcast_selected_pose()
    node.selected_pose_broadcaster.sendTransform.assert_not_called()


def exercise_tf_transport():
    """Real DDS TF messages in an isolated domain, with synthetic poses/sources only."""
    import sys
    import time
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from tf2_msgs.msg import TFMessage
    from tf2_ros import TransformBroadcaster

    rclpy.init()
    publisher, observer = Node("synthetic_teaching_tf"), Node("synthetic_tf_viewer")
    executor = SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(observer)
    messages, sent = [], []
    broadcaster = TransformBroadcaster(publisher)
    observer.create_subscription(TFMessage, "/tf", messages.append, 10)
    context = SimpleNamespace(
        selection_lock=threading.RLock(), selected_pose=None, arm_epoch=7, yolo_enabled=True,
        native=SimpleNamespace(failed=False), fatal_error=None, service=None, events=MagicMock(),
        get_clock=publisher.get_clock, validate_simulation_view=lambda _: None,
        applied=SimpleNamespace(platform=SimpleNamespace(base_from_platform=np.eye(4))),
        settings={"quality": {"result_max_age_sec": 2.}})

    def send(transforms):
        sent.append(deepcopy(transforms))
        broadcaster.sendTransform(transforms)

    context.selected_pose_broadcaster = SimpleNamespace(sendTransform=send)
    context.clear_selected_pose = MethodType(gui.ItemTeachNode.clear_selected_pose, context)
    publisher.create_timer(.1, lambda: gui.ItemTeachNode._broadcast_selected_pose(context))

    def install(count):
        response, view = batch(count)
        now = publisher.get_clock().now()
        response.header.stamp, view["stamp_ns"] = now.to_msg(), now.nanoseconds
        gui.ItemTeachNode.show_simulated_poses(context, response, view)

    def spin_until(predicate, timeout=5.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not predicate():
            executor.spin_once(timeout_sec=.05)
        assert predicate(), "Expected synthetic teaching TF was not received"

    try:
        install(5)
        spin_until(lambda: bool(messages))
        assert [t.child_frame_id for t in messages[-1].transforms] == [
            f"item_teach_candidate_{i}" for i in range(1, 6)]
        assert all(t.header.frame_id == "base_link" for t in messages[-1].transforms)
        assert messages[-1].transforms[0].transform.translation.z == .1
        assert len({(t.header.stamp.sec, t.header.stamp.nanosec)
                    for t in messages[-1].transforms}) == 1
        install(2)
        spin_until(lambda: len(messages[-1].transforms) == 2)
        context.clear_selected_pose()
        stopped_count = len(sent)
        end = time.monotonic() + .35
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.05)
        assert len(sent) == stopped_count
        assert context.service is None
        assert not any(name in sys.modules for name in ("cv2", "torch", "ultralytics"))
        print("Synthetic TF transport: all priorities, replacement and clear passed")
    finally:
        executor.shutdown(timeout_sec=2)
        publisher.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()


def test_simulated_batch_ros_tf_transport():
    code = "import runpy,sys; runpy.run_path(sys.argv[1])['exercise_tf_transport']()"
    result = subprocess.run(
        ["/usr/bin/python3", "-c", code, str(Path(__file__).resolve())],
        env=dict(os.environ, ROS_DOMAIN_ID="231", ROS_LOCALHOST_ONLY="1"),
        capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
