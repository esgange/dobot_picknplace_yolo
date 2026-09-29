import copy
import time
from types import MethodType, SimpleNamespace
from unittest.mock import MagicMock

import pytest
from rclpy.time import Time

from tray_perception.node import TrayTeachNode
from tray_perception.simulation import FRAME, TraySimulationPreview
from test_requests import artifact as artifact_fixture, backend as backend_fixture, arm, trigger
from test_core import camera_info, settings


artifact, backend = artifact_fixture, backend_fixture


@pytest.fixture
def simulated(backend):
    node, path, digest = backend
    value = node.requests.simulate(path, settings(), digest, (100_000_000_000, time.monotonic()))
    now = [101.]
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=now[0]))
    node.broadcaster = MagicMock()
    node.rviz = SimpleNamespace(clear_pose=MagicMock(), show_pose=MagicMock(), accept=MagicMock())
    node.color_info = camera_info()
    node._check_snapshot = MethodType(TrayTeachNode._check_snapshot, node)
    node.selected = "previous live pose"
    node.simulation = TraySimulationPreview(node)
    return node, value, now


def test_frozen_tf_matches_response_and_does_not_follow_robot_or_live_preview(simulated):
    node, value, now = simulated
    response, view = value["response"], value["view"]
    node.simulation.install(response, view)
    assert node.selected is None
    assert not any(key in node.simulation.binding for key in ("rgb", "depth", "overlay"))
    assert node.simulation.tick()
    first = copy.deepcopy(node.broadcaster.sendTransform.call_args.args[0])
    node.selected = (4, {"position": [9., 9., 9.]}, 123, time.monotonic())
    now[0] += 60  # Historical geometry remains visible, labelled stale in the GUI.
    assert node.simulation.tick()
    second = node.broadcaster.sendTransform.call_args.args[0]
    assert first.transform == second.transform
    assert first.header.stamp != second.header.stamp
    assert second.child_frame_id == FRAME and second.header.frame_id == "base_link"
    assert second.transform.translation.x == response.tray.pose.position.x
    assert second.transform.rotation == response.tray.pose.orientation
    node.simulation.clear()
    assert not node.simulation.tick() and node.selected is None


@pytest.mark.parametrize("changed", ["epoch", "generation", "yaml", "model", "camera_info",
                                     "yolo", "worker", "fatal"])
def test_timer_revokes_frozen_pose_without_gui_processing(simulated, changed):
    node, value, _ = simulated
    node.simulation.install(value["response"], value["view"])
    if changed == "epoch":
        node.requests.disarm()
    elif changed == "generation":
        node.generation += 1
    elif changed == "yaml":
        value["view"]["trigger_binding"]["path"].write_text("changed")
    elif changed == "model":
        value["view"]["trigger_binding"]["paths"][1].write_bytes(b"changed")
    elif changed == "camera_info":
        node.color_info = None
    elif changed == "yolo":
        node.yolo_enabled = False
    elif changed == "worker":
        node.native.failed = True
    else:
        node.fatal_error = "executor stopped"
    assert not node.simulation.tick()
    node.broadcaster.sendTransform.assert_not_called()
    assert node.simulation.binding is None
    node.rviz.clear_pose.assert_called()


@pytest.mark.parametrize("changed", ["stamp", "frame", "position", "orientation", "id",
                                     "found", "count", "status", "success"])
def test_mismatched_response_never_installs_a_pose(simulated, changed):
    node, value, now = simulated
    response = copy.deepcopy(value["response"])
    if changed == "stamp":
        response.header.stamp.nanosec += 1
    elif changed == "frame":
        response.header.frame_id = "camera"
    elif changed == "position":
        response.tray.pose.position.z += .01
    elif changed == "orientation":
        response.tray.pose.orientation.w = 0.
    elif changed == "id":
        response.tray.id = "old:1"
    elif changed == "found":
        response.found = False
    elif changed == "count":
        response.valid_count += 1
    elif changed == "status":
        response.status = "NO_VALID_TRAY"
    else:
        response.success = False
    with pytest.raises(ValueError):
        node.simulation.install(response, value["view"])
    assert node.simulation.binding is None and not node.simulation.tick()


def test_empty_batch_removes_old_pose_and_emits_no_transform(simulated):
    node, value, _ = simulated
    node.simulation.install(value["response"], value["view"])
    response = value["response"]
    response.found, response.status, response.valid_count = False, "NO_VALID_TRAY", 0
    response.tray.id = ""
    value["view"]["result"].update(selected=None, detections=[])
    node.simulation.install(response, value["view"])
    assert node.simulation.tick()
    assert node.simulation.transform is None and node.selected is None
    node.broadcaster.sendTransform.assert_not_called()


@pytest.mark.parametrize("empty", [False, True])
def test_simulation_timer_expires_without_gui_and_replacement_restarts_hold(
        simulated, monkeypatch, empty):
    from tray_perception import simulation
    now = [100.]
    monkeypatch.setattr(simulation, "time", SimpleNamespace(monotonic=lambda: now[0]))
    node, value, _ = simulated
    response, view = value["response"], value["view"]
    if empty:
        response.found, response.status, response.valid_count = False, "NO_VALID_TRAY", 0
        response.tray.id = ""
        view["result"].update(selected=None, detections=[])
    node.simulation.install(response, view)
    now[0] = 109.999
    assert node.simulation.tick()
    node.simulation.install(response, view)
    now[0] = 110.
    assert node.simulation.tick()
    node.broadcaster.sendTransform.reset_mock()
    now[0] = 119.999
    assert not node.simulation.tick()
    assert node.simulation.binding is None and node.simulation.expires_at is None
    node.broadcaster.sendTransform.assert_not_called()
    node.rviz.clear_pose.assert_called()


def test_frozen_cloud_is_accepted_once_without_retaining_images_in_tf_state(simulated):
    node, value, _ = simulated
    cloud = {"point_count": 1, "data": bytes(16)}
    value["view"]["cloud"] = cloud
    node.simulation.install(value["response"], value["view"])
    node.simulation.tick()
    node.simulation.tick()
    node.rviz.accept.assert_called_once_with(cloud)
    assert "cloud" not in node.simulation.binding


def test_armed_request_while_frozen_still_acquires_a_new_observation(backend):
    node, path, digest = backend
    api = arm(backend)
    local = api.simulate(path, settings(), digest, (100_000_000_000, time.monotonic()))
    node.broadcaster, node.rviz = MagicMock(), MagicMock()
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=101))
    node.simulation = TraySimulationPreview(node)
    node.simulation.install(local["response"], local["view"])
    snapshot = node.snapshot.side_effect

    def fresh():
        view = snapshot()
        view["rgb"]["stamp_ns"] = 101_000_000_001
        return view
    node.snapshot.side_effect = fresh
    actual = trigger(backend)
    assert actual.success and actual.header.stamp.sec == 101
    assert actual.batch_id != local["response"].batch_id
    assert node.preview.call_count == 2
    assert node.simulation.transform.header.stamp.sec == 100
    assert node.requests.service is not None
