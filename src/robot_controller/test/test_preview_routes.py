"""Read-only nominal routes, fresh inputs and cancellation without robot clients."""

from types import SimpleNamespace
import threading
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.time import Time
from robot_controller_interfaces.srv import Preview

import robot_controller.preview as preview_module
from robot_controller.preview import RobotControllerPreview
from robot_controller.errors import FeedbackFailure
from robot_controller.kinematics import pose_matrix
from robot_controller.motion import cartesian_home_targets, pick_targets
from robot_controller.placement import place_targets
from test_feedback_v2 import feed, primed_monitor
from test_placement import operation_node
from test_operation_retries import TrayRig


@pytest.fixture
def preview(monkeypatch):
    config = operation_node().configuration
    config.configuration_id = "preview-config"
    config.home_joints = (.1,) * 6
    config.home_matrix = pose_matrix([500, 150, 900, 175, 10, 20])
    config.tray.detect_joints = (0.,) * 6
    config.selection = SimpleNamespace(
        station=SimpleNamespace(platform=SimpleNamespace(base_from_platform=np.eye(4))),
        robot_camera=SimpleNamespace(reference_from_camera_link=np.eye(4)),
        bin=SimpleNamespace(points=[]))
    origin = pose_matrix([100, -200, 200, 170, 8, 30])
    node = SimpleNamespace(
        root=None, bringup_node="dobot", monitor=primed_monitor(),
        cancel=threading.Event(), lock=threading.Lock(), state_lock=threading.RLock(),
        generation=0, configuration=None, origin=None, request_origin=None,
        targets=(), frames=(), events=Mock(), broadcaster=Mock(),
        kinematics=SimpleNamespace(forward=Mock(return_value=origin)),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=1.)),
        get_publishers_info_by_topic=lambda _: [SimpleNamespace(
            node_name="dobot", node_namespace="/")])
    for name in ("_preview", "_clear", "_snapshot", "_broadcast", "_check_observation",
                 "wait_for_resume", "operation_progress", "_on_joints", "_on_feed"):
        setattr(node, name, getattr(RobotControllerPreview, name).__get__(node))
    candidates = [SimpleNamespace(
        position_m=(.1 * i, .2, .3), quaternion=(0., 0., 0., 1.)) for i in range(2)]
    node.client = SimpleNamespace(request=Mock(return_value=SimpleNamespace(candidates=candidates)))
    node.trays = SimpleNamespace(request=Mock(return_value=np.array([.3, .2, .25])))
    monkeypatch.setattr(preview_module, "load_configuration", Mock(return_value=config))
    monkeypatch.setattr(preview_module, "select_pick_attitude", lambda *_args: SimpleNamespace(
        accepted=True, rotation=config.home_matrix[:3, :3]))
    node.config = config
    node.run = lambda operation, **kwargs: node._preview(
        Preview.Request(operation=operation, item_teach_file="item", bin_teach_file="bin",
                        tray_teach_file="tray", **kwargs), Preview.Response())
    return node


def test_home_preview_matches_both_cartesian_commands_from_actual_joint_pose(preview):
    result = preview.run(Preview.Request.HOME)
    assert result.success
    assert [t.name for t in preview.targets] == ["home_align", "home"]
    expected = cartesian_home_targets(
        preview.kinematics.forward(()), preview.config.home_matrix,
        speed_percent=preview.config.profile["speed"]["travel_percent"],
        acceleration_percent=preview.config.profile["acceleration"]["travel_percent"])
    assert all(np.allclose(a.matrix, b.matrix) for a, b in zip(preview.targets, expected))
    assert all(t.joints_rad is None for t in preview.targets)
    preview.client.request.assert_not_called()
    preview.trays.request.assert_not_called()
    preview._broadcast()
    messages = preview.broadcaster.sendTransform.call_args.args[0]
    assert [m.child_frame_id for m in messages] == result.tf_frames
    assert all(m.header.frame_id == "base_link" for m in messages)


def test_home_preview_skips_motion_when_already_at_cartesian_home(preview):
    preview.kinematics.forward.return_value = preview.config.home_matrix.copy()
    result = preview.run(Preview.Request.HOME)
    assert result.success and not result.tf_frames
    assert "Already at Home" in result.message


def test_pick_preview_includes_success_tray_and_missed_or_put_back_home_branches(preview):
    result = preview.run(Preview.Request.PICK)
    assert result.success
    targets = {t.name: t for t in preview.targets}
    assert {"home_height", "home"} <= targets.keys()
    for index in (1, 2):
        for suffix in ("transit", "initial", "prepick", "pick", "retract", "final",
                       "transit_exit", "return_home", "put_back_return_release",
                       "put_back_return_clearance", "put_back_return_park_transit"):
            assert f"p{index}_{suffix}" in targets
        item = np.eye(4)
        item[:3, 3] = [.1 * (index - 1), .2, .3]
        plan = pick_targets(preview.config.home_matrix, item, preview.config.profile,
                            index, rotation=preview.config.home_matrix[:3, :3])
        assert np.allclose(targets[f"p{index}_pick"].matrix, plan[3].matrix)
        assert targets[f"p{index}_return_home"].joints_rad == preview.config.home_joints
        destination = targets[f"p{index}_success_tray_detect"]
        assert destination.joints_rad == preview.config.tray.detect_joints
        assert np.array_equal(destination.matrix, preview.config.tray.detect_matrix)
        assert destination.speed_percent == preview.config.profile["speed"]["travel_percent"]
    assert len(set(result.tf_frames)) == len(preview.targets)
    assert "one fresh batch" in result.message
    preview.client.request.assert_called_once()
    preview.trays.request.assert_not_called()
    assert preview_module.load_configuration.call_args.kwargs['tray_path'] == 'tray'


@pytest.mark.parametrize('tray', [None, SimpleNamespace(detect_joints=None)])
def test_pick_preview_rejects_missing_tray_destination_before_detection(preview, tray):
    preview.config.tray = tray
    result = preview.run(Preview.Request.PICK)
    assert not result.success and 'Tray Detect Pose' in result.message
    assert not preview.targets
    preview.client.request.assert_not_called()


def test_pick_preview_skips_initial_home_when_joint_feedback_is_at_home(preview):
    preview.config.home_joints = (0.,) * 6
    result = preview.run(Preview.Request.PICK)
    assert result.success
    assert not {'home_height', 'home'} & {t.name for t in preview.targets}


def test_pick_preview_empty_observation_never_invents_candidates(preview):
    preview.client.request.return_value.candidates = []
    result = preview.run(Preview.Request.PICK)
    assert result.success and "no valid item candidates" in result.message
    assert [t.name for t in preview.targets] == ["home_height", "home"]


@pytest.mark.parametrize("angle", [-180., 0., 90., 180.])
def test_place_preview_contains_only_three_placement_targets(preview, angle):
    # Preview remains read-only and works without Startup/EnableRobot.
    preview.monitor.update_feed(feed(EnableStatus=0, robot_mode=4))
    preview.monitor.update_status(SimpleNamespace(is_connected=True, is_enable=False))
    preview.config.profile["motion"]["trayplace_height"] = 42.5
    result = preview.run(Preview.Request.PLACE, x_mm=30., y_mm=40., rotation_deg=angle)
    assert result.success
    assert [t.name for t in preview.targets] == [
        "place_pre", "place_release", "place_retract"]
    expected = place_targets(preview.config.tray.detect_matrix, [.3, .2, .25],
                             preview.config.profile, angle, preview.config.home_matrix)
    assert all(t.joints_rad is None for t in preview.targets)
    assert preview.targets[1].matrix[2, 3] == pytest.approx(.2925)
    assert all(np.allclose(a.matrix, b.matrix) for a, b in zip(preview.targets, expected))
    preview.trays.request.call_args.kwargs["check_state"]()
    preview.client.request.assert_not_called()


def test_place_preview_away_from_tray_shows_observation_travel_without_detection(preview):
    preview.config.tray.detect_joints = (.1,) * 6
    result = preview.run(Preview.Request.PLACE, x_mm=30., y_mm=40.)
    assert result.success and 'Tray Detect travel only' in result.message
    preview.trays.request.assert_not_called()
    assert [t.name for t in preview.targets] == ['tray_detect_position']
    assert preview.targets[0].joints_rad == preview.config.tray.detect_joints
    assert preview.targets[0].speed_percent == 100
    assert not preview.targets[0].motion_io


@pytest.mark.parametrize("failure", ["perception", "sources", "moving", "ownership", "stale"])
def test_preview_faults_clear_previous_targets_and_never_install_partial_route(preview, failure):
    assert preview.run(Preview.Request.HOME).success
    if failure == "perception":
        preview.trays.request.side_effect = FeedbackFailure("no tray")
    elif failure == "sources":
        preview.config.validate_sources.side_effect = ValueError("changed file")
    elif failure == "moving":
        preview.monitor.update_feed(feed(RunningStatus=1))
    elif failure == "ownership":
        preview.get_publishers_info_by_topic = lambda _: []
    else:
        preview.monitor._ros_now_ns = lambda: 4_000_000_000
    result = preview.run(Preview.Request.PLACE, x_mm=30., y_mm=40.)
    assert not result.success and not result.tf_frames
    assert preview.configuration is None and not preview.targets


@pytest.mark.parametrize("operation", [Preview.Request.PICK, Preview.Request.PLACE])
def test_clear_during_perception_preempts_owner_and_discards_late_result(preview, operation):
    client = preview.client if operation == Preview.Request.PICK else preview.trays
    reply = client.request.return_value

    def clear_then_reply(*_args, **_kwargs):
        assert preview.lock.locked()
        assert preview.run(Preview.Request.CLEAR).success
        return reply
    client.request.side_effect = clear_then_reply
    result = preview.run(operation, x_mm=30., y_mm=40.)
    assert not result.success and "cancel" in result.message.lower()
    preview._broadcast()
    preview.broadcaster.sendTransform.assert_not_called()
    assert not preview.targets and preview.configuration is None


def test_preview_tf_publication_stops_when_actual_robot_moves(preview):
    assert preview.run(Preview.Request.HOME).success
    preview.kinematics.forward.return_value = preview.config.home_matrix.copy()
    preview._broadcast()
    assert preview.configuration is None
    preview.broadcaster.sendTransform.assert_not_called()


def test_invalid_joint_message_cancels_an_installed_preview(preview):
    assert preview.run(Preview.Request.HOME).success
    preview._on_joints(SimpleNamespace(name=[], position=[]))
    assert preview.cancel.is_set() and not preview.targets


def test_read_only_tray_requests_keep_retry_bound_without_requiring_robot_enable(monkeypatch):
    rig = TrayRig(monkeypatch, [None, None, "valid"])
    rig.node.monitor.snapshot.side_effect = FeedbackFailure("disabled robot")
    stationary = Mock()
    point = rig.request(require_held_item=False, check_state=stationary)
    assert point == pytest.approx([.13, .24, .25]) and rig.attempts.count == 3
    assert stationary.call_count >= 6
    rig.node.monitor.snapshot.assert_not_called()
    rig.node._preflight_item_state.assert_not_called()
