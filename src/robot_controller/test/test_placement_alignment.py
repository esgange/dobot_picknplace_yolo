"""Item/tray axis behavior across arbitrary grasps, tilt, preview and transport."""

from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from robot_controller_interfaces.msg import ControllerStatus
from robot_controller_interfaces.srv import Preview

from robot_controller.errors import FeedbackFailure
from robot_controller.kinematics import pose_matrix
from robot_controller.motion import pick_attitude
from robot_controller.placement import place_targets
from robot_controller.placement_alignment import aligned_rotation, held_item_axis
from test_feedback_v2 import feed
from test_operation_retries import pick_rig, TrayRig
from test_placement import operation_node, tray_observation
from test_preview_routes import preview  # noqa: F401


def unit(vector):
    return vector / np.linalg.norm(vector)


@pytest.mark.parametrize('tray_yaw', [-135., -90., 0., 90., 179.])
@pytest.mark.parametrize('pick_offset', [0., 25., 90.])
@pytest.mark.parametrize('mirrored', [False, True])
@pytest.mark.parametrize('placement_offset', [0., 30., -90.])
def test_placement_follows_tray_for_actual_grasp_and_keeps_taught_tilt(
        tray_yaw, pick_offset, mirrored, placement_offset):
    home = pose_matrix([0, 0, 800, 176, 5, 23])
    item = pose_matrix([0, 0, 200, 3, 4, -71])
    pickup, *_ = pick_attitude(home, item, pick_offset)
    if mirrored:
        pickup = pickup @ np.diag([-1., -1., 1.])
    axis = pickup.T @ item[:3, 0]
    detect = pose_matrix([0, 0, 800, 179, 2, -38])
    tray = pose_matrix([0, 0, 200, 1, 3, tray_yaw])
    placed = aligned_rotation(detect, tray[:3, :3], axis, placement_offset)
    assert np.allclose(placed[:, 2], detect[:3, 2], atol=1e-12)
    assert np.allclose(placed.T @ placed, np.eye(3), atol=1e-12)
    assert np.linalg.det(placed) == pytest.approx(1.)
    actual = unit((detect[:3, :3].T @ placed @ axis)[:2])
    expected = unit((detect[:3, :3].T @ tray[:3, 0])[:2])
    angle = np.deg2rad(placement_offset)
    expected = np.array([[np.cos(angle), -np.sin(angle)],
                         [np.sin(angle), np.cos(angle)]]) @ expected
    assert np.allclose(actual, expected, atol=1e-12)


def test_quarter_turn_of_tray_turns_item_and_tray_local_position_together():
    node = operation_node()
    axis = held_item_axis(node.managed.session, node.configuration)
    origin = np.array([.3, .2, .25])
    offset = np.array([.03, .04, 0.])
    results = []
    for yaw in (0., 90.):
        rotation = pose_matrix([0, 0, 0, 0, 0, yaw])[:3, :3]
        observation = tray_observation(origin + rotation @ offset, rotation)
        plan = place_targets(node.configuration.tray.detect_matrix, observation,
                             node.configuration.profile, 0., node.configuration.home_matrix,
                             item_x_in_tool=axis)
        results.append(plan[1].matrix)
        assert np.allclose(unit(plan[1].matrix[:3, :3] @ axis), rotation[:, 0])
    assert np.allclose(results[0][:2, 3], [.33, .24])
    assert np.allclose(results[1][:2, 3], [.26, .23])
    assert abs(np.dot(results[0][:3, :3] @ axis, results[1][:3, :3] @ axis)) < 1e-12


def test_tray_z_sign_never_flips_the_tool():
    detect = pose_matrix([0, 0, 800, 177, 2, 20])
    a = pose_matrix([0, 0, 0, 0, 0, 45])[:3, :3]
    b = a @ np.diag([1., -1., -1.])
    assert np.allclose(aligned_rotation(detect, a, [0., 1., 0.], 0.),
                       aligned_rotation(detect, b, [0., 1., 0.], 0.))


@pytest.mark.parametrize('axis', [[0., 0., 1.], [0., 2., 0.], [float('nan'), 1., 0.]])
def test_invalid_or_vertical_item_axis_rejects_before_planning(axis):
    with pytest.raises(ValueError):
        aligned_rotation(np.eye(4), np.eye(3), axis, 0.)


def test_pick_saves_measured_grasp_before_transport_and_uses_bound_candidate(monkeypatch):
    node = pick_rig(monkeypatch, [1], [True])
    move = node.hardware.move_batch
    pickups = []

    def observe(*args, **kwargs):
        result = move(*args, **kwargs)
        if kwargs.get('return_terminal_pose') and result[0]:
            pickups.append(result[1].copy())
        return result
    node.hardware.move_batch = observe
    assert node.execute().outcome == 0
    session = node.managed.session
    assert np.array_equal(session.attempts[0].pickup_matrix, pickups[0])
    axis = held_item_axis(session, node.configuration)
    assert np.linalg.norm(axis) == pytest.approx(1.)
    with pytest.raises(FeedbackFailure, match='retained'):
        held_item_axis(session, SimpleNamespace())
    session.set_state(1, 'DROPPED')
    with pytest.raises(FeedbackFailure, match='confirmed'):
        held_item_axis(session, node.configuration)


@pytest.mark.parametrize('missing', ['pickup', 'batch', 'configuration', 'unknown_item'])
def test_missing_grasp_cannot_dispatch_observation_travel_or_placement(missing):
    node = operation_node()
    if missing == 'pickup':
        node.managed.session.attempts[0].pickup_matrix = None
    elif missing == 'batch':
        node.managed.session.batch = None
    elif missing == 'configuration':
        node.managed.session.configuration = None
    else:
        node.holding_item = False  # DI1 HIGH, no trusted source.
        node.managed.session = None
    with pytest.raises(FeedbackFailure, match='alignment'):
        node.placement.run(node)
    assert not node.hardware.calls
    node._execute_tray_position.assert_not_called()
    node.trays.request.assert_not_called()


@pytest.mark.parametrize('damage', [
    None, 'stale', 'receipt', 'configuration', 'unknown', 'changed'])
def test_held_preview_requires_same_fresh_grasp_as_hardware(
        preview, damage):  # noqa: F811
    preview.monitor.update_feed(feed(digital_input_bits=1))
    status = ControllerStatus(configuration_id=preview.config.configuration_id,
                              holding_item=True, held_item_alignment_known=True,
                              held_item_x_tool=[1., 0., 0.], feedback_fresh=True)
    status.header.stamp = Time(sec=1)
    preview._on_controller_status(status)
    if damage == 'stale':
        preview.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=3_000_000_000))
    elif damage == 'configuration':
        status.configuration_id = 'old'
    elif damage == 'unknown':
        status.held_item_alignment_known = False
    elif damage == 'receipt':
        preview.controller_status_received -= 2.
    elif damage == 'changed':
        def observation(*args, **kwargs):
            status.held_item_x_tool = [0., 1., 0.]
            return tray_observation()
        preview.trays.request.side_effect = observation
    result = preview.run(Preview.Request.PLACE, x_mm=30., y_mm=40.)
    assert result.success is (damage is None)
    if damage is None:
        target = preview.targets[1].matrix
        assert np.allclose(target[:3, 0], tray_observation().rotation[:, 0])
    else:
        assert not preview.targets


def test_tray_client_keeps_quaternion_with_its_validated_depth_point(monkeypatch):
    rig = TrayRig(monkeypatch, ['valid'])
    pose = rig.result.tray.pose
    pose.orientation.z = pose.orientation.w = np.sqrt(.5)
    rig.result.placement.surface_base.x = .06
    rig.result.placement.surface_base.y = .23
    observation = rig.request()
    assert observation.surface == pytest.approx([.06, .23, .25])
    assert observation.rotation[:, 0] == pytest.approx([0., 1., 0.])
