"""Attended placement with real transport and synthetic robot feedback only."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rclpy.action import GoalResponse
from robot_controller_interfaces.action import PlaceItem

from robot_controller.controller import RobotController
from robot_controller.errors import OperationCanceled
from robot_controller.state_machine import ControllerStateMachine
from test_feedback_v2 import joint_message
from test_placement import operation_node, run_place
from test_placement_queue import QueueRig, HELD, OPEN


@pytest.mark.parametrize('headless', [False, True])
@pytest.mark.parametrize('held', [False, True])
def test_explicit_place_admission_allows_empty_or_held_in_either_mode(headless, held):
    node = operation_node()
    node.headless, node.holding_item = headless, held
    if not held:
        node.managed.session = None
    node.machine = ControllerStateMachine(initial='HOLDING' if held else 'READY')
    node.configuration.configuration_id = 'bound'
    node._begin_operation = Mock()
    node._perception_ready = Mock(return_value=True)
    assert RobotController._reserve_goal(node, 'place', 'bound') == GoalResponse.ACCEPT
    node._begin_operation.reset_mock()
    assert RobotController._reserve_goal(node, 'place', 'stale') == GoalResponse.REJECT
    node._perception_ready.return_value = False
    assert RobotController._reserve_goal(node, 'place', 'bound') == GoalResponse.REJECT
    node._begin_operation.assert_not_called()


@pytest.mark.parametrize('during_reply', [False, True])
@pytest.mark.parametrize('held_context,outputs,inputs', [
    (False, 0, 0), (False, HELD, 1), (True, HELD, 1),
])
def test_manual_queue_reaches_retract_regardless_of_initial_item_presence(
        during_reply, held_context, outputs, inputs):
    rig = QueueRig(during_reply=during_reply)
    node = rig.node
    node.placement.require_held_item = False
    node.holding_item = held_context
    if not held_context:
        node.managed.session = None
    node.expected_outputs = {ch: bool(outputs & (1 << (ch - 1))) for ch in (1, 2, 13, 14)}
    for _ in range(32):  # Exceed the real DI1 LOW debounce, including a stale held record.
        rig.emit(outputs=outputs, inputs=inputs, running=0)
    assert rig.monitor.snapshot(require_enabled=True).suction_present is bool(inputs)
    rig.script[0] = dict(outputs=outputs, inputs=inputs)
    rig.run()
    assert [name for name, _ in rig.requests] == ['MovL', 'MovLIO', 'MovLIO']
    assert node.placement.phase == 'DONE' and not node.holding_item
    assert not any(node.expected_outputs.values())
    node._preflight_item_state.assert_not_called()
    assert node.trays.request.call_args.kwargs['require_held_item'] is False
    assert node.trays.request.call_args.kwargs['attempts'] is node.placement.tray_attempts
    if held_context:
        assert node.managed.session.attempts[0].state == 'PLACED'
    else:
        assert node.managed.session is None  # Never invent a picked candidate.


@pytest.mark.parametrize('held_context', [False, True])
@pytest.mark.parametrize('inputs', [0, 1])
def test_manual_observation_move_has_no_item_presence_gate(held_context, inputs):
    rig = QueueRig()
    node = rig.node
    node.placement.require_held_item = False
    node.holding_item = held_context
    for _ in range(32):
        rig.emit(outputs=HELD, inputs=inputs, running=0)
    joints = joint_message()
    joints.position[0] = .2
    rig.monitor.update_joints(joints)
    node.kinematics = SimpleNamespace(forward=lambda _j: node.configuration.tray.detect_matrix)
    rig.steps = iter([dict(outputs=HELD, inputs=inputs, home=True)])

    def arrived(*args, **kwargs):
        rig.monitor.update_joints(joint_message())
        return rig.next_sample(*args, **kwargs)

    rig.monitor.wait_next = arrived
    RobotController._execute_tray_position(node)
    assert [name for name, _ in rig.requests] == ['MovL']
    assert rig.requests[0][1].mode
    node._preflight_item_state.assert_not_called()


@pytest.mark.parametrize('held', [False, True])
@pytest.mark.parametrize('at_tray', [False, True])
def test_place_moves_to_tray_only_if_needed_and_detects_after_arrival(held, at_tray):
    rig = QueueRig()
    node = rig.node
    node.placement.require_held_item = False
    node.holding_item = held
    if not held:
        node.managed.session = None
    outputs, inputs = (HELD, 1) if held else (0, 0)
    node.expected_outputs = {ch: bool(outputs & (1 << (ch - 1))) for ch in (1, 2, 13, 14)}
    for _ in range(32):
        rig.emit(outputs=outputs, inputs=inputs, running=0)
    if not at_tray:
        joints = joint_message()
        joints.position[0] = .2
        origin = node.configuration.tray.detect_matrix.copy()
        origin[0, 3] += .1
        rig.joint_poses[tuple(joints.position)] = origin
        rig.monitor.update_joints(joints)
    node._execute_tray_position = Mock(
        side_effect=lambda: RobotController._execute_tray_position(node))
    rig.transport.current_pose = lambda: rig.transport.pose_from_snapshot(rig.monitor.snapshot())
    rig.script[0] = dict(outputs=outputs, inputs=inputs)
    rig.steps = iter(([] if at_tray else [dict(outputs=outputs, inputs=inputs, home=True)])
                     + rig.script)

    def observe(*_args, **kwargs):
        assert rig.transport.home_already_reached(node.configuration.tray.detect_joints)
        assert len(rig.requests) == (0 if at_tray else 1)
        assert node.expected_outputs == {ch: bool(outputs & (1 << (ch - 1)))
                                         for ch in (1, 2, 13, 14)}
        assert not kwargs['require_held_item']
        return [.3, .2, .25]

    node.trays.request.side_effect = observe
    node.placement.run(node)
    assert node._execute_tray_position.call_count == (0 if at_tray else 1)
    assert [name for name, _ in rig.requests] == (
        [] if at_tray else ['MovL']) + ['MovL', 'MovLIO', 'MovLIO']
    if not at_tray:
        assert rig.requests[0][1].mode
        assert 'v=100' in rig.requests[0][1].param_value
    node.placement.finish_pending(node)
    assert node.placement.phase == 'DONE' and not node.holding_item
    assert not any(node.expected_outputs.values())


@pytest.mark.parametrize('inputs', [0, OPEN | 1])
def test_manual_mode_finishes_retract_with_optional_release_evidence(inputs):
    rig = QueueRig()
    rig.node.placement.require_held_item = False
    rig.script[1]['inputs'] = inputs
    rig.run()
    assert rig.node.placement.phase == 'DONE'
    assert rig.node.placement.release_confirmed == (not bool(inputs & 1))


def empty_node():
    node = operation_node()
    node.headless = False
    node.holding_item = False
    node.managed.session = None
    node.hardware.suction = False
    node.hardware.outputs = dict.fromkeys((1, 2, 13, 14), False)
    node.expected_outputs = dict(node.hardware.outputs)
    node.placement.require_held_item = False
    node._execute_tray_position.side_effect = lambda: setattr(
        node.hardware, 'current', node.configuration.tray.detect_matrix.copy())
    return node


@pytest.mark.parametrize('headless', [False, True])
def test_empty_action_transitions_ready_to_placing_and_back_without_a_pick(headless):
    node = empty_node()
    node.headless = headless
    node.machine = ControllerStateMachine(initial='READY')
    node._end_operation = Mock()
    goal = SimpleNamespace(request=PlaceItem.Goal(x_mm=30., y_mm=40., rotation_deg=-90.),
                           succeed=Mock())
    result = RobotController._execute_place_action(node, goal)
    assert result.outcome == PlaceItem.Result.SUCCESS and result.final_state == 'PLACING'
    node.placement_thread.join(timeout=1.)
    assert node.managed.session is None and node.placement is None
    node._preflight_item_state.assert_not_called()
    goal.succeed.assert_called_once()


@pytest.mark.parametrize('previous_state', ['DROPPED', 'RETURNED', 'PLACED'])
def test_manual_placement_does_not_relabel_a_previous_terminal_candidate(previous_state):
    node = empty_node()
    node.managed.session = operation_node().managed.session
    node.managed.session.set_state(1, previous_state)
    run_place(node)
    assert node.placement.phase == 'DONE'
    assert node.managed.session.attempts[0].state == previous_state
    assert node.managed.session.held_index is None


@pytest.mark.parametrize('at', [1, 2])
def test_empty_pause_preserves_mode_and_resumes_release_recovery(at):
    node = empty_node()
    node.hardware.interrupt_at = at
    with pytest.raises(OperationCanceled):
        run_place(node)
    node.machine.transition('PAUSING', 'Pause')
    node.managed.kind = 'pause'
    node.wait_control = lambda _seconds: node.managed.resume.set()
    node.placement.handle_pause(node)
    assert node.machine.state == 'PLACING'
    run_place(node)
    assert node.placement.phase == 'DONE'
    assert node.trays.request.call_count == 1
    assert not node.placement.require_held_item
    node._preflight_item_state.assert_not_called()
