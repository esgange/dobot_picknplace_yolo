"""Read-only tray arrival and asynchronous queue ownership, using synthetic feedback."""

import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rclpy.action import GoalResponse
from robot_controller_interfaces.action import PlaceItem

import robot_controller.hardware as hardware_module
from robot_controller.controller import RobotController
from robot_controller.errors import FeedbackFailure, OperationCanceled, CommandRejected
from robot_controller.hardware import DobotTransport
from robot_controller.state_machine import ControllerStateMachine
from test_joint_position import PositionRig
from test_placement import operation_node
from test_placement_queue import QueueRig


def test_tray_position_already_idle_and_matched_has_no_wait_or_motion():
    rig = PositionRig()
    rig.transport.wait_tray_position((0.,) * 6)
    assert rig.waits == 0
    assert rig.monitor.sequence == 1


@pytest.mark.parametrize('idle,joints', [(False, (0.,) * 6), (True, (.1,) * 6)])
def test_tray_position_waits_three_seconds_without_moving_or_querying(monkeypatch, idle, joints):
    rig = PositionRig()
    clock = [0.]
    monkeypatch.setattr(hardware_module.time, 'monotonic', lambda: clock[0])
    rig.publish(joints=joints, idle=idle)

    def tick(_revision, duration, **kwargs):
        assert kwargs['position']
        clock[0] += duration
        rig.publish(joints=joints, idle=idle)
    rig.monitor.wait_next = tick
    with pytest.raises(FeedbackFailure, match='Not at Tray Detect position after 3 seconds'):
        rig.transport.wait_tray_position((0.,) * 6)
    assert clock[0] == pytest.approx(3.)
    assert rig.monitor.sequence == 1  # Only the joint/status streams advanced.


def test_tray_position_proceeds_as_soon_as_both_streams_match(monkeypatch):
    rig = PositionRig()
    clock = [0.]
    monkeypatch.setattr(hardware_module.time, 'monotonic', lambda: clock[0])
    rig.publish(joints=(.1,) * 6, idle=False)
    steps = iter([dict(joints=(0.,) * 6, idle=False), dict(joint_update=False, idle=True)])

    def tick(*_args, **_kwargs):
        clock[0] += .1
        rig.publish(**next(steps))
    rig.monitor.wait_next = tick
    rig.transport.wait_tray_position((0.,) * 6)
    assert clock[0] == pytest.approx(.2)


@pytest.mark.parametrize('reason', ['stop', 'stale', 'fault'])
def test_tray_wait_preserves_stop_freshness_and_fault_gates(reason):
    rig = PositionRig()
    if reason == 'stop':
        rig.transport.node.raise_if_cancelled = Mock(side_effect=OperationCanceled('Stop'))
    elif reason == 'stale':
        rig.monitor._ros_now_ns = lambda: 3_000_000_000
    else:
        values = rig.monitor.snapshot().feed.copy()
        values.update(controller_timer=2, ErrorStatus=1)
        rig.monitor.update_feed(values)
    with pytest.raises((FeedbackFailure, OperationCanceled)):
        rig.transport.wait_tray_position((0.,) * 6)
    assert rig.waits == 0


def test_wrong_tray_position_blocks_perception_and_all_motion():
    node = operation_node()
    node.hardware.current[0, 3] += .1
    with pytest.raises(FeedbackFailure, match='Not at Tray Detect position'):
        node.placement.run(node)
    node.trays.request.assert_not_called()
    node._execute_tray_position.assert_not_called()
    assert not node.hardware.calls


def test_position_lost_during_observation_cannot_start_placement():
    node = operation_node()

    def moved(*_args, **kwargs):
        node.hardware.current[0, 3] += .1
        kwargs['check_state']()
        pytest.fail('Position loss must stop detection')
    node.trays.request.side_effect = moved
    with pytest.raises(FeedbackFailure, match='Not at Tray Detect position'):
        node.placement.run(node)
    assert not node.hardware.calls


def test_place_transport_returns_at_third_acceptance_with_live_completion_context():
    rig = QueueRig()
    advance = rig.monitor.wait_next

    def drop_only(*args, **kwargs):
        assert len(rig.requests) == 2, 'Must not await retract arrival'
        return advance(*args, **kwargs)
    rig.monitor.wait_next = Mock(side_effect=drop_only)
    operation = rig.node.placement
    operation.run(rig.node)
    assert [name for name, _ in rig.requests] == ['MovL', 'MovLIO', 'MovLIO']
    assert operation.pending_motion is not None
    assert rig.transport.moving and rig.monitor._motion is not None
    assert operation.phase != 'DONE' and rig.node.holding_item
    assert rig.node.managed.session.attempts[0].state == 'HELD'
    assert rig.monitor.wait_next.call_count == 2  # Motion evidence then drop; no settling.
    operation.close_pending()
    assert not rig.transport.moving and rig.monitor._motion is None


def background_rig():
    rig = QueueRig()
    node = rig.node
    node.machine = ControllerStateMachine(initial='HOLDING')
    node.configuration.configuration_id = 'bound'
    node.cancel_event, node.shutdown_event = threading.Event(), threading.Event()
    node.stop_guard = threading.RLock()
    node.stop_attempt = None
    node.active_goal = None
    node.phase = node.waypoint = ''
    node.candidate_index = node.candidate_total = 0
    node.operation_lock.acquire()
    node.publish_status = Mock()
    node.cancel_requested = node.cancel_event.is_set

    def check_cancel():
        if node.cancel_event.is_set():
            raise OperationCanceled('Stop during queued placement')
    node.raise_if_cancelled = check_cancel
    node.wait_for_resume = node.managed.checkpoint
    node._perception_ready = lambda _: True
    for name in ('_begin_operation', '_end_operation', '_request_stop', '_confirm_shared_stop',
                 '_finish_stop_state', '_contain_queue_control_failure', '_finish_placement_queue'):
        setattr(node, name, getattr(RobotController, name).__get__(node))
    node.hardware.confirm_stop = Mock()  # Physical Stop transport is tested separately.
    node._action_failure = RobotController._action_failure.__get__(node)
    node._failure_outcome = RobotController._failure_outcome
    node.waiting, node.release = threading.Event(), threading.Event()

    def next_sample(*_args, **_kwargs):
        if node.placement.release_issued and not node.placement.neutral_issued:
            return rig.next_sample()
        node.waiting.set()
        assert node.release.wait(2.), 'Test failed to release the completion worker'
        values = next(rig.steps)
        return rig.emit(**values).revision
    rig.monitor.wait_next = next_sample
    rig.steps = iter(rig.script)
    goal = SimpleNamespace(
        request=PlaceItem.Goal(configuration_id='bound', x_mm=30., y_mm=40., rotation_deg=90.),
        succeed=Mock(), abort=Mock(), canceled=Mock(), is_cancel_requested=False)
    node.goal = goal
    return rig


def test_action_result_precedes_arrival_but_ownership_and_supervision_continue():
    rig = background_rig()
    node = rig.node
    result = RobotController._execute_place_action(node, node.goal)
    try:
        assert result.outcome == result.SUCCESS and result.final_state == 'PLACING'
        assert 'queue accepted' in result.message
        node.goal.succeed.assert_called_once()
        assert node.waiting.wait(1.)
        assert node.operation_lock.locked() and node.hardware.moving
        assert node.active_goal is None and node.active_action == 'place'
        assert node.placement.phase != 'DONE' and node.holding_item
        assert RobotController._reserve_goal(node, 'home', 'bound') == GoalResponse.REJECT
        RobotController._supervise(node)  # Must not classify this queue as unexpected idle motion.
        assert all(name != 'Stop' for name, _ in rig.requests)
    finally:
        node.release.set()
        node.placement_thread.join(2.)
    assert not node.placement_thread.is_alive()
    assert not node.operation_lock.locked() and not node.hardware.moving
    assert node.machine.state == 'READY' and node.placement is None
    assert node.managed.session.attempts[0].state == 'PLACED'
    assert all(name != 'Stop' for name, _ in rig.requests)


@pytest.mark.parametrize('failure', ['stop', 'fault', 'grip', 'stale'])
def test_failure_after_action_acceptance_is_contained_and_reported(failure):
    rig = background_rig()
    node = rig.node
    if failure == 'fault':
        rig.script[1]['ErrorStatus'] = 1
    elif failure == 'grip':
        rig.script[-1]['inputs'] = 1
    result = RobotController._execute_place_action(node, node.goal)
    assert result.outcome == result.SUCCESS
    assert node.waiting.wait(1.)
    if failure == 'stop':
        node._request_stop('Explicit Stop', fresh=True)
    elif failure == 'stale':
        node.monitor._ros_now_ns = lambda: 5_000_000_000
    node.release.set()
    node.placement_thread.join(2.)
    assert not node.placement_thread.is_alive()
    assert not node.operation_lock.locked() and not node.hardware.moving
    assert node.machine.state == 'RECOVERY_REQUIRED'
    assert 'Queued placement failed:' in node.machine.message
    assert any(name == 'Stop' for name, _ in rig.requests)
    assert node.managed.session.attempts[0].state == 'HELD'
    node.goal.succeed.assert_called_once()  # Acceptance cannot be retracted or reported twice.
    node.goal.abort.assert_not_called()


def test_queue_only_cannot_abandon_a_nonplacement_move():
    transport = object.__new__(DobotTransport)
    with pytest.raises(CommandRejected, match='supervised placement'):
        transport.move_batch((), queue_only=True)


def test_completion_worker_start_failure_stops_queue_and_releases_owner(monkeypatch):
    rig = background_rig()
    worker = Mock()
    worker.start.side_effect = RuntimeError('Cannot start completion worker')
    monkeypatch.setattr('robot_controller.controller.threading.Thread', lambda **_kw: worker)
    node = rig.node
    result = RobotController._execute_place_action(node, node.goal)
    assert result.outcome == result.SUCCESS  # Three replies were already accepted.
    assert node.machine.state == 'RECOVERY_REQUIRED'
    assert 'Cannot start completion worker' in node.machine.message
    assert not node.operation_lock.locked() and not node.hardware.moving
    assert node.monitor._motion is None and node.placement.pending_motion is None
    assert any(name == 'Stop' for name, _ in rig.requests)


def test_pause_after_action_result_retains_owner_and_never_replays_unconfirmed_release():
    rig = background_rig()
    node = rig.node
    node.hardware.confirm_stop.side_effect = lambda *_a, **_kw: rig.emit(
        outputs=(1 << 12) | 2, inputs=1, running=0)
    node.wait_control = lambda _seconds: node.managed.continue_operation()
    result = RobotController._execute_place_action(node, node.goal)
    assert result.outcome == result.SUCCESS and node.waiting.wait(1.)
    node.managed.request('pause')
    assert node.machine.state == 'PAUSING' and node.operation_lock.locked()
    node.release.set()
    node.placement_thread.join(2.)
    assert not node.placement_thread.is_alive()
    assert node.machine.state == 'RECOVERY_REQUIRED'
    assert 'release unconfirmed' in node.machine.message
    assert [name for name, _ in rig.requests if name != 'Stop'] == ['MovL', 'MovLIO', 'MovLIO']
    assert not node.operation_lock.locked()


def test_pause_before_release_admission_rechecks_position_and_observes_again():
    rig = background_rig()
    node = rig.node
    node.wait_control = lambda _seconds: node.managed.continue_operation()
    paused = []

    def pause_first(index):
        if index == 1 and not paused:
            paused.append(True)
            node.managed.request('pause')
    rig.on_request = pause_first
    result = RobotController._execute_place_action(node, node.goal)
    try:
        assert result.outcome == result.SUCCESS and paused
        assert node.waiting.wait(1.)
        assert node.trays.request.call_count == 2
    finally:
        node.release.set()
        node.placement_thread.join(2.)
    assert node.machine.state == 'READY'
    assert [name for name, _ in rig.requests if name != 'Stop'] == [
        'MovL', 'MovL', 'MovLIO', 'MovLIO']
