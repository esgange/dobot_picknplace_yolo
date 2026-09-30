"""Fresh Stop feedback replaces expired placement history only on explicit Recover."""

from types import SimpleNamespace

import pytest

from robot_controller.errors import FeedbackFailure, HeldSuctionLost, HeldUnknown, OperationCanceled
from robot_controller.recovery import HomeRecovery
from robot_controller.kinematics import pose_matrix
from test_feedback_v2 import joint_message
from test_placement_queue import QueueRig, HELD, OPEN
from test_transport_v2 import CompletedFuture


def confirm_fresh_stop(rig, recovery, *, outputs=0, inputs=0):
    def wait(predicate, *_args, **_kwargs):
        first = rig.emit(outputs=outputs, inputs=inputs, running=0)
        assert not predicate(first)
        second = rig.emit(outputs=outputs, inputs=inputs, running=0)
        assert predicate(second)
        return second
    rig.monitor.wait = wait
    rig.transport.confirm_stop(CompletedFuture(), recovery_home=recovery)


def test_expired_placement_history_does_not_trap_explicit_recovery_or_claim_placed():
    rig = QueueRig()
    rig.script[1]['inputs'] = 0  # Reproduce original missing full-open evidence.
    advance = rig.next_sample

    def stop_after_neutral(*args, **kwargs):
        result = advance(*args, **kwargs)
        if not rig.monitor.snapshot().feed['digital_outputs']:
            raise OperationCanceled('Operator Stop before Home')
        return result

    rig.monitor.wait_next = stop_after_neutral
    with pytest.raises(OperationCanceled, match='Operator Stop'):
        rig.run()
    operation = rig.node.placement
    for _ in range(1002):
        rig.monitor.update_joints(joint_message())
        rig.monitor.update_status(SimpleNamespace(is_connected=True, is_enable=True))
        rig.emit(outputs=0, inputs=0, running=0)
    operation.observe(rig.node, rig.monitor.snapshot(require_enabled=False))
    assert operation.phase != 'DONE'  # Expired telemetry cannot prove arrival.
    sent = len(rig.requests)
    recovery = HomeRecovery.cancel_action(rig.node)
    confirm_fresh_stop(rig, recovery)
    assert rig.node.placement is None and not operation.observing
    assert not recovery.holding and not rig.node.holding_item
    assert rig.node.managed.session.attempts[0].state == 'CANCELED'
    assert not operation.release_confirmed  # Never invent successful release.
    assert len(rig.requests) == sent  # Capture itself never sends DO/motion.
    assert rig.node.expected_outputs == {1: False, 2: False, 13: False, 14: False}


@pytest.mark.parametrize('held,outputs,inputs', [
    (True, HELD, 1), (False, 0, 0), (False, (1 << 13) | 1, OPEN),
])
def test_stop_adopts_stable_current_outputs_without_neutralizing(held, outputs, inputs):
    rig = QueueRig()
    for _ in range(8):
        rig.emit(outputs=outputs, inputs=inputs, running=0)
    recovery = HomeRecovery.cancel_action(rig.node)
    confirm_fresh_stop(rig, recovery, outputs=outputs, inputs=inputs)
    assert recovery.holding is held
    assert sum(1 << (ch - 1) for ch, on in rig.node.expected_outputs.items() if on) == outputs
    assert not rig.requests


def test_stop_waits_for_both_stationary_pose_and_stable_current_outputs():
    rig = QueueRig()
    recovery = HomeRecovery.cancel_action(rig.node)

    def wait(predicate, *_args, **_kwargs):
        assert not predicate(rig.emit(outputs=HELD, inputs=1, running=0))
        assert not predicate(rig.emit(outputs=1 << 12, inputs=1, running=0))
        assert not predicate(rig.emit(outputs=1 << 12, inputs=1, running=1))
        assert not predicate(rig.emit(outputs=1 << 12, inputs=1, running=0))
        assert predicate(rig.emit(outputs=1 << 12, inputs=1, running=0))
        return rig.monitor.snapshot(require_enabled=True)
    rig.monitor.wait = wait
    rig.transport.confirm_stop(CompletedFuture(), recovery_home=recovery)
    assert recovery.outputs[13] and not recovery.outputs[2]


@pytest.mark.parametrize('outputs', [HELD | (1 << 13), HELD | 1])
def test_opposing_outputs_block_recovery_before_enable(outputs):
    rig = QueueRig()
    recovery = HomeRecovery.cancel_action(rig.node)
    with pytest.raises(FeedbackFailure, match='opposing'):
        confirm_fresh_stop(rig, recovery, outputs=outputs, inputs=1)
    assert not rig.requests


@pytest.mark.parametrize('change', ['outputs', 'lost_suction', 'unexpected_suction'])
def test_recovery_monitor_keeps_outputs_and_suction_guards(change):
    rig = QueueRig()
    held = change != 'unexpected_suction'
    for _ in range(8):
        rig.emit(outputs=HELD if held else 0, inputs=int(held), running=0)
    recovery = HomeRecovery.cancel_action(rig.node)
    confirm_fresh_stop(rig, recovery, outputs=HELD if held else 0, inputs=int(held))
    for _ in range(8):
        sample = rig.emit(outputs=(1 << 12) if change == 'outputs' else HELD if held else 0,
                          inputs=0 if change == 'lost_suction' else 1, running=0)
    error = {'outputs': FeedbackFailure, 'lost_suction': HeldSuctionLost,
             'unexpected_suction': HeldUnknown}[change]
    with pytest.raises(error):
        recovery.check(sample)


@pytest.mark.parametrize('holding', [False, True])
def test_recovery_encodes_vertical_then_joint_home_through_real_transport(holding):
    rig = QueueRig()
    node = rig.node
    node.active_action = 'recover'
    node.configuration.home_joints = (0.,) * 6
    node.configuration.tray.detect_joints = (.1,) * 6
    rig.joint_poses[node.configuration.home_joints] = node.configuration.home_matrix.copy()
    current = node.configuration.home_matrix.copy()
    current[0, 3], current[2, 3] = .1, .3
    node.configuration.tray.detect_matrix = current.copy()
    joints = joint_message()
    joints.position[0] = .2
    rig.monitor.update_joints(joints)
    outputs, inputs = (HELD, 1) if holding else (0, 0)
    for _ in range(8):
        rig.emit(outputs=outputs, inputs=inputs, running=0)
    recovery = node.recovery_home = HomeRecovery.cancel_action(node)
    real_wait = rig.monitor.wait
    confirm_fresh_stop(rig, recovery, outputs=outputs, inputs=inputs)
    rig.monitor.wait = real_wait
    rig.transport.current_pose = lambda: pose_matrix(
        rig.monitor.snapshot(require_enabled=True).feed['tool_vector_actual'])
    completions = []

    def arrival(*_args, **_kwargs):
        if not completions:
            assert [name for name, _ in rig.requests] == ['RelMovLUser']
            node.configuration.tray.detect_matrix[2, 3] = .8
            rig.emit(outputs=outputs, inputs=inputs, running=0, currentCommandId=3)
        else:
            assert [name for name, _ in rig.requests] == ['RelMovLUser', 'MovL']
            rig.monitor.update_joints(joint_message())
            rig.emit(outputs=outputs, inputs=inputs, home=True)
        completions.append(rig.monitor.sequence)
        return rig.monitor.sequence
    rig.monitor.wait_next = arrival
    recovery.run(node)
    assert len(completions) == 2
    rise, home = [request for _name, request in rig.requests]
    assert (rise.a, rise.b, rise.c, rise.d, rise.e, rise.f) == (0., 0., 500., 0., 0., 0.)
    assert home.mode and [home.a, home.b, home.c, home.d, home.e, home.f] == [0.] * 6
    assert all(list(request.param_value) == ['user=0', 'tool=0', 'v=80', 'a=70']
               for _name, request in rig.requests)
    assert node.monitor.snapshot().feed['digital_outputs'] == outputs

    def reset_echo(*_args, **_kwargs):
        name, request = rig.requests[-1]
        assert name == 'DO' and request.status == 0 and request.time == 0
        sample = rig.monitor.snapshot()
        bits = sample.feed['digital_outputs'] & ~(1 << (request.index - 1))
        inputs = 0 if request.index == 13 else sample.feed['digital_input_bits']
        rig.emit(outputs=bits, inputs=inputs, home=True)
        return rig.monitor.sequence

    rig.on_request = reset_echo
    rig.monitor.wait_next = reset_echo
    recovery.relax(node)
    assert [name for name, _ in rig.requests] == ['RelMovLUser', 'MovL'] + ['DO'] * 4
    assert [request.index for _, request in rig.requests[-4:]] == [1, 2, 13, 14]
    assert node.monitor.snapshot().feed['digital_outputs'] == 0
    assert not node.holding_item and node.managed.session.held_index is None


def test_reset_at_home_waits_out_its_own_io_queue_without_false_position_failure():
    rig = QueueRig()
    node = rig.node
    node.active_action = 'recover'
    node.configuration.home_joints = (0.,) * 6
    for _ in range(8):
        rig.emit(outputs=0, inputs=0, home=True)
    recovery = node.recovery_home = HomeRecovery.cancel_action(node)
    recovery.capture(node, rig.monitor.snapshot(require_enabled=True))
    rig.transport.home_recovery = recovery

    def echo(_index):
        request = rig.requests[-1][1]
        rig.emit(outputs=0, inputs=0, home=True,
                 isRunQueuedCmd=int(request.index == 14))

    rig.on_request = echo
    rig.steps = iter([dict(outputs=0, inputs=0, home=True)])
    recovery.relax(node)
    assert [(name, req.index, req.status) for name, req in rig.requests] == [
        ('DO', channel, 0) for channel in (1, 2, 13, 14)]
    assert next(rig.steps, None) is None
    assert not recovery.holding and not node.holding_item
