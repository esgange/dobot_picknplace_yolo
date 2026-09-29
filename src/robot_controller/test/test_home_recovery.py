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
    node.kinematics = SimpleNamespace(forward=lambda _j: node.configuration.home_matrix)
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
    confirm_fresh_stop(rig, recovery, outputs=outputs, inputs=inputs)
    rig.transport.current_pose = lambda: pose_matrix(
        rig.monitor.snapshot(require_enabled=True).feed['tool_vector_actual'])
    completions = []

    def arrival(*_args, **_kwargs):
        if not completions:
            assert [name for name, _ in rig.requests] == ['RelMovLUser']
            node.configuration.tray.detect_matrix[2, 3] = .8
            rig.emit(outputs=outputs, inputs=inputs, running=0)
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
