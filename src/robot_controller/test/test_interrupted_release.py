"""Explicit Recover abandons interrupted release; Stop still protects I/O."""

import pytest

from robot_controller.errors import OperationCanceled, StopUnconfirmed
from robot_controller.feedback import SUCTION_LOSS_DEBOUNCE_SEC
from robot_controller.hardware import DobotTransport
from robot_controller.kinematics import pose_matrix, pose_values
from robot_controller.item_return import ItemReturnOperation
from robot_controller.state_machine import ControllerStateMachine

from test_recovery_return import RecoveryRig
from test_transport_v2 import CompletedFuture, StopMonitor, snapshot


def return_rig():
    rig = RecoveryRig(count=2)
    rig.machine = ControllerStateMachine(initial="PICKING")
    rig._begin_operation("pick")
    rig.managed.executing = True

    return rig


def settle_stop(rig):
    if not rig.feed["digital_input_bits"] & 1:
        rig.set_di1(False, SUCTION_LOSS_DEBOUNCE_SEC)
    rig.managed._clear_request()
    rig._settle_lifecycle_cancellation("Stop during return")
    rig._end_operation()
    assert rig.machine.state == "RECOVERY_REQUIRED"
    assert rig.managed.session.held_index == 1


@pytest.mark.parametrize("boundary", ["before_release", 2, 14, 13, 1, "retract"])
def test_interrupted_release_recovery_preserves_io_to_home_then_neutralizes(boundary):
    rig = return_rig()

    def interrupt():
        rig.cancel_event.set()
        raise OperationCanceled("Stop during queued return")

    def target_reached(target, kwargs):
        if ((boundary == "before_release" and target.name == "return_pre")
                or (boundary == "retract" and target.name == "return_retract")):
            interrupt()

    def timed_output(event, target):
        if event.percent == 80 and event.channel == boundary:
            operation = rig.managed.return_progress
            rig.feed_sequence += 1
            rig.output_history.append((rig.feed_sequence, rig.clock,
                                       rig.feed["digital_outputs"], rig.feed["digital_input_bits"]))
            operation.observe(rig, rig.snapshot())
            interrupt()
    rig.hardware.on_target = target_reached
    rig.hardware.on_timed_output = timed_output
    with pytest.raises(OperationCanceled):
        rig.managed._put_back(dropped=False)
    assert rig.managed.return_progress is not None
    assert rig.managed.session.attempts[0].state == "HELD"
    settle_stop(rig)
    rig.hardware.on_target = lambda *_args: None
    rig.hardware.on_timed_output = lambda *_args: None
    offset = len(rig.log)
    result = rig.recover()
    if boundary in (13, 1):
        assert not result.success  # HIGH with vacuum OFF is not trusted holding.
        rig.lose_suction()
        result = rig.recover()
    if boundary in ("before_release", 2, 14):
        assert result.success and result.state == "PAUSED"
        assert rig.managed.can_return_item()
        assert rig.managed.session.attempts[0].state == "HELD"
        assert not any(row[0] in ("output", "timed_output") for row in rig.log[offset:])
        return
    assert result.success and result.state == "READY"
    assert rig.managed.session.attempts[0].state == (
        "CANCELED" if boundary == "retract" else "DROPPED")
    assert rig.managed.session.attempts[1].state == "CANCELED"
    assert rig.managed.return_progress is None
    assert rig.feed["digital_outputs"] == 0
    assert [row for row in rig.log[offset:] if row[0] == "output"] == [
        *([("output", 1, False)] if boundary == 1 else []),
        ("output", 13, True),
        *[("output", channel, False) for channel in (1, 2, 13, 14)],
    ]
    assert not any(row[0] == "timed_output" for row in rig.log[offset:])


def test_stop_after_release_recovers_upward_without_second_release():
    rig = return_rig()
    stopped = rig.configuration.home_matrix.copy()
    stopped[0, 3] = .04

    def stop_after_retract(target, kwargs):
        if target.name == "return_retract":
            rig.feed["tool_vector_actual"] = pose_values(stopped)
            rig.cancel_event.set()
            raise OperationCanceled("Stop before queued Home completes")
    rig.hardware.on_target = stop_after_retract
    with pytest.raises(OperationCanceled):
        rig.managed._put_back(dropped=False)
    assert rig.managed.return_progress.release_confirmed
    assert rig.managed.session.attempts[0].state == "HELD"
    settle_stop(rig)
    resumed = []
    rig.hardware.on_move = lambda targets, kwargs: resumed.append((targets, kwargs))
    result = rig.recover()
    assert result.success and result.state == "READY"
    assert rig.log.count(("timed_output", 80, 1, True)) == 1
    assert len(resumed) == 1
    targets, kwargs = resumed[0]
    assert kwargs["batch_name"] == "recovery_home" and kwargs["preserve_outputs"]
    assert targets[-1].joints_rad == rig.configuration.home_joints
    for target in targets[:-1]:
        assert target.matrix[2, 3] >= stopped[2, 3]
        assert target.matrix[0, 3] == stopped[0, 3]


def test_new_high_after_confirmed_release_blocks_recovery_without_releasing_again():
    rig = return_rig()
    rig.managed.return_progress = ItemReturnOperation(index=1, phase="RELEASED")
    rig.managed.session.set_state(1, "RETURNED")
    rig.holding_item = False
    rig.machine = ControllerStateMachine(initial="RECOVERY_REQUIRED")
    rig._end_operation()
    result = rig.recover()
    assert not result.success
    assert "DI1 HIGH after confirmed release" in result.message
    assert not any(x[0] == "pulse" for x in rig.log)
    assert rig.managed.return_progress is None
    assert rig.recovery_home.release_confirmed


@pytest.mark.parametrize("unrelated_change", [False, True])
def test_stop_reconciles_only_an_issued_unconfirmed_release_output(unrelated_change):
    rig = return_rig()
    rig.holding_item = False
    rig.managed.return_progress = ItemReturnOperation(
        index=1, phase="RELEASING", pending_outputs={13: False})
    transport = object.__new__(DobotTransport)
    transport.node = rig
    bits = 2  # Suction OFF command took effect; fingers still CLOSE.
    if unrelated_change:
        bits = 0  # Finger CLOSE also changed without an issued command.
    sample = snapshot(di1=True, outputs=bits)
    sample.feed["tool_vector_actual"] = [0.] * 6
    transport.monitor = StopMonitor(sample)
    if unrelated_change:
        with pytest.raises(StopUnconfirmed, match="DO2 changed"):
            transport.confirm_stop(CompletedFuture())
    else:
        transport.confirm_stop(CompletedFuture())
        assert rig.expected_outputs[13] is False
        assert rig.expected_outputs[2] is True


def test_exhaust_can_finish_after_an_interrupted_stop_confirmation():
    rig = return_rig()
    rig.holding_item = False
    rig.expected_outputs = {1: False, 2: False, 13: False, 14: True}
    rig.managed.return_progress = ItemReturnOperation(
        index=1, phase="RELEASING", pending_outputs={1: True})
    transport = object.__new__(DobotTransport)
    transport.node = rig
    for exhaust_on in (True, False):
        sample = snapshot(di1=False, outputs=(1 << 13) | int(exhaust_on))
        sample.feed["tool_vector_actual"] = [0.] * 6
        transport.monitor = StopMonitor(sample)
        transport.confirm_stop(CompletedFuture())
        assert rig.expected_outputs[1] is exhaust_on


def test_pause_skips_optional_rise_inside_existing_position_tolerance():
    rig = return_rig()
    current = rig.configuration.home_matrix.copy()
    current[2, 3] -= .001
    actual = rig.managed._rise(current, holding=True)
    assert actual[2, 3] == current[2, 3]
    assert not any(x[0] == "move" for x in rig.log)
    assert pose_matrix(pose_values(actual))[2, 3] == current[2, 3]


def test_recover_after_completed_release_cancels_next_candidate_instead_of_handover():
    rig = return_rig()
    rig.lose_suction()
    rig.managed._mark_drop()
    prefix, origin = rig.managed._put_back(dropped=True, continue_candidates=True)
    rig.managed.executing = False
    executed = []

    def stop_at_retract(target, kwargs):
        executed.append(target.name)
        if target.name == "return_retract":
            rig.cancel_event.set()
            raise OperationCanceled("Stop before next queued candidate")
    rig.hardware.on_target = stop_at_retract
    with pytest.raises(OperationCanceled):
        rig.managed.run_pick(
            [attempt.plan for attempt in rig.managed.session.attempts],
            check=lambda _index: None, departure=prefix, departure_pose=origin)
    assert rig.managed.return_progress.release_confirmed
    settle_stop(rig)
    result = rig.recover()
    assert result.success and result.state == "READY"
    assert rig.managed.session.held_index is None
    assert rig.managed.session.attempts[1].state == "CANCELED"
    assert rig.managed.return_progress is None
    assert rig.log.count(("timed_output", 80, 1, True)) == 1
    assert "p2_pick" not in executed
