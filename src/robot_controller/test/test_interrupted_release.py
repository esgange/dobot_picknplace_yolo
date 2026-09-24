"""Stop at release boundaries retains enough evidence for explicit Recovery."""

import pytest

from robot_controller.errors import OperationCanceled, StopUnconfirmed
from robot_controller.hardware import DobotTransport
from robot_controller.kinematics import pose_matrix, pose_values
from robot_controller.managed_control import ReturnProgress
from robot_controller.state_machine import ControllerStateMachine

from test_recovery_return import RecoveryRig
from test_transport_v2 import CompletedFuture, StopMonitor, snapshot


def return_rig():
    rig = RecoveryRig(count=2)
    rig.machine = ControllerStateMachine(initial="PICKING")
    rig._begin_operation("pick")
    rig.managed.executing = True

    def recover(speed, *, return_item):
        rig.log.append(("recover", speed, {"return_item": return_item}))
        # Exercise the real unknown/known item gates without any ROS clients.
        transport = object.__new__(DobotTransport)
        transport.node, transport.monitor = rig, rig.monitor
        transport.return_recovery = return_item
        transport._check_held_context(rig.holding_item, rig.expected_outputs)
    rig.hardware.recover = recover
    return rig


def settle_stop(rig):
    rig.managed._clear_request()
    rig._settle_lifecycle_cancellation("Stop during return")
    rig._end_operation()
    assert rig.machine.state == "RECOVERY_REQUIRED"
    assert rig.managed.session.held_index == 1


@pytest.mark.parametrize("boundary", ["before_release", 2, 13, 14, 1, "pulse"])
def test_interrupted_unconfirmed_release_is_recoverable_with_saved_source(boundary):
    rig = return_rig()
    output, pulse = rig.hardware.output, rig.hardware.exhaust_pulse

    def interrupted_output(channel, active):
        if boundary == "before_release":
            rig.cancel_event.set()
            raise OperationCanceled("before first release output")
        output(channel, active)
        if boundary == channel:
            rig.cancel_event.set()
            raise OperationCanceled("after release output")

    def interrupted_pulse():
        rig.cancel_event.set()
        raise OperationCanceled("during pulse")
    rig.hardware.output = interrupted_output
    if boundary == "pulse":
        rig.hardware.exhaust_pulse = interrupted_pulse
    with pytest.raises(OperationCanceled):
        rig.managed._put_back(dropped=False)
    assert not rig.holding_item
    assert rig.managed.return_progress.phase == "RELEASING"
    if boundary == "before_release":
        assert rig.managed.return_progress.pending_outputs == {}
    assert rig.managed.session.attempts[0].state == "HELD"
    settle_stop(rig)
    rig.hardware.output, rig.hardware.exhaust_pulse = output, pulse
    result = rig.recover()
    assert result.success and result.state == "READY"
    assert rig.managed.return_progress is None
    assert rig.managed.session.held_index is None
    assert rig.managed.session.attempts[0].state == "RETURNED"
    assert rig.managed.session.attempts[1].state == "PENDING"
    assert sum(x[0] == "pulse" for x in rig.log) == 1


def test_stop_after_release_resumes_upward_retreat_without_second_release():
    rig = return_rig()
    stopped = rig.configuration.home_matrix.copy()
    stopped[0, 3] = .04  # Stopped during Home, beyond the old item's clearance.

    def stop_home(_targets, kwargs):
        if kwargs.get("batch_name") == "return_item_to_home":
            rig.feed["tool_vector_actual"] = pose_values(stopped)
            rig.cancel_event.set()
            raise OperationCanceled("Stop during return Home")
    rig.hardware.on_move = stop_home
    with pytest.raises(OperationCanceled):
        rig.managed._put_back(dropped=False)
    assert rig.managed.return_progress.phase == "RELEASED"
    assert rig.managed.session.attempts[0].state == "RETURNED"
    settle_stop(rig)
    resumed = []
    rig.hardware.on_move = lambda targets, kwargs: resumed.append((targets, kwargs))
    result = rig.recover()
    assert result.success and result.state == "READY"
    assert sum(x[0] == "pulse" for x in rig.log) == 1
    assert len(resumed) == 1
    targets, kwargs = resumed[0]
    assert kwargs["batch_name"] == "return_item_to_home"
    assert targets[-2].name == "return_park_transit"
    for target in targets[:-1]:
        assert target.matrix[2, 3] >= stopped[2, 3]
        assert target.matrix[0, 3] == stopped[0, 3]


def test_new_high_after_confirmed_release_blocks_recovery_without_releasing_again():
    rig = return_rig()
    rig.managed.return_progress = ReturnProgress(1, False, False, phase="RELEASED")
    rig.managed.session.set_state(1, "RETURNED")
    rig.holding_item = False
    rig.machine = ControllerStateMachine(initial="RECOVERY_REQUIRED")
    rig._end_operation()
    result = rig.recover()
    assert not result.success
    assert any("DI1 HIGH after confirmed release" in str(entry) for entry in rig.log)
    assert not any(x[0] == "pulse" for x in rig.log)
    assert rig.managed.return_progress.phase == "RELEASED"


@pytest.mark.parametrize("unrelated_change", [False, True])
def test_stop_reconciles_only_an_issued_unconfirmed_release_output(unrelated_change):
    rig = return_rig()
    rig.holding_item = False
    rig.managed.return_progress = ReturnProgress(
        1, False, False, phase="RELEASING", pending_outputs={13: False})
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
    rig.managed.return_progress = ReturnProgress(
        1, False, False, phase="RELEASING", pending_outputs={1: True})
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


def test_stop_during_release_to_next_candidate_handover_retains_completed_release():
    rig = RecoveryRig(count=2)
    rig.lose_suction()

    def stop_before_next_entry(_targets, kwargs):
        if kwargs.get("stop_on_suction"):
            rig.cancel_event.set()
            raise OperationCanceled("Stop before next entry acceptance")
    rig.hardware.on_move = stop_before_next_entry
    assert not rig.recover().success
    assert rig.managed.return_progress.phase == "RELEASED"
    assert rig.managed.session.held_index == 1
    assert sum(x[0] == "pulse" for x in rig.log) == 1
    rig.hardware.on_move = lambda *_args: None
    rig.hardware.acquisitions = iter((True,))
    result = rig.recover()
    assert result.success and result.state == "HOLDING"
    assert rig.managed.session.held_index == 2
    assert rig.managed.return_progress is None
    assert sum(x[0] == "pulse" for x in rig.log) == 1
