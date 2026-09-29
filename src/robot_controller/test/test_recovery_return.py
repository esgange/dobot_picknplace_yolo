"""Recover preserves grip through travel, then resets the gripper only at Home."""

from types import SimpleNamespace
import threading

import numpy as np
import pytest

from robot_controller.controller import RobotController
from robot_controller.errors import FeedbackFailure, OperationCanceled
from robot_controller.hardware import DobotTransport
from robot_controller.kinematics import pose_values
from robot_controller.state_machine import ControllerStateMachine
from test_managed_control import Rig, states


class RecoveryRig(Rig):
    def __init__(self, *, count=2, prepick=80.):
        super().__init__(held=True, count=count, prepick=prepick)
        self.machine = ControllerStateMachine(initial="FAULT")
        self.operation_lock.release()
        self.startup_complete = False
        self.global_speed_percent = 60
        self.stop_guard = threading.RLock()
        self.stop_attempt = None
        self.active_goal = None
        self.publish_status = lambda: None
        self.cancel_requested = self.cancel_event.is_set
        self._begin_operation = lambda name: RobotController._begin_operation(self, name)
        self._end_operation = lambda: RobotController._end_operation(self)
        self._request_stop = lambda reason, **kwargs: RobotController._request_stop(self, reason, **kwargs)
        self._confirm_shared_stop = lambda future: RobotController._confirm_shared_stop(self, future)
        self._finish_stop_state = lambda attempt: RobotController._finish_stop_state(self, attempt)
        self._settle_lifecycle_cancellation = lambda message: RobotController._settle_lifecycle_cancellation(self, message)
        self._contain_queue_control_failure = lambda operation, error: RobotController._contain_queue_control_failure(self, operation, error)
        self._candidate_progress = lambda *_args: None
        self.configuration.home_joints = (0.,) * 6
        self.feed["tool_vector_actual"][0] = 100.
        self.hardware.home_already_reached = lambda _joints: np.allclose(
            self.hardware.current_pose(), self.configuration.home_matrix)

        def recover(speed, *, home_recovery):
            self.log.append(("recover", speed))
            home_recovery.capture(self, self.snapshot())
        self.hardware.recover = recover
        self.clear_suction_on_reset = True
        output = self.hardware.output

        def reset_output(channel, active, **kwargs):
            output(channel, active, **kwargs)
            if (self.active_action == "recover" and channel == 13 and not active
                    and self.clear_suction_on_reset):
                self.lose_suction()
        self.hardware.output = reset_output

    def recover(self):
        return RobotController._recover(self, None, SimpleNamespace())


@pytest.mark.parametrize("held", [False, True])
@pytest.mark.parametrize("height", [.3, .8, 1.])
def test_recovery_preserves_grip_during_travel_then_relaxes_at_home(held, height):
    rig = RecoveryRig(count=3)
    if not held:
        rig.lose_suction()
    current = rig.hardware.current_pose()
    current[:3, 3] = [.13, .17, height]
    rig.feed["tool_vector_actual"] = pose_values(current)
    outputs = rig.feed["digital_outputs"]
    batches = []

    def travelling(targets, kwargs):
        assert rig.feed["digital_outputs"] == outputs
        assert not any(row[0] == "output" for row in rig.log)
        batches.append((targets, kwargs))
    rig.hardware.on_move = travelling
    result = rig.recover()
    assert result.success and result.state == "READY"
    assert "at Home" in result.message and "cancelled" in result.message
    assert rig.feed["digital_outputs"] == 0 and not rig.holding_item
    assert states(rig) == ["CANCELED"] * 3
    assert rig.managed.session.next_eligible is None and rig.recovery_home is None
    assert [row for row in rig.log if row[0] == "output"] == [
        ("output", channel, False) for channel in (1, 2, 13, 14)]
    assert not any(row[0] == "pulse" for row in rig.log)
    assert [kwargs["batch_name"] for _, kwargs in batches] == (
        ["recovery_lift", "recovery_home"] if height < .8 else ["recovery_home"])
    if height < .8:
        rise = batches[0][0][0]
        assert rise.relative_z
        assert rise.matrix[:2, 3] == pytest.approx(current[:2, 3])
        assert rise.matrix[:3, :3] == pytest.approx(current[:3, :3])
        assert rise.matrix[2, 3] == .8
    assert batches[-1][0][0].joints_rad == rig.configuration.home_joints
    for targets, policy in batches:
        assert policy["preserve_outputs"]
        assert policy["require_suction"] is held and policy["forbid_suction"] is not held
        assert all(not target.motion_io for target in targets)
    assert not rig.operation_lock.locked()


def test_already_home_recovery_relaxes_without_motion():
    rig = RecoveryRig()
    rig.feed["tool_vector_actual"] = pose_values(rig.configuration.home_matrix)
    assert rig.recover().success
    assert not any(row[0] in ("move", "pulse") for row in rig.log)
    assert rig.feed["digital_outputs"] == 0


def test_reset_refuses_to_run_before_confirmed_home():
    from robot_controller.recovery import HomeRecovery

    rig = RecoveryRig()
    recovery = HomeRecovery.cancel_action(rig)
    recovery.capture(rig, rig.snapshot())
    with pytest.raises(FeedbackFailure, match="stationary Home"):
        recovery.relax(rig)
    assert not any(row[0] == "output" for row in rig.log)


def test_stuck_suction_after_reset_keeps_ready_blocked_and_retry_waits_for_low():
    rig = RecoveryRig()
    rig.clear_suction_on_reset = False
    result = rig.recover()
    assert not result.success and result.state == "HELD_UNKNOWN"
    assert "outputs relaxed at Home" in result.message and "still HIGH" in result.message
    assert rig.feed["digital_outputs"] == 0 and not rig.startup_complete
    assert states(rig) == ["CANCELED", "CANCELED"]
    offset = len(rig.log)
    assert not rig.recover().success
    assert not any(row[0] in ("move", "output", "pulse") for row in rig.log[offset:])
    rig.lose_suction()
    assert rig.recover().success and rig.machine.state == "READY"


@pytest.mark.parametrize("failure", ["cancel", "output_failure", "unexpected_output"])
def test_reset_failure_or_stop_prevents_later_outputs_and_success(failure):
    rig = RecoveryRig()
    output = rig.hardware.output

    def interrupted(channel, active, **kwargs):
        if channel == 2 and failure == "output_failure":
            raise FeedbackFailure("DO2 did not confirm OFF")
        output(channel, active, **kwargs)
        if channel == 1:
            if failure == "cancel":
                rig.cancel_event.set()
            elif failure == "unexpected_output":
                rig.feed["digital_outputs"] |= 1 << 13  # Unrequested finger OPEN.
    rig.hardware.output = interrupted
    result = rig.recover()
    assert not result.success and result.state != "READY"
    assert not rig.startup_complete
    assert [row for row in rig.log if row[0] == "output"] == [("output", 1, False)]
    assert not any(row[0] == "pulse" for row in rig.log)


@pytest.mark.parametrize("phase", ["recovery_lift", "recovery_home"])
def test_stop_during_recovery_prevents_later_motion_and_retry_only_homes(phase):
    rig = RecoveryRig()

    def stop(_targets, policy):
        if policy["batch_name"] == phase:
            rig.cancel_event.set()
            raise OperationCanceled("explicit Stop")
    rig.hardware.on_move = stop
    result = rig.recover()
    assert not result.success and result.state == "RECOVERY_REQUIRED"
    assert not rig.operation_lock.locked()
    assert not any(row[0] in ("output", "pulse") for row in rig.log)
    assert rig.managed.session.next_eligible is None
    rig.hardware.on_move = lambda *_args: None
    assert rig.recover().success
    assert rig.machine.state == "READY" and rig.feed["digital_outputs"] == 0


def test_failed_lift_does_not_dispatch_home():
    rig = RecoveryRig()
    visited = []

    def fail(_targets, policy):
        visited.append(policy["batch_name"])
        raise FeedbackFailure("Lift not confirmed")
    rig.hardware.on_move = fail
    result = rig.recover()
    assert not result.success and visited == ["recovery_lift"]
    assert "Lift not confirmed" in result.message and "stopped" in result.message


def test_source_change_blocks_recovery_before_robot_commands():
    rig = RecoveryRig()

    def changed(_root):
        raise FeedbackFailure("Source changed")
    rig.configuration.validate_sources = changed
    assert not rig.recover().success
    assert not any(row[0] in ("recover", "move", "output", "pulse") for row in rig.log)


def test_dropped_item_that_reappears_is_not_trusted_again():
    rig = RecoveryRig()
    rig.lose_suction()
    rig.managed.observe(rig.snapshot())
    rig.set_di1(True)
    result = rig.recover()
    assert not result.success and "DI1 suction is HIGH" in result.message
    assert not any(row[0] in ("move", "output", "pulse") for row in rig.log)


def test_unknown_high_blocks_until_fresh_low_then_returns_home_and_relaxes():
    rig = RecoveryRig()
    rig.managed.session = None
    rig.holding_item = False
    rig.machine = ControllerStateMachine(initial="HELD_UNKNOWN")
    result = rig.recover()
    assert not result.success and result.state == "HELD_UNKNOWN"
    assert "Recover again" in result.message
    assert not any(row[0] == "move" for row in rig.log)
    rig.lose_suction()
    assert rig.recover().success and rig.machine.state == "READY"
    assert rig.feed["digital_outputs"] == 0


def test_recovery_rejects_stale_feedback_without_moving():
    rig = RecoveryRig()

    def stale(**_kwargs):
        raise FeedbackFailure("Canonical feedback is stale")
    rig.monitor.snapshot = stale
    result = rig.recover()
    assert not result.success and "stale" in result.message
    assert not any(row[0] in ("move", "output", "pulse") for row in rig.log)


def test_real_lifecycle_preserves_unheld_outputs_through_clear_enable_and_settings():
    rig = RecoveryRig()
    rig.lose_suction()
    transport = object.__new__(DobotTransport)
    transport.node, transport.monitor = rig, rig.monitor
    transport.clients = {}
    transport.wait_services = lambda **_kwargs: None
    transport.ensure_no_pending_response = lambda: None
    transport.request_stop = rig.hardware.request_stop
    transport.confirm_stop = lambda _future, **kw: kw["recovery_home"].capture(rig, rig.snapshot())
    transport._clear_errors_if_needed = lambda: rig.log.append(("clear",))
    transport._call_startup = lambda name: rig.log.append((name,))
    transport._wait_enabled = lambda: None
    transport._apply_settings = lambda speed: rig.log.append(("settings", speed))
    transport._reset_outputs_if_unheld = lambda: pytest.fail("Recovery reset outputs")
    transport._confirm_ready = lambda: None
    rig.check_all_command_owners = rig.check_feedback_owners = lambda *_args: None
    rig.hardware.recover = transport.recover
    assert rig.recover().success
    assert ("EnableRobot",) in rig.log and ("settings", 60) in rig.log
    assert rig.feed["digital_outputs"] == 0
    first_reset = next(i for i, row in enumerate(rig.log) if row[0] == "output")
    last_move = max(i for i, row in enumerate(rig.log) if row[0] == "move")
    assert first_reset > last_move
