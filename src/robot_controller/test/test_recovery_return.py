"""Recover preserves grip through travel, then resets the gripper only at Home."""

from types import SimpleNamespace
import threading
import time
from unittest.mock import patch

import numpy as np
import pytest

from robot_controller.controller import RobotController
from robot_controller.errors import CommandRejected, FeedbackFailure, OperationCanceled
from robot_controller.hardware import DobotTransport
from robot_controller.kinematics import pose_values
from robot_controller.state_machine import ControllerStateMachine
from test_managed_control import Rig, states


class RecoveryRig(Rig):
    def __init__(self, *, count=2, prepick=80., held=True):
        super().__init__(held=True, count=count, prepick=prepick)
        if not held:
            self.lose_suction()
        self.machine = ControllerStateMachine(initial="FAULT")
        self.operation_lock.release()
        self.startup_complete = False
        self.global_speed_percent = 60
        self.global_cp_percent = 100
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
        self.managed.start_idle_worker = lambda: self.log.append(("recovery_worker",))
        self.monitor.wait = self.wait_test

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

    def snapshot(self, **kwargs):
        sample = super().snapshot(**kwargs)
        sample.robot_enabled = True
        sample.joints = ((0.,) * 6 if np.allclose(
            self.hardware.current_pose(), self.configuration.home_matrix) else (1.,) * 6)
        return sample

    def wait_test(self, predicate, _timeout, **kwargs):
        self.raise_if_cancelled()
        sample = self.snapshot()
        if predicate(sample):
            return sample
        self.set_di1(bool(self.feed["digital_input_bits"] & 1), 1.01)
        with patch("robot_controller.recovery.time", SimpleNamespace(
                monotonic=lambda: time.monotonic() + 1.01)):
            sample = self.snapshot()
            assert predicate(sample)
            return sample

    def recover(self):
        return RobotController._recover(self, None, SimpleNamespace())


@pytest.mark.parametrize("held", [False, True])
@pytest.mark.parametrize("height", [.3, .8, 1.])
def test_recovery_preserves_grip_during_travel_then_relaxes_at_home(held, height):
    rig = RecoveryRig(count=3, held=held)
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
    if held:
        assert result.success and result.state == "PAUSED"
        assert "Return Item" in result.message
        assert rig.feed["digital_outputs"] == outputs and rig.holding_item
        assert states(rig) == ["HELD", "CANCELED", "CANCELED"]
        assert rig.managed.can_return_item() and rig.operation_lock.locked()
        assert ("recovery_worker",) in rig.log
        assert not any(row[0] == "output" for row in rig.log)
        return
    assert result.success and result.state == "READY"
    assert "at Home" in result.message and "cancelled" in result.message
    assert rig.feed["digital_outputs"] == 0 and not rig.holding_item
    assert states(rig) == ["DROPPED", "CANCELED", "CANCELED"]
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
    rig = RecoveryRig(held=False)
    rig.feed["tool_vector_actual"] = pose_values(rig.configuration.home_matrix)
    assert rig.recover().success
    assert not any(row[0] in ("move", "pulse") for row in rig.log)
    assert rig.feed["digital_outputs"] == 0


def test_reset_refuses_to_run_before_confirmed_home():
    from robot_controller.recovery import HomeRecovery

    rig = RecoveryRig(held=False)
    recovery = HomeRecovery.cancel_action(rig)
    recovery.capture(rig, rig.snapshot())
    with pytest.raises(FeedbackFailure, match="stationary Home"):
        recovery.relax(rig)
    assert not any(row[0] == "output" for row in rig.log)


def test_positive_suction_offers_shared_return_without_reset_or_resuming_old_batch():
    rig = RecoveryRig(held=True)
    assert rig.recover().state == "PAUSED"
    offset = len(rig.log)
    rig.managed.request("return")
    rig.managed._idle_worker()
    assert rig.machine.state == "READY" and not rig.holding_item
    assert rig.recovery_home is None and not rig.operation_lock.locked()
    assert states(rig) == ["RETURNED", "CANCELED"]
    assert [row[2]["batch_name"] for row in rig.log[offset:] if row[0] == "move"] == [
        "return_item_queued_home"]
    assert rig.feed["digital_outputs"] == 0


@pytest.mark.parametrize("failure", ["cancel", "output_failure", "unexpected_output"])
def test_reset_failure_or_stop_prevents_later_outputs_and_success(failure):
    rig = RecoveryRig(held=False)
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
    rig = RecoveryRig(held=False)

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
    rig = RecoveryRig(held=False)
    visited = []

    def fail(_targets, policy):
        visited.append(policy["batch_name"])
        raise FeedbackFailure("Lift not confirmed")
    rig.hardware.on_move = fail
    result = rig.recover()
    assert not result.success and visited == ["recovery_lift"]
    assert "Lift not confirmed" in result.message and "stopped" in result.message


def test_source_change_blocks_recovery_before_robot_commands():
    rig = RecoveryRig(held=False)

    def changed(_root):
        raise FeedbackFailure("Source changed")
    rig.configuration.validate_sources = changed
    assert not rig.recover().success
    assert not any(row[0] in ("recover", "move", "output", "pulse") for row in rig.log)


def test_dropped_item_that_reappears_is_not_trusted_again():
    rig = RecoveryRig(held=True)
    rig.lose_suction()
    rig.managed.observe(rig.snapshot())
    rig.set_di1(True)
    result = rig.recover()
    assert not result.success and "DI1 suction is HIGH" in result.message
    assert not any(row[0] in ("move", "output", "pulse") for row in rig.log)


def test_unknown_high_blocks_until_fresh_low_then_returns_home_and_relaxes():
    rig = RecoveryRig(held=True)
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
    rig = RecoveryRig(held=False)

    def stale(**_kwargs):
        raise FeedbackFailure("Canonical feedback is stale")
    rig.monitor.snapshot = stale
    result = rig.recover()
    assert not result.success and "stale" in result.message
    assert not any(row[0] in ("move", "output", "pulse") for row in rig.log)


def test_real_lifecycle_preserves_unheld_outputs_through_clear_enable_and_settings():
    rig = RecoveryRig(held=False)
    rig.lose_suction()
    transport = object.__new__(DobotTransport)
    transport.node, transport.monitor = rig, rig.monitor
    transport.clients = {}
    transport.wait_services = lambda **_kwargs: None
    transport.ensure_no_pending_response = lambda: None
    transport.request_stop = rig.hardware.request_stop
    transport._acknowledge_stop = lambda *_args, **_kw: rig.log.append(("stop_accepted",))
    transport.confirm_stop = lambda _future, **kw: rig.log.append(("stationary_after_enable",))
    transport._clear_errors_if_needed = lambda: rig.log.append(("clear",))
    transport._call_startup = lambda name: rig.log.append((name,))
    transport._wait_enabled = lambda: None
    transport._apply_settings = lambda speed, cp: rig.log.append(("settings", speed, cp))
    transport._reset_outputs_if_unheld = lambda: pytest.fail("Recovery reset outputs")
    transport._confirm_ready = lambda: None
    rig.check_all_command_owners = rig.check_feedback_owners = lambda *_args: None
    rig.hardware.recover = transport.recover
    assert rig.recover().success
    assert ("EnableRobot",) in rig.log and ("settings", 60, 100) in rig.log
    assert rig.log.index(("stop_accepted",)) < rig.log.index(("clear",))
    assert rig.log.index(("EnableRobot",)) < rig.log.index(("stationary_after_enable",))
    assert rig.log.index(("stationary_after_enable",)) < rig.log.index(("settings", 60, 100))
    assert rig.feed["digital_outputs"] == 0
    first_reset = next(i for i, row in enumerate(rig.log) if row[0] == "output")
    last_move = max(i for i, row in enumerate(rig.log) if row[0] == "move")
    assert first_reset > last_move


def arm_empty_test(rig, *, detect, source=True):
    """Begin with vacuum OFF and fingers unchanged; DI1 responds only to SUCK."""
    rig.feed["digital_outputs"] = 1 << 1
    if not source:
        rig.managed.session = None
    output = rig.hardware.output

    def suction(channel, active, **kwargs):
        output(channel, active, **kwargs)
        if channel == 13 and active and detect:
            rig.set_di1(True)
    rig.hardware.output = suction


def test_clear_active_test_turns_vacuum_on_at_home_then_resets_without_finger_test():
    rig = RecoveryRig(held=False)
    arm_empty_test(rig, detect=False)
    assert rig.recover().state == "READY"
    outputs = [row for row in rig.log if row[0] == "output"]
    assert outputs == [("output", 13, True)] + [
        ("output", channel, False) for channel in (1, 2, 13, 14)]
    last_move = max(i for i, row in enumerate(rig.log) if row[0] == "move")
    assert next(i for i, row in enumerate(rig.log) if row[0] == "output") > last_move
    assert not rig.operation_lock.locked() and rig.feed["digital_outputs"] == 0


@pytest.mark.parametrize("source", [False, True])
def test_positive_probe_preserves_outputs_and_only_offers_an_existing_source(source):
    rig = RecoveryRig(held=False)
    arm_empty_test(rig, detect=True, source=source)
    result = rig.recover()
    assert result.success and result.state == "PAUSED"
    assert rig.feed["digital_outputs"] == (1 << 1) | (1 << 12)
    assert [row for row in rig.log if row[0] == "output"] == [("output", 13, True)]
    assert rig.managed.can_return_item() is source
    assert "Suction remains HIGH" in rig.managed.continue_block_reason(rig.snapshot())
    with pytest.raises(CommandRejected, match="Suction remains HIGH"):
        rig.managed.continue_operation()
    if source:
        assert states(rig) == ["DROPPED", "CANCELED"]  # Never promote probe HIGH to HELD.
        rig.managed.request("return")
    else:
        assert "No saved return position" in result.message
        with pytest.raises(CommandRejected, match="source pose"):
            rig.managed.request("return")
        rig.lose_suction()
        rig.managed.continue_operation()
    rig.managed._idle_worker()
    assert rig.machine.state == "READY" and rig.recovery_home is None
    assert rig.feed["digital_outputs"] == 0 and not rig.operation_lock.locked()


def test_cleared_held_probe_retests_without_resuming_cancelled_candidates():
    rig = RecoveryRig(held=True, count=3)
    assert rig.recover().state == "PAUSED"
    offset = len(rig.log)
    rig.lose_suction()
    rig.managed.continue_operation()
    rig.managed._idle_worker()
    assert rig.machine.state == "READY" and rig.recovery_home is None
    assert states(rig) == ["DROPPED", "CANCELED", "CANCELED"]
    assert not any(row[0] == "move" for row in rig.log[offset:])
    assert rig.feed["digital_outputs"] == 0


@pytest.mark.parametrize("failure", ["stop", "stale", "moved", "output", "source"])
def test_waiting_for_return_keeps_stop_feedback_home_outputs_and_source_guards(failure):
    rig = RecoveryRig(held=True)
    assert rig.recover().state == "PAUSED"
    offset = len(rig.log)
    if failure == "stop":
        rig.cancel_event.set()
    elif failure == "stale":
        rig.monitor.snapshot = lambda **_: (_ for _ in ()).throw(FeedbackFailure("stale"))
    elif failure == "moved":
        rig.feed["tool_vector_actual"][0] += 10.
    elif failure == "output":
        rig.feed["digital_outputs"] |= 1 << 13
    else:
        rig.configuration.validate_sources = lambda _: (_ for _ in ()).throw(
            FeedbackFailure("Source changed"))
        rig.managed.request("return")
    rig.managed._idle_worker()
    assert rig.machine.state in ("RECOVERY_REQUIRED", "FAULT")
    assert not rig.operation_lock.locked()
    assert not any(row[0] in ("move", "output") for row in rig.log[offset:])
    assert any(row[0] == "stop" for row in rig.log[offset:])


@pytest.mark.parametrize("failure", ["stale", "moved", "running", "output", "stop"])
def test_test_failure_at_already_home_is_contained_without_reset(failure):
    rig = RecoveryRig(held=False)
    rig.feed["tool_vector_actual"] = pose_values(rig.configuration.home_matrix)
    arm_empty_test(rig, detect=False)

    def bad_sample(predicate, *_args, **_kwargs):
        if failure == "stale":
            raise FeedbackFailure("Canonical feedback stale during test")
        if failure == "stop":
            rig.cancel_event.set()
            rig.raise_if_cancelled()
        if failure == "moved":
            rig.feed["tool_vector_actual"][0] += 10.
        elif failure == "running":
            rig.feed["RunningStatus"] = 1
        else:
            rig.feed["digital_outputs"] ^= 1 << 1
        predicate(rig.snapshot())
    rig.monitor.wait = bad_sample
    result = rig.recover()
    assert not result.success and result.state != "READY"
    assert [row for row in rig.log if row[0] == "output"] == [("output", 13, True)]
    assert any(row[0] == "stop" for row in rig.log)
    assert not rig.operation_lock.locked()


def test_positive_test_never_restores_a_confirmed_released_source():
    rig = RecoveryRig(held=False)
    rig.placement = SimpleNamespace(release_confirmed=True, observing=True)
    arm_empty_test(rig, detect=True)
    assert rig.recover().state == "PAUSED"
    assert not rig.managed.can_return_item()
    assert rig.managed.session.held_index is None


def test_real_worker_handoff_returns_service_reply_and_retains_single_operation_owner():
    from robot_controller.managed_control import ManagedControl

    rig = RecoveryRig(held=True)
    wake = threading.Event()
    rig.on_wait = lambda: wake.wait(.01)
    rig.managed.start_idle_worker = lambda: ManagedControl.start_idle_worker(rig.managed)
    try:
        result = rig.recover()
        assert result.success and result.state == "PAUSED"
        assert rig.operation_lock.locked() and rig.active_action == "recovery_pause"
        rig.managed.request("return")
        wake.set()
        rig.managed.thread.join(2)
        assert not rig.managed.thread.is_alive()
        assert rig.machine.state == "READY" and not rig.operation_lock.locked()
        assert states(rig) == ["RETURNED", "CANCELED"]
    finally:
        rig.cancel_event.set()
        wake.set()
        if rig.managed.thread is not None:
            rig.managed.thread.join(2)


def test_brief_high_between_test_wakeups_still_requires_operator_choice():
    rig = RecoveryRig(held=False)
    arm_empty_test(rig, detect=False)

    def transient(predicate, *_args, **_kwargs):
        rig.set_di1(True)
        rig.managed.observe_continuous(rig.snapshot())
        rig.lose_suction()
        rig.managed.observe_continuous(rig.snapshot())
        sample = rig.snapshot()
        assert not sample.feed["digital_input_bits"] & 1
        assert predicate(sample)
        return sample
    rig.monitor.wait = transient
    assert rig.recover().state == "PAUSED"
    assert rig.managed.can_return_item()
    assert rig.feed["digital_outputs"] & (1 << 12)


def test_clear_test_requires_full_second_and_advancing_feedback():
    rig = RecoveryRig(held=False)

    def time_and_sequence(predicate, *_args, **_kwargs):
        now = time.monotonic()
        sample = rig.snapshot()
        assert not predicate(sample)
        with patch("robot_controller.recovery.time", SimpleNamespace(monotonic=lambda: now + 2)):
            assert not predicate(sample)  # Time alone cannot validate a frozen feed.
            rig.set_di1(False)
            sample = rig.snapshot()
        with patch("robot_controller.recovery.time", SimpleNamespace(monotonic=lambda: now + .1)):
            assert not predicate(sample)  # A new sample alone cannot shorten the test.
        with patch("robot_controller.recovery.time", SimpleNamespace(monotonic=lambda: now + 2)):
            assert predicate(sample)
        return sample
    rig.monitor.wait = time_and_sequence
    assert rig.recover().state == "READY"


@pytest.mark.parametrize("channel", [1, 13])
def test_rejected_test_output_stops_before_later_output_commands(channel):
    rig = RecoveryRig(held=False)
    rig.feed["tool_vector_actual"] = pose_values(rig.configuration.home_matrix)
    rig.feed["digital_outputs"] = 1  # Exhaust ON, vacuum OFF.
    output = rig.hardware.output

    def rejected(index, active, **kwargs):
        if index == channel:
            raise CommandRejected(f"DO{index} rejected")
        output(index, active, **kwargs)
    rig.hardware.output = rejected
    assert not rig.recover().success
    assert [row for row in rig.log if row[0] == "output"] == (
        [] if channel == 1 else [("output", 1, False)])
    assert any(row[0] == "stop" for row in rig.log)


def test_source_changed_during_test_blocks_reset_and_ready():
    rig = RecoveryRig(held=False)
    wait = rig.monitor.wait

    def changed(*args, **kwargs):
        sample = wait(*args, **kwargs)
        rig.configuration.validate_sources = lambda _: (_ for _ in ()).throw(
            FeedbackFailure("Source changed during suction test"))
        return sample
    rig.monitor.wait = changed
    result = rig.recover()
    assert not result.success and result.state != "READY"
    assert not any(row[0] == "output" for row in rig.log)


def test_stop_between_positive_test_and_worker_handoff_clears_pause_owner(monkeypatch):
    from robot_controller.recovery import HomeRecovery

    rig = RecoveryRig(held=True)
    park = HomeRecovery.park

    def stop_after_park(recovery, node):
        park(recovery, node)
        stopped = RobotController._stop(node, None, SimpleNamespace())
        assert stopped.success
    monkeypatch.setattr(HomeRecovery, "park", stop_after_park)
    rig.recover()
    assert rig.machine.state == "RECOVERY_REQUIRED"
    assert not rig.operation_lock.locked()
    assert rig.managed.kind is None and not rig.managed.executing
    assert ("recovery_worker",) not in rig.log
    rig.lose_suction()
    assert rig.recover().state == "READY"
    assert rig.managed.kind is None and not rig.managed.executing
