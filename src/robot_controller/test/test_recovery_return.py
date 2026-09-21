"""Explicit uncertain-item recovery uses the saved source and remaining batch."""

from types import SimpleNamespace
import threading

import numpy as np
import pytest

from robot_controller.controller import RobotController
from robot_controller.errors import FeedbackFailure, ManagedInterruption, OperationCanceled
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
        self.stop_confirmation_guard = threading.Lock()
        self.stop_future = None
        self.stop_error = None
        self.stop_confirmed = False
        self.state_before_stop = None
        self.active_goal = None
        self.publish_status = lambda: None
        self.cancel_requested = self.cancel_event.is_set
        self._begin_operation = lambda name: RobotController._begin_operation(self, name)
        self._end_operation = lambda: RobotController._end_operation(self)
        self._request_stop = lambda reason: RobotController._request_stop(self, reason)
        self._confirm_shared_stop = lambda future: (
            RobotController._confirm_shared_stop(self, future))
        self._finish_stop_state = lambda: RobotController._finish_stop_state(self)
        self._settle_lifecycle_cancellation = lambda message: (
            RobotController._settle_lifecycle_cancellation(self, message))
        self._contain_queue_control_failure = lambda operation, error: (
            RobotController._contain_queue_control_failure(self, operation, error))
        self._candidate_progress = lambda *_args: None
        self.hardware.recover = lambda speed, **kwargs: self.log.append(("recover", speed, kwargs))

    def recover(self):
        return RobotController._recover(self, None, SimpleNamespace())


@pytest.mark.parametrize("di1_recovers", [False, True])
@pytest.mark.parametrize("count", [1, 2])
@pytest.mark.parametrize("prepick", [20., 80.])
def test_recovery_puts_back_then_next_candidate_or_home(count, di1_recovers, prepick):
    rig = RecoveryRig(count=count, prepick=prepick)
    rig.lose_suction()
    rig.managed.observe(rig.snapshot())
    assert states(rig)[0] == "DROPPED"
    assert not any(entry[0] in ("move", "output", "pulse") for entry in rig.log)
    if di1_recovers:
        rig.set_di1(True)
    rig.hardware.acquisitions = iter((True,))
    batches = []
    rig.hardware.on_move = lambda targets, kwargs: batches.append((targets, kwargs))
    result = rig.recover()

    assert result.success
    assert result.state == ("READY" if count == 1 else "HOLDING")
    assert ("recover", 60, {"return_item": True}) in rig.log
    assert states(rig) == (["DROPPED"] if count == 1 else ["DROPPED", "HELD"])
    pulse = next(i for i, entry in enumerate(rig.log) if entry[0] == "pulse")
    approach = next(i for i, entry in enumerate(rig.log)
                    if entry[0] == "move" and entry[2]["batch_name"] == "return_item_to_release")
    outputs = [entry[1:] for entry in rig.log[:pulse] if entry[0] == "output"]
    assert outputs == [(2, False), (13, False), (14, True), (1, False)]
    assert approach < pulse
    first_output = next(i for i, entry in enumerate(rig.log) if entry[0] == "output")
    assert first_output > approach
    release = next(targets[-1] for targets, kwargs in batches
                   if kwargs["batch_name"] == "return_item_to_release")
    assert release.matrix[2, 3] == pytest.approx(.3 + prepick / 1000)
    if count == 2:
        next_pick = next(i for i, entry in enumerate(rig.log)
                         if entry[0] == "move" and entry[2].get("stop_on_suction"))
        assert next_pick > pulse
        assert not any(entry[0] == "home" for entry in rig.log[:next_pick])
        group = rig.log[next_pick]
        assert group[1] == ("return_clearance", "return_park_transit",
                            "p2_transit", "p2_initial", "p2_prepick", "p2_pick")
        assert group[2]["pick_settling_sec"] == 0.1
        assert np.array_equal(group[2]["confirmed_start_pose"], release.matrix)
        targets = next(targets for targets, kwargs in batches if kwargs.get("stop_on_suction"))
        old_exit, next_entry = targets[1:3]
        assert old_exit.matrix[2, 3] == next_entry.matrix[2, 3] == .8
        assert np.array_equal(old_exit.matrix[:2, 3], targets[0].matrix[:2, 3])
        assert not np.array_equal(old_exit.matrix[:2, 3], next_entry.matrix[:2, 3])
        assert not old_exit.motion_io
    else:
        assert not any(entry[0] == "move" and entry[2].get("stop_on_suction")
                       for entry in rig.log)
        assert any(entry[0] == "home" for entry in rig.log[pulse + 1:])
        home = next(entry[1] for entry in rig.log if entry[0] == "home")
        assert home["preceding"][-1].name == "return_park_transit"
    assert rig.managed.session.held_index == (None if count == 1 else 2)
    assert not rig.operation_lock.locked()


def test_recovery_skips_failed_candidates_and_homes_if_remaining_attempt_misses():
    rig = RecoveryRig(count=3)
    rig.managed.session.set_state(2, "ACTIVE")
    rig.managed.session.set_state(2, "FAILED")
    rig.lose_suction()
    result = rig.recover()
    assert result.success and result.state == "READY"
    assert states(rig) == ["DROPPED", "FAILED", "FAILED"]
    approaches = [entry for entry in rig.log if entry[0] == "move"
                  and entry[2].get("stop_on_suction")]
    assert len(approaches) == 1
    assert approaches[0][1][-1] == "p3_pick"
    assert rig.log[-1] == ("state", "READY")


@pytest.mark.parametrize("stop_phase", [
    "return_item_to_release", "return_item_to_candidate_2_pick"])
@pytest.mark.parametrize("di1_recovers", [False, True])
def test_direct_stop_preempts_recovery_without_later_commands(stop_phase, di1_recovers):
    rig = RecoveryRig()
    rig.lose_suction()
    rig.managed.observe(rig.snapshot())
    if di1_recovers:
        rig.set_di1(True)

    def stop(_targets, kwargs):
        if kwargs["batch_name"] == stop_phase:
            rig.cancel_event.set()
            raise OperationCanceled("explicit Stop")
    rig.hardware.on_move = stop
    result = rig.recover()
    assert not result.success and result.state == "RECOVERY_REQUIRED"
    assert not rig.operation_lock.locked()
    if stop_phase == "return_item_to_release":
        assert not any(entry[0] in ("output", "pulse", "home") for entry in rig.log)
        assert rig.managed.session.held_index == 1
        rig.hardware.on_move = lambda *_args: None
        rig.hardware.acquisitions = iter((True,))
        resumed = rig.recover()
        assert resumed.success and resumed.state == "HOLDING"
        assert sum(entry[0] == "pulse" for entry in rig.log) == 1
    else:
        assert any(entry[0] == "pulse" for entry in rig.log)
        assert not any(entry[0] == "home" for entry in rig.log)


def test_release_failure_prevents_next_candidate_and_retains_return_source():
    rig = RecoveryRig()
    rig.lose_suction()

    def fail_release():
        raise FeedbackFailure("exhaust pulse not confirmed")
    rig.hardware.exhaust_pulse = fail_release
    result = rig.recover()
    assert not result.success
    assert rig.machine.state == "RECOVERY_REQUIRED"
    assert rig.managed.session.held_index == 1
    assert not any(entry[0] == "move" and entry[2].get("stop_on_suction")
                   for entry in rig.log)
    assert not any(entry[0] == "home" for entry in rig.log)


def test_pause_during_recovery_next_attempt_replans_without_repeating_release():
    rig = RecoveryRig()
    rig.lose_suction()
    rig.hardware.acquisitions = iter((True,))

    def pause(_targets, kwargs):
        if kwargs.get("stop_on_suction"):
            rig.hardware.on_move = lambda *_args: None
            rig.managed.request("pause")
            raise ManagedInterruption("Pause during next approach")
    rig.hardware.on_move = pause
    result = rig.recover()
    assert result.success and result.state == "HOLDING"
    assert sum(entry[0] == "pulse" for entry in rig.log) == 1
    assert ("state", "PAUSED") in rig.log
    assert states(rig) == ["DROPPED", "HELD"]


def test_source_change_blocks_recovery_before_robot_commands():
    rig = RecoveryRig()
    rig.lose_suction()

    def changed(_root):
        raise FeedbackFailure("Source changed")
    rig.configuration.validate_sources = changed
    assert not rig.recover().success
    assert not any(entry[0] in ("recover", "move", "output", "pulse") for entry in rig.log)


def test_no_loss_keeps_existing_held_recovery_and_does_not_put_back():
    rig = RecoveryRig()
    result = rig.recover()
    assert result.success and result.state == "HOLDING"
    assert ("recover", 60, {"return_item": False}) in rig.log
    assert not any(entry[0] in ("move", "output", "pulse") for entry in rig.log)
