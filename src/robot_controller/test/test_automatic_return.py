"""Active Pick puts back a lost item and finishes its original action/batch."""

from types import SimpleNamespace

import numpy as np
import pytest

import robot_controller.controller as controller_module
from robot_controller.controller import RobotController
from robot_controller.errors import (CommandResponseTimeout, FeedbackFailure,
                                     OperationCanceled, StopUnconfirmed)
from robot_controller.hardware import DobotTransport
from robot_controller.kinematics import pose_values
from robot_controller.state_machine import ControllerStateMachine

from test_managed_control import states
from test_recovery_return import RecoveryRig


def action_rig(monkeypatch, *, count=2, losses=(1,), loss_at="retract", prepick=80.):
    rig = RecoveryRig(count=count, prepick=prepick)
    rig.machine = ControllerStateMachine(initial="READY")
    rig.startup_complete = True
    rig.holding_item = False
    rig.set_di1(False)
    rig.expected_outputs = {1: False, 2: False, 13: False, 14: False}
    rig.feed["digital_outputs"] = 0
    rig._begin_operation("pick")
    rig.wait_for_resume = rig.managed.checkpoint
    rig.observe_managed_feedback = rig.managed.observe
    rig._attempt_changed = lambda *_args: None
    rig._failure_outcome = RobotController._failure_outcome
    rig._action_failure = lambda *args: RobotController._action_failure(rig, *args)
    cfg = rig.configuration
    cfg.selection = SimpleNamespace(
        station=SimpleNamespace(platform=SimpleNamespace(base_from_platform=np.eye(4))),
        robot_camera=SimpleNamespace(reference_from_camera_link=np.eye(4), sha256="x"),
        bin=SimpleNamespace(points=[]))
    attitude = SimpleNamespace(accepted=True, rotation=np.eye(3), offset_direction="cw",
                               rotation_from_home_deg=0., mirrored=False,
                               selected_camera_platform_xy=(0., 0.))
    monkeypatch.setattr(controller_module, "select_pick_attitude", lambda *_args: attitude)
    candidates = [SimpleNamespace(identifier=f"item{i}", position_m=(i * .1, 0., .3),
                                  quaternion=(0., 0., 0., 1.)) for i in range(count)]
    rig.requests = []
    rig.candidates = SimpleNamespace(
        request=lambda *_a, **_k: rig.requests.append("detect") or SimpleNamespace(
            candidates=candidates, debug_message=""))
    original_home = rig._execute_home

    def home(**kwargs):
        if rig.managed.session is None:
            rig.feed["tool_vector_actual"] = pose_values(cfg.home_matrix)
        else:
            original_home(**kwargs)
    rig._execute_home = home
    rig.hardware.acquisitions = iter([True] * count)
    rig.lost = []
    rig.batches = []

    def lose(targets, kwargs):
        rig.batches.append((targets, kwargs))
        index = rig.managed.session.held_index
        if kwargs.get("require_suction") and index in losses and index not in rig.lost:
            rig.lost.append(index)
            rig.lose_suction()
            if loss_at == "preflight":
                RobotController._preflight_item_state(rig, True)
            rig.feed["tool_vector_actual"] = pose_values(
                targets[-1 if loss_at == "home" else 0].matrix)
            DobotTransport._monitor_motion_policy(
                rig.hardware, rig.snapshot(), require_suction=True, forbid_suction=False,
                stop_on_suction=False, before_suction=None, planned_outputs={})
    rig.hardware.on_move = lose
    rig.finished = []
    rig.goal = SimpleNamespace(
        request=SimpleNamespace(save_debug_images=False), is_cancel_requested=False,
        succeed=lambda: rig.finished.append("success"),
        abort=lambda: rig.finished.append("abort"),
        canceled=lambda: rig.finished.append("canceled"))
    rig.execute = lambda: RobotController._execute_pick_action(rig, rig.goal)
    return rig


@pytest.mark.parametrize("loss_at", ["preflight", "retract", "home"])
@pytest.mark.parametrize("count", [1, 2])
@pytest.mark.parametrize("di1_recovers", [False, True])
@pytest.mark.parametrize("prepick", [20., 80.])
def test_active_pick_puts_back_without_recovery_or_action_failure(
        monkeypatch, count, loss_at, di1_recovers, prepick):
    rig = action_rig(monkeypatch, count=count, loss_at=loss_at, prepick=prepick)
    confirm = rig.hardware.confirm_stop

    def confirmed(future, **kwargs):
        confirm(future, **kwargs)
        if di1_recovers:
            rig.set_di1(True)
    rig.hardware.confirm_stop = confirmed
    result = rig.execute()

    assert rig.finished == ["success"]
    assert result.outcome == (result.NO_PICK if count == 1 else result.SUCCESS)
    assert result.final_state == ("READY" if count == 1 else "HOLDING")
    assert result.attempted_candidates == count
    assert result.selected_candidate_id == ("" if count == 1 else "item1")
    assert states(rig) == (["DROPPED"] if count == 1 else ["DROPPED", "HELD"])
    assert rig.requests == ["detect"]
    assert not any(entry[0] == "recover" for entry in rig.log)
    assert not any(entry[0] == "state" and entry[1] in (
        "FAULT", "RECOVERY_REQUIRED", "RECOVERING", "PAUSED") for entry in rig.log)
    stop = next(i for i, entry in enumerate(rig.log) if entry[0] == "stop_confirmed")
    release = next(i for i, entry in enumerate(rig.log)
                   if entry[0] == "move" and "return_release" in entry[1])
    pulse = rig.log.index(("pulse", 50))
    assert stop < release < pulse
    assert rig.log[release][1] == ("park_transit", "return_release")
    assert rig.log[release][2]["preserve_outputs"]
    release_pose = next(targets[-1].matrix for targets, kwargs in rig.batches
                        if kwargs["batch_name"] == "return_item_to_release")
    assert release_pose[2, 3] == pytest.approx(.3 + prepick / 1000)
    following = next(entry for entry in rig.log[pulse + 1:] if entry[0] == "move")
    assert following[1][:2] == ("return_clearance", "return_park_transit")
    assert following[1][2:] == (("home",) if count == 1 else (
        "p2_transit", "p2_initial", "p2_prepick", "p2_pick"))
    assert rig.startup_complete
    assert not rig.operation_lock.locked()


def test_repeated_losses_consume_each_saved_candidate_once(monkeypatch):
    rig = action_rig(monkeypatch, count=3, losses=(1, 2, 3))
    result = rig.execute()
    assert result.outcome == result.NO_PICK and result.final_state == "READY"
    assert rig.lost == [1, 2, 3]
    assert states(rig) == ["DROPPED"] * 3
    assert sum(entry[0] == "pulse" for entry in rig.log) == 3
    assert rig.requests == ["detect"]


@pytest.mark.parametrize("failure", ["stop", "pending", "source", "output", "release"])
def test_auto_return_failure_blocks_later_pick_and_requires_recovery(monkeypatch, failure):
    rig = action_rig(monkeypatch)

    def fail(*_args, **_kwargs):
        error = (StopUnconfirmed if failure == "stop" else
                 CommandResponseTimeout if failure == "pending" else FeedbackFailure)
        raise error(f"injected {failure} failure")

    if failure == "stop":
        rig.hardware.confirm_stop = fail
    elif failure == "pending":
        rig.hardware.ensure_no_pending_response = fail
    elif failure == "release":
        rig.hardware.exhaust_pulse = fail
    else:
        lose = rig.hardware.on_move

        def corrupt(targets, kwargs):
            if kwargs.get("require_suction"):
                if failure == "source":
                    rig.configuration.validate_sources = fail
                else:
                    rig.feed["digital_outputs"] &= ~(1 << 12)
            lose(targets, kwargs)
        rig.hardware.on_move = corrupt
    result = rig.execute()
    assert rig.finished == ["abort"]
    assert result.final_state in ("RECOVERY_REQUIRED", "FAULT")
    assert states(rig)[1] == "PENDING"
    assert not any(entry[0] == "move" and "p2_pick" in entry[1] for entry in rig.log)
    if failure != "release":
        assert not any(entry[0] == "pulse" for entry in rig.log)
    assert not rig.operation_lock.locked()


@pytest.mark.parametrize("phase", ["return_item_to_release", "return_item_to_candidate_2_pick"])
def test_direct_stop_preempts_automatic_return_and_continuation(monkeypatch, phase):
    rig = action_rig(monkeypatch)
    lose = rig.hardware.on_move

    def stop(targets, kwargs):
        if kwargs.get("batch_name") == phase:
            rig.cancel_event.set()
            raise OperationCanceled("direct Stop")
        lose(targets, kwargs)
    rig.hardware.on_move = stop
    result = rig.execute()
    assert result.outcome == result.CANCELED
    assert result.final_state == "RECOVERY_REQUIRED"
    assert states(rig)[1] == "PENDING"
    assert not any(entry[0] == "move" and "p2_pick" in entry[1] for entry in rig.log)
    if phase == "return_item_to_release":
        assert not any(entry[0] == "pulse" for entry in rig.log)


def test_pause_after_automatic_release_resumes_next_candidate_without_releasing_twice(monkeypatch):
    rig = action_rig(monkeypatch)
    lose = rig.hardware.on_move
    paused = []

    def pause(targets, kwargs):
        if kwargs.get("batch_name") == "return_item_to_candidate_2_pick" and not paused:
            paused.append(True)
            rig.managed.session.admitted(next(t for t in targets if t.name == "p2_transit"))
            rig.managed.request("pause")
            rig.managed.checkpoint()
        lose(targets, kwargs)
    rig.hardware.on_move = pause
    result = rig.execute()
    assert result.outcome == result.SUCCESS and rig.finished == ["success"]
    assert states(rig) == ["DROPPED", "HELD"]
    assert ("state", "PAUSED") in rig.log
    assert sum(entry[0] == "pulse" for entry in rig.log) == 1


def test_direct_stop_at_automatic_return_handover_leaves_no_managed_owner(monkeypatch):
    rig = action_rig(monkeypatch)
    transition = rig._transition

    def stop_at_handover(state, message):
        if state == "RETURNING_ITEM":
            rig._request_stop("Direct Stop during automatic-return handover")
        transition(state, message)
    rig._transition = stop_at_handover
    result = rig.execute()
    assert result.outcome == result.CANCELED
    assert result.final_state == "RECOVERY_REQUIRED"
    assert not rig.managed.executing
    assert not rig.operation_lock.locked()
    assert not any(entry[0] == "move" and "return_release" in entry[1] for entry in rig.log)
    assert not any(entry[0] == "pulse" for entry in rig.log)
