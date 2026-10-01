"""Acquisition failure choices use real retry/pause/return logic and fake hardware."""

from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from robot_controller_interfaces.action import PlaceItem

from robot_controller.controller import RobotController
from robot_controller.errors import (CommandRejected, FeedbackFailure, HeldSuctionLost,
                                     ManagedInterruption, OperationCanceled)
from robot_controller.hardware import DobotTransport
from robot_controller.kinematics import pose_values
from robot_controller.placement import PlacementOperation
from robot_controller.state_machine import ControllerStateMachine
from test_operation_retries import TrayRig
from test_recovery_return import RecoveryRig


def acquisition_rig(monkeypatch, replies, *, held=True):
    tray = TrayRig(monkeypatch, replies)
    node = RecoveryRig(count=2)
    node.machine = ControllerStateMachine(initial="HOLDING" if held else "READY")
    node.startup_complete = True
    node.headless = held
    node._begin_operation("place")
    node.configuration.profile = tray.config.profile
    node.configuration.tray.__dict__.update(tray.config.tray.__dict__)
    node.configuration.validate_sources = Mock()
    node.feed["tool_vector_actual"] = pose_values(node.configuration.tray.detect_matrix)
    if not held:
        node.managed.session = None
        node.holding_item = False
        node.lose_suction()
        node.expected_outputs = dict.fromkeys((1, 2, 13, 14), False)
        node.feed["digital_outputs"] = 0
    node.placement = PlacementOperation(30., 40., 90., require_held_item=held)
    node.monitor.output_history = lambda _sequence: ()
    node._preflight_item_state = lambda *args: RobotController._preflight_item_state(node, *args)
    node.wait_for_resume = node.managed.checkpoint
    node.hardware.home_already_reached = lambda _joints: np.allclose(
        node.hardware.current_pose(), node.configuration.tray.detect_matrix)

    def check_position(_joints):
        if not node.hardware.home_already_reached(_joints):
            raise FeedbackFailure("Not at Tray Detect position")
    node.hardware.wait_tray_position = check_position
    node.get_clock = tray.node.get_clock
    node._service_providers = tray.node._service_providers
    node._perception_ready = lambda _action: True
    node.trays = tray.observer
    node.trays.node = node
    node.wait_control = lambda seconds: (
        node.on_wait() if node.machine.state == "PAUSED" else tray.advance(seconds))
    progress = node.operation_progress

    def report(phase, message, **kwargs):
        progress(phase, message, **kwargs)
        node.phase = phase
    node.operation_progress = report
    node._failure_outcome = RobotController._failure_outcome
    node._action_failure = lambda *args: RobotController._action_failure(node, *args)
    node._finish_placement_queue = lambda: RobotController._finish_placement_queue(node)
    node.goal = SimpleNamespace(
        request=PlaceItem.Goal(x_mm=30., y_mm=40., rotation_deg=90.),
        succeed=Mock(), abort=Mock(), canceled=Mock(), is_cancel_requested=False)
    return node, tray


def exhaust(node):
    node._transition("PLACING", "Observing tray")
    with pytest.raises(ManagedInterruption, match="failed after 3 attempts"):
        node.placement.run(node)
    assert node.machine.state == "PAUSING"
    assert node.placement.acquisition_paused
    assert not any(row[0] in ("move", "output", "pulse") for row in node.log)


@pytest.mark.parametrize("held", [False, True])
@pytest.mark.parametrize("reply", [None, "timeout"])
def test_exhaustion_pauses_in_place_preserving_outputs_source_and_ownership(
        monkeypatch, held, reply):
    node, tray = acquisition_rig(monkeypatch, [reply] * 3, held=held)
    pose, outputs = node.hardware.current_pose(), node.feed["digital_outputs"]
    exhaust(node)

    def paused():
        assert node.machine.state == "PAUSED"
        assert node.phase == "TRAY_ACQUISITION_PAUSED"
        assert "failed after 3 attempts" in node.machine.message
        assert "Place Item (Retry)" in node.machine.message
        assert "Pick Item" not in node.machine.message
        assert ("Return Item" in node.machine.message) is held
        assert node.operation_lock.locked() and node.active_action == "place"
        assert np.array_equal(node.hardware.current_pose(), pose)
        assert node.feed["digital_outputs"] == outputs
        assert node.managed.can_return_item() is held
        if held:
            assert node.managed.session.attempts[0].state == "HELD"
        assert tray.client.call_async.call_count == 3
        node.cancel_event.set()
    node.on_wait = paused
    with pytest.raises(OperationCanceled):
        node.placement.handle_pause(node)
    assert not any(row[0] in ("move", "output", "pulse") for row in node.log)
    assert node.trays.pending is None


def test_continue_explicitly_grants_new_three_request_budget(monkeypatch):
    node, tray = acquisition_rig(monkeypatch, [None] * 5 + ["valid"])
    exhaust(node)
    old_budget = node.placement.tray_attempts
    node.on_wait = node.managed.continue_operation
    node.placement.handle_pause(node)
    assert node.machine.state == "PLACING"
    assert old_budget.count == 3 and node.placement.tray_attempts.count == 0
    assert not node.placement.acquisition_failure
    # Admission geometry is covered by the real transport suite; isolate the
    # observation retry here without executing another fake placement route.
    node.hardware.move_batch = Mock(side_effect=[(False, np.eye(4)), None])
    node.placement.run(node)
    assert tray.client.call_async.call_count == 6
    assert node.placement.tray_attempts.count == 3
    assert node.hardware.move_batch.call_count == 2
    assert node.hardware.move_batch.call_args.kwargs["queue_only"]


def test_another_exhausted_budget_pauses_again_without_automatic_fourth_request(monkeypatch):
    node, tray = acquisition_rig(monkeypatch, [None] * 6)
    for total in (3, 6):
        exhaust(node)
        assert tray.client.call_async.call_count == total
        node.on_wait = node.managed.continue_operation
        node.placement.handle_pause(node)


@pytest.mark.parametrize("headless", [False, True])
def test_return_puts_item_at_saved_bin_prepick_then_homes_and_cancels_place(monkeypatch, headless):
    node, tray = acquisition_rig(monkeypatch, [None] * 3)
    node.headless = headless
    source = node.managed.session.attempts[0].plan
    node.on_wait = lambda: node.managed.request("return")
    result = RobotController._execute_place_action(node, node.goal)
    assert result.outcome == result.CANCELED and result.final_state == "READY"
    assert "returned to bin" in result.message
    node.goal.abort.assert_called_once()
    node.goal.succeed.assert_not_called()
    assert not node.operation_lock.locked() and node.placement is None
    assert not node.holding_item and node.managed.session.held_index is None
    assert node.managed.session.attempts[0].state == "RETURNED"
    assert tray.client.call_async.call_count == 3
    routes = [row for row in node.log if row[0] == "move"]
    assert routes[-1][1][-1] == "home"
    assert len(routes) == 2
    assert routes[0][2]["batch_name"] == "return_item_drop"
    assert routes[1][2]["batch_name"] == "return_item_queued_home"
    assert routes[0][1][-2:] == ("return_pre", "return_release")
    assert routes[1][1] == ("return_retract", "home")
    progress = routes[0][2]["placement"].owner
    release = next(target for target in progress.plan if target.name == "return_release")
    assert np.array_equal(release.matrix, source[2].matrix)
    assert not any(row[0] in ("output", "pulse", "home") for row in node.log)
    assert np.array_equal(node.hardware.current_pose(), node.configuration.home_matrix)
    assert not any(name.startswith("place_") for row in routes for name in row[1])


@pytest.mark.parametrize("returning", [False, True])
def test_gui_bin_return_restores_normal_held_feedback_checks(monkeypatch, returning):
    node, _tray = acquisition_rig(monkeypatch, [])
    node.placement.require_held_item = False
    node.placement.returning_to_bin = returning
    node.lose_suction()
    transport = SimpleNamespace(node=node)
    if returning:
        with pytest.raises(HeldSuctionLost, match="lost DI1"):
            DobotTransport._validate_held_snapshot(transport, node.snapshot())
        assert node.managed.session.attempts[0].state == "DROPPED"
    else:
        DobotTransport._validate_held_snapshot(transport, node.snapshot())
        assert node.managed.session.attempts[0].state == "HELD"


@pytest.mark.parametrize("failure", ["stop", "source", "suction", "position", "outputs"])
def test_paused_acquisition_keeps_failure_gates(monkeypatch, failure):
    node, tray = acquisition_rig(monkeypatch, [None] * 3)
    exhaust(node)

    def fail():
        if failure == "stop":
            node.cancel_event.set()
        elif failure == "source":
            node.configuration.validate_sources.side_effect = FeedbackFailure("source changed")
            node.managed.continue_operation()
        elif failure == "suction":
            node.lose_suction()
        elif failure == "position":
            node.feed["tool_vector_actual"][0] += 20.
        else:
            node.feed["digital_outputs"] ^= 2
    node.on_wait = fail
    with pytest.raises((OperationCanceled, FeedbackFailure)):
        node.placement.handle_pause(node)
    assert tray.client.call_async.call_count == 3
    assert not any(row[0] in ("move", "output", "pulse") for row in node.log)


@pytest.mark.parametrize("phase,issued", [
    ("OBSERVE", False), ("APPROACH", False), ("RELEASING", True), ("RELEASED", True)])
def test_other_placement_pauses_still_refuse_bin_return(monkeypatch, phase, issued):
    node, _tray = acquisition_rig(monkeypatch, [])
    node.placement.phase, node.placement.release_issued = phase, issued
    node.machine = ControllerStateMachine(initial="PAUSED")
    node.managed.kind = "pause"
    if issued:
        node.placement.acquisition_failure = "Old failed acquisition"
    assert not node.managed.can_return_item()
    with pytest.raises(CommandRejected, match="paused failed tray acquisition"):
        node.managed.request("return")


def test_invalid_response_remains_terminal_without_acquisition_pause(monkeypatch):
    node, tray = acquisition_rig(monkeypatch, ["valid"])
    tray.result.diagnostics_json = '{}'
    node._transition("PLACING", "Observing tray")
    with pytest.raises(FeedbackFailure, match="Invalid tray placement response"):
        node.placement.run(node)
    assert not node.placement.acquisition_failure and node.managed.kind is None
    assert tray.client.call_async.call_count == 1


def test_manual_pause_keeps_partly_used_acquisition_budget(monkeypatch):
    node, _tray = acquisition_rig(monkeypatch, [])
    node.placement.tray_attempts.count = 2
    node._transition("PLACING", "Observing tray")
    node.managed.request("pause")
    node.on_wait = node.managed.continue_operation
    node.placement.handle_pause(node)
    assert node.placement.tray_attempts.count == 2


def test_stop_during_bin_return_preserves_return_context(monkeypatch):
    node, _tray = acquisition_rig(monkeypatch, [None] * 3)
    exhaust(node)
    node.on_wait = lambda: node.managed.request("return")

    def stop_on_move(_targets, _kwargs):
        node.cancel_event.set()
        node.raise_if_cancelled()
    node.hardware.on_move = stop_on_move
    with pytest.raises(OperationCanceled):
        node.placement.handle_pause(node)
    assert node.managed.return_progress.phase == "APPROACH"
    assert node.managed.session.attempts[0].state == "HELD"
    assert not any(row[0] in ("output", "pulse") for row in node.log)
