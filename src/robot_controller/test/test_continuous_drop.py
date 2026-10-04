"""Confirmed held loss interrupts queues and retains the original candidate batch."""

from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.task import Future

from robot_controller.autorun import AutoRunOperation
from robot_controller.controller import RobotController
from robot_controller.errors import (
    CommandRejected, CommandResponseTimeout, FeedbackFailure, HeldSuctionLost, OperationCanceled)
from robot_controller.pick_session import PickSession
from robot_controller.state_machine import ControllerStateMachine
from test_auto_run import cycle_rig, queue_rig, request
from test_feedback_v2 import feed
from test_placement_queue import QueueRig, HELD, RELEASE
from test_recovery_return import RecoveryRig


def low_feedback(rig, outputs=HELD, count=8):
    for _ in range(count):
        sample = rig.emit(outputs=outputs, inputs=0)
        try:
            rig.node.managed.observe_continuous(sample)
        except HeldSuctionLost:
            pass  # Same boundary as the production FeedInfo callback.


@pytest.mark.parametrize("phase", ["OBSERVE", "APPROACH", "RELEASING"])
@pytest.mark.parametrize("required", [False, True])
def test_feedback_stops_during_detection_approach_and_descent_and_latches(phase, required):
    rig = QueueRig()
    node = rig.node
    placement = node.placement
    placement.require_held_item = required
    if phase != "OBSERVE":
        placement.begin_queue(node)
    if phase == "RELEASING":
        placement.issued(1)  # Dispatch is not evidence of intentional release.
    low_feedback(rig)
    assert [name for name, _ in rig.requests] == ["Stop"]
    assert node.managed.held_loss_pending and node.holding_item
    assert node.managed.session.held_index == 1
    assert node.managed.session.attempts[0].state == "DROPPED"
    assert not placement.release_confirmed
    rig.emit(outputs=HELD, inputs=1)
    with pytest.raises(HeldSuctionLost):
        node.managed.checkpoint()
    assert node.managed.session.next_eligible is None


def test_feed_callback_dispatches_stop_without_waiting_for_action_owner():
    rig = QueueRig()
    for _ in range(8):
        rig.clock += .01
        rig.timer += 1
        RobotController._on_feed(rig.node, SimpleNamespace(data=feed(
            controller_timer=rig.timer, digital_outputs=HELD, digital_input_bits=0)))
    assert rig.node.managed.held_loss_pending
    assert [name for name, _ in rig.requests] == ["Stop"]


def test_feed_callback_remains_live_after_drop_and_explicit_action_cancellation():
    rig = QueueRig()
    node = rig.node
    node.placement.begin_queue(node)
    low_feedback(rig)
    node.machine.transition("STOPPING", "Operator Stop")
    node.raise_if_cancelled = Mock(side_effect=OperationCanceled("Stop"))
    RobotController._on_feed(node, SimpleNamespace(data=feed(
        controller_timer=rig.timer + 1, digital_outputs=HELD, digital_input_bits=0)))
    assert node.managed.held_loss_pending and node.holding_item
    assert node.managed.session.attempts[0].state == "DROPPED"
    assert node.machine.state == "STOPPING"
    assert [name for name, _ in rig.requests] == ["Stop"]


def test_finger_transition_does_not_disable_monitor_before_suction_off():
    rig = QueueRig()
    rig.node.placement.begin_queue(rig.node)
    rig.node.placement.issued(1)
    low_feedback(rig, outputs=(1 << 12) | (1 << 13))
    assert rig.node.managed.held_loss_pending
    assert rig.node.holding_item


def test_planned_suction_off_and_neutral_retract_are_not_drops():
    rig = QueueRig()
    placement = rig.node.placement
    placement.begin_queue(rig.node)
    placement.issued(1)
    low_feedback(rig, outputs=RELEASE)
    placement.issued(2)
    low_feedback(rig, outputs=0)
    assert not rig.requests
    assert placement.release_confirmed and not rig.node.holding_item
    assert not rig.node.managed.held_loss_pending


def test_uncommanded_suction_off_cannot_end_held_monitoring():
    rig = QueueRig()
    rig.node.placement.begin_queue(rig.node)
    with pytest.raises(FeedbackFailure, match="Uncommanded"):
        rig.node.placement.observe(rig.node, rig.emit(outputs=2, inputs=0))
    assert rig.node.holding_item


def test_late_planned_release_cannot_erase_a_drop_latched_before_stop():
    rig = QueueRig()
    placement = rig.node.placement
    placement.begin_queue(rig.node)
    placement.issued(1)
    low_feedback(rig)
    low_feedback(rig, outputs=RELEASE)
    assert rig.node.holding_item and rig.node.managed.held_loss_pending
    assert not placement.release_confirmed
    assert rig.node.managed.session.attempts[0].state == "DROPPED"


def test_stop_reconciles_issued_release_transition_without_erasing_latched_source():
    rig = QueueRig()
    node = rig.node
    node.placement.run(node)
    low_feedback(rig)
    node.managed.containing_loss = True

    def wait(predicate, *_args, **_kwargs):
        assert not predicate(rig.emit(outputs=RELEASE, inputs=0, running=0))
        sample = rig.emit(outputs=RELEASE, inputs=0, running=0)
        assert predicate(sample)
        return sample
    rig.monitor.wait = wait
    rig.transport.confirm_stop(rig.transport.stop_future, allow_suction_loss=True)
    assert node.holding_item and node.managed.held_loss_pending
    assert node.managed.session.held_index == 1
    assert node.managed.session.attempts[0].state == "DROPPED"
    assert not node.placement.release_confirmed
    assert node.expected_outputs == {1: True, 2: False, 13: False, 14: True}
    node.placement.close_pending()


@pytest.mark.parametrize("pending_index", [1, 2, 3])
def test_drop_during_admission_resolves_reply_but_blocks_all_later_commands(pending_index):
    rig = QueueRig()
    node = rig.node
    node.wait_for_resume = node.managed.checkpoint
    pending = Future()
    for client in rig.transport.clients.values():
        original = client.call_async

        def send(message, call=original):
            result = call(message)
            if len(rig.requests) == pending_index:
                return pending
            return result
        client.call_async = send
    iterations = []

    def wait(_seconds):
        low_feedback(rig, count=1)
        iterations.append(1)
        if len(iterations) >= 8:
            pending.set_result(SimpleNamespace(res=0, robot_return="{4}"))
    node.wait_control = wait
    with pytest.raises(HeldSuctionLost):
        node.placement.run(node)
    assert pending.done()
    motions = [name for name, _ in rig.requests if name != "Stop"]
    assert motions == ["MovL", "MovLIO", "MovLIO"][:pending_index]
    assert "Stop" in [name for name, _ in rig.requests]
    assert node.managed.held_loss_pending


def recovery_rig(count=3):
    node = RecoveryRig(count=count)
    node.startup_complete = True
    node.machine = ControllerStateMachine(initial="PLACING")
    node._begin_operation("place")
    node.wait_for_resume = node.managed.checkpoint
    node.hardware.acquisitions = iter([True] * count)
    node.placement = None
    node.lose_suction()
    node.managed.interrupt_held_loss(node.snapshot())
    return node


def test_return_uses_saved_prepick_and_joins_next_eligible_pose_without_home_or_detection():
    node = recovery_rig()
    session = node.managed.session
    source = session.attempts[0].plan[2].matrix.copy()
    session.set_state(2, "ACTIVE")
    session.set_state(2, "FAILED")
    assert node.managed.continue_after_loss()
    assert node.managed.session is session
    assert [a.state for a in session.attempts] == ["DROPPED", "FAILED", "HELD"]
    assert node.machine.state == "HOLDING" and session.held_index == 3
    moves = [row for row in node.log if row[0] == "move"]
    assert not any("home" in row[1] for row in moves)
    assert any(row[1][:2] == ("return_clearance", "return_park_transit")
               and "p3_pick" in row[1] for row in moves)
    np.testing.assert_array_equal(source, session.attempts[0].plan[2].matrix)
    assert ("pulse", 50) in node.log
    stop_index = next(i for i, row in enumerate(node.log) if row[0] == "stop_confirmed")
    move_index = next(i for i, row in enumerate(node.log) if row[0] == "move")
    assert stop_index < move_index


def test_exhausted_drop_return_finishes_upward_without_home():
    node = recovery_rig(1)
    assert not node.managed.continue_after_loss()
    moves = [row for row in node.log if row[0] == "move"]
    assert moves[-1][1] == ("return_clearance", "return_park_transit")
    assert not any("home" in row[1] for row in moves)
    assert node.machine.state == "READY" and not node.holding_item


def test_direct_stop_preempts_automatic_drop_return():
    node = recovery_rig()
    node.cancel_event.set()
    with pytest.raises(OperationCanceled):
        node.managed.continue_after_loss()
    assert not any(row[0] in ("move", "output", "pulse") for row in node.log)


@pytest.mark.parametrize("reply", ["accepted", "rejected", "timeout", "stop"])
def test_outstanding_reply_must_resolve_successfully_before_recovery(reply):
    import time

    rig = QueueRig()
    pending = Future()
    audit = {"started": time.monotonic() - (6 if reply == "timeout" else 0)}
    rig.transport.pending_group = [("MovL", pending, audit)]
    rig.transport._finish_service_audit = Mock()

    def wait(_seconds):
        if reply == "stop":
            rig.node.raise_if_cancelled = Mock(side_effect=OperationCanceled("Stop"))
        else:
            pending.set_result(SimpleNamespace(res=0 if reply == "accepted" else -1))
    rig.node.wait_control = wait
    if reply == "accepted":
        rig.transport.resolve_interrupted_commands()
        assert rig.transport.pending_group is None
    else:
        error = {"rejected": CommandRejected, "timeout": CommandResponseTimeout,
                 "stop": OperationCanceled}[reply]
        with pytest.raises(error):
            rig.transport.resolve_interrupted_commands()
    assert not rig.requests


def test_auto_run_discards_speculative_batch_and_keeps_original_source(monkeypatch):
    node = recovery_rig()
    session = node.managed.session
    run = node.auto_run = AutoRunOperation(node, request(2))
    speculative = PickSession(["new-camera-pose"], [session.attempts[1].plan])
    run.bridge = SimpleNamespace(next_session=speculative)
    worker = run.prefetch = SimpleNamespace(close=Mock())
    run._run_cycles = Mock(side_effect=[HeldSuctionLost("lost before release"), True])
    assert run.run()
    worker.close.assert_called_once()
    assert run.prefetch is None and run.bridge is None
    assert speculative.attempts[0].state == "CANCELED"
    assert node.managed.session is session
    assert [a.state for a in session.attempts] == ["DROPPED", "HELD", "PENDING"]
    assert run.completed == 0 and run.resume_held


def test_drop_after_next_motion_admission_keeps_old_source_and_does_not_count_placement():
    rig, run, bridge = queue_rig()
    node = rig.node
    old = node.managed.session
    plan = recovery_rig().managed.session.attempts[1].plan
    bridge.next_session = PickSession(["speculative"], [plan])
    pending, node.placement.pending_motion = node.placement.pending_motion, None
    rig.transport.finish_batch(pending, handoff=lambda: True)
    appended = rig.transport._move_batch(
        (*run.queued_home(bridge), plan[0], plan[2], plan[3]),
        confirmed_start_pose=bridge.origin, placement_bridge=bridge,
        stop_on_suction=True, return_terminal_pose=True)
    next(appended)
    low_feedback(rig)
    with pytest.raises(HeldSuctionLost):
        rig.transport.finish_batch(appended)
    assert [name for name, _ in rig.requests][:7] == [
        "MovL", "MovLIO", "MovLIO", "MovL", "MovLIO", "MovL", "MovLIO"]
    assert node.managed.session is old and old.held_index == 1
    assert old.attempts[0].state == "DROPPED"
    assert not bridge.completed and run.completed == 0
    run.close()
    assert bridge.next_session.attempts[0].state == "CANCELED"


def test_old_placement_loss_propagates_out_of_speculative_pick_owner(monkeypatch):
    node = recovery_rig()
    next_session = PickSession(["speculative"], [node.managed.session.attempts[1].plan])
    bridge = SimpleNamespace(next_session=next_session, completed=False)
    monkeypatch.setattr("robot_controller.managed_control.PickExecutor.run",
                        Mock(side_effect=HeldSuctionLost("old item")))
    recover = node.managed._return_after_suction_loss = Mock()
    with pytest.raises(HeldSuctionLost):
        node.managed.run_pick([next_session.attempts[0].plan], check=lambda _i: None,
                              placement_bridge=bridge)
    recover.assert_not_called()
    assert node.managed.session is not next_session


def test_idle_held_loss_reserves_the_existing_hardware_owner_before_worker():
    rig = QueueRig()
    node = rig.node
    node.machine = ControllerStateMachine(initial="HOLDING")
    node._begin_operation = Mock()
    node.managed.start_loss_worker = Mock()
    for _ in range(8):
        node.managed.observe_continuous(rig.emit(outputs=HELD, inputs=0, running=0))
    RobotController._supervise(node)
    node._begin_operation.assert_called_once_with("pick")
    node.managed.start_loss_worker.assert_called_once()
    assert rig.requests[0][0] == "Stop"


@pytest.mark.parametrize("accepted", [False, True])
def test_manual_place_recovers_before_or_after_action_acceptance(monkeypatch, accepted):
    node = recovery_rig()
    placement = SimpleNamespace(
        pending_motion=object(), observing=False, close_pending=Mock(),
        run=Mock(side_effect=HeldSuctionLost("drop")),
        finish_pending=Mock(side_effect=HeldSuctionLost("drop")))
    node.placement = placement
    if accepted:
        RobotController._finish_placement_queue(node)
    else:
        monkeypatch.setattr("robot_controller.controller.PlacementOperation",
                            lambda *_a, **_k: placement)
        goal = SimpleNamespace(request=request(), abort=Mock())
        result = RobotController._execute_place_action(node, goal)
        assert result.outcome == result.CANCELED
        goal.abort.assert_called_once()
    assert node.machine.state == "HOLDING"
    assert node.managed.session.held_index == 2
    assert node.placement is None and not node.operation_lock.locked()


def test_auto_run_resumes_placement_after_saved_pick_without_an_extra_pick_request(monkeypatch):
    run, node, order, workers = cycle_rig(monkeypatch, 2)
    original = node.hardware.finish_batch
    calls = []

    def lose_once(*args, **kwargs):
        if not calls:
            calls.append(1)
            raise HeldSuctionLost("drop during placement")
        return original(*args, **kwargs)

    def recover():
        order.append("saved batch recovery Pick")
        node.placement = None
        node._transition("RETURNING_ITEM", "Return")
        node._transition("PICKING", "Next saved candidate")
        node._transition("HOLDING", "Tray Detect")
        return True
    node.managed.continue_after_loss = recover
    node.hardware.finish_batch = lose_once
    assert run.run() and run.completed == 2
    assert order.count("Pick") == 1
    assert order.count("saved batch recovery Pick") == 1
    assert order.count("tray acquisition") == 3
    assert workers[0].closed
