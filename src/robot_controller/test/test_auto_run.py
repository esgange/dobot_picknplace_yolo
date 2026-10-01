"""Auto Run uses synthetic robot queues and read-only detector workers only."""

from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock
import threading

import numpy as np
import pytest

import robot_controller.autorun as auto_module
from robot_controller.autorun import (
    AutoRunOperation, CandidatePrefetch, PlacementBridge, validate_quantity)
from robot_controller.controller import RobotController
from robot_controller.errors import FeedbackFailure, OperationCanceled
from robot_controller.motion import home_targets, pick_targets
from robot_controller.pick_session import PickSession
from robot_controller_interfaces.action import AutoRun
from robot_controller_interfaces.srv import Command
from test_placement_queue import QueueRig, HELD, RELEASE, OPEN
from test_status_ui import status, window  # noqa: F401


def request(quantity=2):
    return AutoRun.Goal(configuration_id="test", quantity=quantity,
                        x_mm=30., y_mm=40., rotation_deg=0., save_debug_images=False)


def queue_rig(quantity=2):
    rig = QueueRig()
    node = rig.node
    node._home_plan = lambda current: home_targets(
        current, node.configuration.home_matrix, node.configuration.home_joints,
        speed_percent=80, acceleration_percent=70)
    run = node.auto_run = AutoRunOperation(node, request(quantity))
    rig.order.clear()
    node.placement.run(node)
    bridge = run.bridge = PlacementBridge(run, node.placement)
    return rig, run, bridge


@pytest.mark.parametrize("value", [0, -1, 10001, True, 1.5, "2", None])
def test_auto_quantity_is_a_bounded_positive_integer(value):
    with pytest.raises(ValueError):
        validate_quantity(value)


def test_next_home_and_pick_are_admitted_before_placement_finishes_without_old_di1_acquisition():
    rig, run, bridge = queue_rig()
    old_session = rig.node.managed.session
    item = np.eye(4)
    item[:3, 3] = [.4, .1, .25]
    plan = pick_targets(rig.node.configuration.home_matrix, item,
                        rig.node.configuration.profile, 1)
    bridge.next_session = PickSession(["next-item"], [plan])
    pending = rig.node.placement.pending_motion
    rig.node.placement.pending_motion = None
    rig.transport.finish_batch(pending, handoff=lambda: True)
    assert rig.node.placement.phase != "DONE"
    assert rig.monitor.snapshot().feed["digital_input_bits"] & 1
    rig.transport.confirm_stop = Mock(side_effect=AssertionError("No stationary pickup wait"))
    rig.steps = iter([
        dict(outputs=RELEASE, inputs=OPEN, currentCommandId=2),
        dict(outputs=0, inputs=OPEN, currentCommandId=3),
        dict(outputs=0, inputs=OPEN, currentCommandId=4),
        dict(outputs=(1 << 12) | (1 << 13), inputs=1, currentCommandId=7),
    ])
    acquired, _origin = rig.transport.move_batch(
        (*run.queued_home(bridge), plan[0], plan[2], plan[3]),
        stop_on_suction=True, pick_settling_sec=10., return_terminal_pose=True,
        confirmed_start_pose=bridge.origin, placement_bridge=bridge)
    assert acquired
    assert [name for name, _ in rig.requests] == [
        "MovL", "MovLIO", "MovLIO", "MovL", "MovLIO", "MovL", "MovLIO", "Stop"]
    # No new feedback/arrival wait between placement admission and all next targets.
    assert rig.order[:7] == [name for name, _ in rig.requests[:7]]
    assert run.completed == 1
    assert old_session.attempts[0].state == "PLACED" and old_session.held_index is None
    assert rig.node.managed.session is bridge.next_session
    assert bridge.next_session.attempts[0].state == "ACTIVE"
    assert rig.node.placement is None
    assert rig.node.machine.state == "PICKING"


def test_final_home_is_queued_behind_placement_and_success_waits_only_at_final_home():
    rig, run, bridge = queue_rig(quantity=1)
    rig.steps = iter([
        dict(outputs=0, inputs=OPEN, currentCommandId=3),
        dict(outputs=0, inputs=OPEN, currentCommandId=4),
        dict(outputs=0, inputs=OPEN, home=True),
    ])
    run.finish_home(bridge)
    assert [name for name, _ in rig.requests] == ["MovL", "MovLIO", "MovLIO", "MovL"]
    assert rig.order[:4] == ["MovL", "MovLIO", "MovLIO", "MovL"]
    assert run.completed == 1 and rig.node.machine.state == "READY"
    assert rig.monitor.snapshot().robot_enabled
    rig.node._preflight_item_state.assert_called_with(False)


@pytest.mark.parametrize("bad_outputs,bad_di1", [(HELD, 1), (0, 1), (1 << 13, 0)])
def test_crossing_placement_boundary_requires_neutral_outputs_and_di1_clear(bad_outputs, bad_di1):
    rig, run, bridge = queue_rig()
    bridge.accepted(0, SimpleNamespace(robot_return="{4}"))
    sample = rig.emit(outputs=bad_outputs, inputs=bad_di1, currentCommandId=4)
    with pytest.raises(FeedbackFailure, match="without neutral"):
        bridge.observe(sample)
    assert run.completed == 0 and rig.node.placement is bridge.placement
    assert bridge.old_session.attempts[0].state == "HELD"
    bridge.placement.close_pending()


def test_neutral_feedback_or_old_home_id_alone_cannot_count_placement():
    rig, run, bridge = queue_rig()
    old = rig.emit(outputs=0, inputs=0, currentCommandId=4)
    bridge.accepted(0, SimpleNamespace(robot_return="{4}"))
    bridge.observe(old)
    assert run.completed == 0
    bridge.observe(rig.emit(outputs=0, inputs=0, currentCommandId=3))
    assert run.completed == 0
    bridge.observe(rig.emit(outputs=0, inputs=0, currentCommandId=4))
    bridge.observe(rig.monitor.snapshot())
    assert run.completed == 1
    bridge.placement.close_pending()


def test_direct_stop_during_append_keeps_old_source_and_never_counts_the_placement():
    rig, run, bridge = queue_rig()
    old_session = rig.node.managed.session
    canceled = [False]

    def on_request(index):
        if index == 4:
            canceled[0] = True

    rig.on_request = on_request
    rig.node.cancel_requested = lambda: canceled[0]
    bridge.placement.close_pending()
    with pytest.raises(OperationCanceled):
        rig.transport.move_batch((*run.queued_home(bridge), *run.queued_home(bridge)),
                                 confirmed_start_pose=bridge.origin, placement_bridge=bridge)
    assert [name for name, _ in rig.requests].count("MovL") == 2
    assert rig.node.managed.session is old_session and old_session.held_index == 1
    assert run.completed == 0


def test_prefetch_is_read_only_and_cancellation_discards_its_result():
    entered, finish = threading.Event(), threading.Event()
    requested = []

    def acquire(configuration, *, save_debug_images, cancel):
        requested.append((configuration, save_debug_images, threading.current_thread().name))
        entered.set()
        assert finish.wait(2.)
        assert cancel()
        return object()

    node = SimpleNamespace(candidates=SimpleNamespace(request=acquire),
                           cancel_requested=lambda: False)
    worker = CandidatePrefetch(node, "bound-config", True)
    assert entered.wait(2.)
    worker.cancel.set()
    finish.set()
    worker.close()
    with pytest.raises(OperationCanceled):
        worker.future.result()
    assert requested == [("bound-config", True, "auto_run_item_poses")]


def test_auto_ui_sends_count_and_offsets_and_locks_all_manual_controls(window):  # noqa: F811
    sent = []
    window.node.action_clients["auto_run"] = SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=lambda goal, **_kwargs: sent.append(goal) or Future())
    window.node.status = status(tray_position_recorded=True, at_tray_detect=False)
    window.auto_quantity.setValue(7)
    window._refresh()
    assert window.auto_run_button.isEnabled()  # Starting at Tray Detect is not required.
    window._action("auto_run")
    assert len(sent) == 1 and isinstance(sent[0], AutoRun.Goal)
    assert sent[0].quantity == 7 and (sent[0].x_mm, sent[0].y_mm) == (30., 40.)
    window.node.status = status(state="PLACING", operation_active=True, operation="auto_run",
                                auto_run_active=True, auto_run_requested=7, auto_run_completed=2)
    window.pending_goal = None
    window._refresh()
    assert all(window._availability().values())
    assert window.stop.isEnabled()
    assert not window.auto_quantity.isEnabled() and not window.place_x.isEnabled()
    assert window.auto_progress.text() == "2/7 completed"
    assert window.status.text() == "AUTO RUN"


@pytest.mark.parametrize("command", ["pause", "continue", "return_item"])
def test_external_manual_queue_controls_are_rejected_during_auto_run(command):
    node = SimpleNamespace(auto_run=object(), machine=SimpleNamespace(state="PICKING"),
                           managed=Mock(), events=Mock())
    response = Command.Response()
    if command == "continue":
        RobotController._continue(node, None, response)
    else:
        RobotController._managed_request(node, command, response)
    assert not response.success and "Auto Run" in response.message
    assert not node.managed.mock_calls


def cycle_rig(monkeypatch, quantity, *, slow=False, detection_error=False):
    from robot_controller.motion import Target
    from robot_controller.state_machine import ControllerStateMachine

    order, workers = [], []
    node = SimpleNamespace(
        configuration=object(), managed=SimpleNamespace(session=None),
        machine=ControllerStateMachine(initial="READY"), placement=None,
        raise_if_cancelled=lambda: None, operation_progress=Mock(), events=Mock(),
        _preflight_item_state=Mock(),
        monitor=SimpleNamespace(snapshot=lambda **_k: object()))
    node._transition = lambda state, message: node.machine.transition(state, message)

    class Placement:
        def __init__(self, *_args, **kwargs):
            assert kwargs == {"require_held_item": True}
            self.phase = "OBSERVE"
            self.plan = (Target("retract", np.eye(4), 100, 100),)
            self.observing = True

        def run(self, owner):
            assert owner.machine.state == "PLACING"
            order.append("place queued")
            self.phase = "APPROACH"
            self.pending_motion = object()

        def close_pending(self):
            self.pending_motion = None

    class Worker:
        def __init__(self, *_args):
            assert order[-1] == "place queued"
            order.append("prefetch")
            self.future = Future()
            self.closed = False
            workers.append(self)
            if detection_error:
                self.future.set_exception(FeedbackFailure("detector unavailable"))
            elif not slow:
                self.future.set_result(f"batch{len(workers)}")

        def close(self):
            self.closed = True

    monkeypatch.setattr(auto_module, "PlacementOperation", Placement)
    monkeypatch.setattr(auto_module, "CandidatePrefetch", Worker)
    run = AutoRunOperation(node, request(quantity))

    def pick(batch, bridge):
        if batch is not None:
            assert batch == f"batch{len(workers)}" and workers[-1].closed
        if bridge is not None:
            assert bridge.placement.phase != "DONE"
            order.append("append next Pick")
            bridge.home_id = 4
            bridge._complete()
        else:
            node._transition("PICKING", "Pick")
            order.append("Pick")
        node._transition("HOLDING", "Tray Detect reached")
        return True

    def finish(_pending, *, handoff):
        assert handoff() is not slow
        if slow:
            order.append("placement arrived")
            node.placement.phase = "DONE"

    def home(bridge):
        assert order[-1] == "place queued"
        order.append("append final Home")
        bridge.home_id = 4
        bridge._complete()
        node._transition("READY", "Home")
        run.bridge = None

    run._pick = pick
    run.finish_home = home
    node.hardware = SimpleNamespace(finish_batch=finish, _idle=lambda _sample: True)
    node.wait_control = lambda _seconds: workers[-1].future.set_result(f"batch{len(workers)}")
    return run, node, order, workers


@pytest.mark.parametrize("quantity", [1, 2, 4])
@pytest.mark.parametrize("slow", [False, True])
def test_counted_cycles_prefetch_only_when_needed_and_append_final_home(
        monkeypatch, quantity, slow):
    run, node, order, workers = cycle_rig(monkeypatch, quantity, slow=slow)
    assert run.run()
    assert run.completed == quantity
    assert order.count("place queued") == quantity
    assert order.count("prefetch") == quantity - 1
    assert order.count("append next Pick") == (0 if slow else quantity - 1)
    assert order[-1] == "append final Home"
    assert node.machine.state == "READY"
    assert all(worker.closed for worker in workers)


def test_prefetch_error_ends_run_without_queuing_another_pick_or_counting_release(monkeypatch):
    run, _node, order, workers = cycle_rig(monkeypatch, 3, detection_error=True)
    with pytest.raises(FeedbackFailure, match="detector unavailable"):
        run.run()
    run.close()
    assert order == ["Pick", "place queued", "prefetch"]
    assert run.completed == 0 and workers[0].closed


def test_stop_before_prefetch_consumption_discards_it_without_next_pick(monkeypatch):
    run, node, order, workers = cycle_rig(monkeypatch, 3)

    def cancel(_pending, **_kwargs):
        def canceled():
            raise OperationCanceled("operator Stop")
        node.raise_if_cancelled = canceled

    node.hardware.finish_batch = cancel
    with pytest.raises(OperationCanceled):
        run.run()
    run.close()
    assert order == ["Pick", "place queued", "prefetch"]
    assert run.completed == 0 and workers[0].closed


def test_auto_pick_exhausts_only_three_batches_and_reports_no_pick(monkeypatch):
    from test_automatic_return import action_rig

    node = action_rig(monkeypatch, count=1, losses=())
    node.managed.session = None

    def empty(*_args, **_kwargs):
        node.requests.append("detect")
        return SimpleNamespace(identifier=f"empty{len(node.requests)}", candidates=[])
    node.candidates.request = empty
    node._plan_candidate_batch = lambda batch: RobotController._plan_candidate_batch(node, batch)
    run = AutoRunOperation(node, request(4))
    run.completed = 2
    assert not run._pick()
    assert len(node.requests) == 3
    assert run.completed == 2 and node.machine.state == "READY"


@pytest.mark.parametrize("missing", ["pick", "place"])
def test_auto_goal_requires_both_detectors_before_reserving_robot(missing):
    from rclpy.action import GoalResponse

    node = SimpleNamespace(
        startup_complete=True, headless=False, holding_item=False,
        configuration=SimpleNamespace(configuration_id="test", selection=object(),
                                      tray=SimpleNamespace(detect_joints=(0.,) * 6)),
        machine=SimpleNamespace(state="READY"), _begin_operation=Mock(),
        _perception_ready=lambda name: name != missing)
    assert RobotController._reserve_goal(node, "auto_run", "test") == GoalResponse.REJECT
    node._begin_operation.assert_not_called()


@pytest.mark.parametrize("value", ["", "0", "10001"])
def test_invalid_quantity_cannot_dispatch_auto_goal(window, value):  # noqa: F811
    send = Mock()
    window.node.action_clients["auto_run"] = SimpleNamespace(
        server_is_ready=lambda: True, send_goal_async=send)
    window.node.status = status(tray_position_recorded=True)
    window.auto_quantity.lineEdit().setText(value)
    window._action("auto_run")
    assert window._availability()["auto_run"]
    send.assert_not_called()


def test_preview_never_dispatches_auto_run(window):  # noqa: F811
    send = Mock()
    window.node.action_clients["auto_run"] = SimpleNamespace(
        server_is_ready=lambda: True, send_goal_async=send)
    window.node.status = status(tray_position_recorded=True)
    window.preview_mode = True
    window._action("auto_run")
    assert window._availability()["auto_run"]
    send.assert_not_called()


@pytest.mark.parametrize("outcome", ["success", "no_pick", "failure"])
def test_native_auto_action_reports_quantity_and_releases_its_one_owner(monkeypatch, outcome):
    import robot_controller.controller as controller_module

    run = SimpleNamespace(quantity=3, completed=3 if outcome == "success" else 1,
                          close=Mock())
    run.run = Mock(return_value=outcome == "success")
    if outcome == "failure":
        run.run.side_effect = FeedbackFailure("tray acquisition exhausted")
    monkeypatch.setattr(controller_module, "AutoRunOperation", lambda *_args: run)
    node = SimpleNamespace(
        machine=SimpleNamespace(state="READY"), placement=None, events=Mock(),
        raise_if_cancelled=Mock(), _transition=Mock(), _end_operation=Mock(),
        _failure_outcome=RobotController._failure_outcome)

    def fail(goal, result, exc, code):
        result.outcome, result.message = code, str(exc)
        goal.abort()
        return result

    node._action_failure = fail
    goal = SimpleNamespace(request=request(3), succeed=Mock(), abort=Mock())
    result = RobotController._execute_auto_run_action(node, goal)
    assert result.completed_quantity == run.completed and result.requested_quantity == 3
    assert result.outcome == {"success": result.SUCCESS, "no_pick": result.NO_PICK,
                              "failure": result.FEEDBACK_FAILURE}[outcome]
    assert node.auto_run is None
    node._end_operation.assert_called_once()
    assert goal.succeed.called is (outcome != "failure")
    assert f"{run.completed}/3" in result.message


def test_final_home_suction_reappearance_faults_after_counted_placement():
    rig, run, bridge = queue_rig(quantity=1)
    rig.steps = iter([
        dict(outputs=0, inputs=0, currentCommandId=3),
        dict(outputs=0, inputs=0, currentCommandId=4),
        dict(outputs=0, inputs=1, currentCommandId=4),
    ])
    with pytest.raises(FeedbackFailure, match="Auto Run Home requires"):
        run.finish_home(bridge)
    assert run.completed == 1
    assert rig.node.machine.state == "HOMING"  # No false READY/SUCCESS.


def test_already_finished_placement_is_counted_before_a_ready_prefetch_handoff():
    rig, run, bridge = queue_rig()
    rig.emit(outputs=0, inputs=0, retract=True)
    pending = bridge.placement.pending_motion
    bridge.placement.pending_motion = None
    rig.transport.finish_batch(pending, handoff=lambda: True)
    assert bridge.placement.phase == "DONE"
    bridge.complete_idle()
    assert run.completed == 1


def test_completed_placement_is_reported_even_when_prefetch_fails(monkeypatch):
    run, node, order, _workers = cycle_rig(monkeypatch, 3, detection_error=True)

    def finish(_pending, **_kwargs):
        node.placement.phase = "DONE"

    node.hardware.finish_batch = finish
    with pytest.raises(FeedbackFailure, match="detector unavailable"):
        run.run()
    run.close()
    assert run.completed == 1
    assert order == ["Pick", "place queued", "prefetch"]
