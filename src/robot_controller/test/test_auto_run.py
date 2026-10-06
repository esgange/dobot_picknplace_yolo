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
from robot_controller.errors import (
    CommandRejected, CommandResponseTimeout, FeedbackFailure, OperationCanceled)
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


@pytest.mark.parametrize("coalesced", [False, True])
def test_direct_pick_is_admitted_after_placement_without_home_or_old_di1_acquisition(coalesced):
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
    # Real command order: placement 1..3, next entry 4, pre-pick 5, pick 6.
    original = rig.transport.clients["MovL"].call_async

    def prepick_reply(command):
        future = original(command)
        future.result().robot_return = "{5}"
        return future
    rig.transport.clients["MovL"].call_async = prepick_reply
    rig.steps = iter([
        dict(outputs=RELEASE, inputs=OPEN, currentCommandId=2),
        dict(outputs=0, inputs=OPEN, currentCommandId=3),
        *([] if coalesced else [
            dict(outputs=1 << 13, inputs=OPEN, currentCommandId=4),
            dict(outputs=1 << 13, inputs=OPEN, currentCommandId=5)]),
        dict(outputs=(1 << 12) | (1 << 13), inputs=1, currentCommandId=6),
    ])
    acquired, _origin = rig.transport.move_batch(
        (plan[0], plan[2], plan[3]),
        stop_on_suction=True, pick_settling_sec=10., return_terminal_pose=True,
        confirmed_start_pose=bridge.origin, placement_bridge=bridge)
    assert acquired
    assert [name for name, _ in rig.requests] == [
        "MovL", "MovLIO", "MovLIO", "MovLIO", "MovL", "MovLIO", "Stop"]
    # No new feedback/arrival wait between placement admission and all next targets.
    assert rig.order[:6] == [name for name, _ in rig.requests[:6]]
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
    assert [name for name, _ in rig.requests] == ["MovL", "MovLIO", "MovLIO", "MovJ"]
    assert rig.order[:4] == ["MovL", "MovLIO", "MovLIO", "MovJ"]
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
    assert [name for name, _ in rig.requests].count("MovJ") == 1
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
                                auto_run_active=True, auto_run_requested=7, auto_run_completed=2,
                                auto_run_elapsed_sec=42.5)
    window.pending_goal = None
    window._refresh()
    assert all(window._availability().values())
    assert window.stop.isEnabled()
    assert not window.auto_quantity.isEnabled() and not window.place_x.isEnabled()
    assert window.auto_progress.text() == "2/7 completed · Elapsed: 42.5 s"
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


def cycle_rig(monkeypatch, quantity, *, slow=False, detection_error=False, save_images=False):
    from robot_controller.motion import Target
    from robot_controller.state_machine import ControllerStateMachine

    order, workers = [], []
    node = SimpleNamespace(
        configuration=object(), managed=SimpleNamespace(session=None, lock=threading.RLock()),
        machine=ControllerStateMachine(initial="READY"), placement=None,
        raise_if_cancelled=lambda: None, operation_progress=Mock(), events=Mock(),
        _preflight_item_state=Mock(),
        monitor=SimpleNamespace(snapshot=lambda **_k: object()))
    node._transition = lambda state, message: node.machine.transition(state, message)

    class Placement:
        def __init__(self, *_args, **kwargs):
            assert kwargs == {"require_held_item": True, "save_debug_images": save_images}
            self.phase = "OBSERVE"
            self.plan = (Target("retract", np.eye(4), 100, 100),)
            self.observing = True

        def run(self, owner, *, on_observed=None):
            assert owner.machine.state == "PLACING"
            order.append("tray acquisition")
            if on_observed is not None:
                on_observed()
            order.append("place queued")
            self.phase = "APPROACH"
            self.pending_motion = object()

        def close_pending(self):
            self.pending_motion = None

    class Worker:
        def __init__(self, *_args):
            assert _args[-1] is save_images
            assert node.machine.state == "PLACING" and node.placement.phase == "OBSERVE"
            assert order[-1] == "tray acquisition"
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
    goal = request(quantity)
    goal.save_debug_images = save_images
    run = AutoRunOperation(node, goal)

    def pick(batch, bridge):
        if batch is not None:
            assert batch == f"batch{len(workers)}" and workers[-1].closed
        if bridge is not None and not bridge.completed:
            assert bridge.placement.phase != "DONE"
            order.append("append next Pick")
            bridge.boundary_id = 4
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
        bridge.boundary_id = 4
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
@pytest.mark.parametrize("save_images", [False, True])
def test_counted_cycles_prefetch_only_when_needed_and_append_final_home(
        monkeypatch, quantity, slow, save_images):
    run, node, order, workers = cycle_rig(
        monkeypatch, quantity, slow=slow, save_images=save_images)
    assert run.run()
    assert run.completed == quantity
    assert order.count("place queued") == quantity
    assert order.count("prefetch") == quantity - 1
    assert order.count("append next Pick") == (0 if slow else quantity - 1)
    assert order[:4] == (["Pick", "tray acquisition", "prefetch", "place queued"]
                         if quantity > 1 else
                         ["Pick", "tray acquisition", "place queued", "append final Home"])
    assert order[-1] == "append final Home"
    assert node.machine.state == "READY"
    assert all(worker.closed for worker in workers)


def test_prefetch_error_ends_run_without_queuing_another_pick_or_counting_release(monkeypatch):
    run, _node, order, workers = cycle_rig(monkeypatch, 3, detection_error=True)
    with pytest.raises(FeedbackFailure, match="detector unavailable"):
        run.run()
    run.close()
    assert order == ["Pick", "tray acquisition", "prefetch", "place queued"]
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
    assert order == ["Pick", "tray acquisition", "prefetch", "place queued"]
    assert run.completed == 0 and workers[0].closed


def test_auto_pick_retries_empty_poses_once_and_preserves_completed_quantity(monkeypatch):
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
    assert len(node.requests) == 2
    assert run.completed == 2 and node.machine.state == "READY"


@pytest.mark.parametrize("counts,acquisitions", [
    ([1], [True]), ([0, 1], [True]), ([1, 1, 1], [False] * 3), ([0, 1, 0], [False]),
    ([0, 1, 1, 1], [False] * 3), ([1, 0, 1, 1], [False] * 3),
])
def test_auto_pick_detects_first_and_keeps_separate_acquisition_and_pick_budgets(
        monkeypatch, counts, acquisitions):
    from test_operation_retries import pick_rig

    node = pick_rig(monkeypatch, counts, acquisitions)
    node.managed.session = None
    node._plan_candidate_batch = lambda batch: RobotController._plan_candidate_batch(node, batch)
    run = AutoRunOperation(node, request(2))
    assert run._pick() is any(acquisitions)
    assert len(node.requests) == len(counts)
    assert node.log.index(("detect", 1)) < node.log.index(("ensure_home",))
    if not counts[0]:
        assert node.log.index(("ensure_home",)) < node.log.index(("detect", 2))


def test_empty_prefetch_finishes_queued_home_before_its_single_retry():
    rig, run, bridge = queue_rig(quantity=2)
    rig.node._execute_home = Mock(side_effect=AssertionError("Home is owned by the bridge"))
    rig.steps = iter([
        dict(outputs=RELEASE, inputs=OPEN, currentCommandId=2),
        dict(outputs=0, inputs=OPEN, currentCommandId=3),
        dict(outputs=0, inputs=OPEN, currentCommandId=4),
        dict(outputs=0, inputs=0, home=True),
    ])

    def detect(*_args, **_kwargs):
        assert run.completed == 1 and bridge.completed
        assert rig.monitor.snapshot().robot_enabled
        assert np.allclose(rig.transport.pose_from_snapshot(rig.monitor.snapshot()),
                           rig.node.configuration.home_matrix)
        assert [name for name, _ in rig.requests] == ["MovL", "MovLIO", "MovLIO", "MovJ"]
        return SimpleNamespace(identifier="retry-empty", candidates=[])
    rig.node.candidates = SimpleNamespace(request=Mock(side_effect=detect))
    assert not run._pick(SimpleNamespace(identifier="prefetched-empty", candidates=[]), bridge)
    rig.node.candidates.request.assert_called_once()
    assert run.completed == 1 and rig.node.machine.state == "READY"


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


def test_auto_and_place_away_from_tray_show_missing_provider(window):  # noqa: F811
    window.node.action_clients['auto_run'] = SimpleNamespace(server_is_ready=lambda: True)
    window.node.status = status(tray_position_recorded=True, at_tray_detect=False,
                                tray_detector_ready=False)
    window._refresh()
    assert window.pick_item.isEnabled()
    assert not window.auto_run_button.isEnabled() and not window.place_item.isEnabled()
    assert 'Auto Run: Arm Tray Teach' in window.availability_details.text()
    assert 'Place Item: Arm Tray Teach' in window.availability_details.text()
    window.node.status.tray_detector_ready = True
    window._refresh()
    assert window.auto_run_button.isEnabled() and window.place_item.isEnabled()
    assert 'Tray Detect position' not in window.availability_details.text()


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


@pytest.mark.parametrize("outcome", ["success", "no_pick", "failure", "stop"])
def test_native_auto_action_reports_quantity_and_releases_its_one_owner(monkeypatch, outcome):
    import robot_controller.controller as controller_module

    run = SimpleNamespace(quantity=3, completed=3 if outcome == "success" else 1,
                          close=Mock(), finish_timer=Mock(return_value=45.6))
    run.run = Mock(return_value=outcome == "success")
    if outcome == "failure":
        run.run.side_effect = FeedbackFailure("tray acquisition exhausted")
    elif outcome == "stop":
        run.run.side_effect = OperationCanceled("Operator Stop")
    monkeypatch.setattr(controller_module, "AutoRunOperation", lambda *_args: run)
    node = SimpleNamespace(
        machine=SimpleNamespace(state="READY"), placement=None, events=Mock(),
        raise_if_cancelled=Mock(), _transition=Mock(), _end_operation=Mock(),
        _failure_outcome=RobotController._failure_outcome)
    node._record_auto_run_result = lambda *args: RobotController._record_auto_run_result(
        node, *args)

    def fail(goal, result, exc, code):
        result.outcome = result.CANCELED if isinstance(exc, OperationCanceled) else code
        result.message = str(exc)
        goal.abort()
        return result

    node._action_failure = fail
    goal = SimpleNamespace(request=request(3), succeed=Mock(), abort=Mock())
    result = RobotController._execute_auto_run_action(node, goal)
    assert result.completed_quantity == run.completed and result.requested_quantity == 3
    assert result.outcome == {"success": result.SUCCESS, "no_pick": result.NO_PICK,
                              "failure": result.FEEDBACK_FAILURE, "stop": result.CANCELED}[outcome]
    assert node.auto_run is None
    node._end_operation.assert_called_once()
    assert goal.succeed.called is (outcome in ("success", "no_pick"))
    assert f"{run.completed}/3" in result.message
    assert result.elapsed_sec == 45.6 and "elapsed 45.6 s" in result.message
    assert node.last_auto_run_result is result
    assert node.events.record.call_args.kwargs["elapsed_sec"] == 45.6
    run.finish_timer.assert_called_once()


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
    assert order == ["Pick", "tray acquisition", "prefetch", "place queued"]


@pytest.mark.parametrize("outcome", ["no_pick", "stop", "arrival_failed"])
def test_prefetch_requires_successful_pick_and_confirmed_tray_arrival(monkeypatch, outcome):
    run, node, order, workers = cycle_rig(monkeypatch, 2)
    run._pick = Mock(return_value=False)
    error = {"stop": OperationCanceled, "arrival_failed": FeedbackFailure}.get(outcome)
    if error:
        run._pick.side_effect = error("Tray Detect not confirmed")
        with pytest.raises(error):
            run.run()
    else:
        assert not run.run()
    run.close()
    assert not order and not workers and node.placement is None


@pytest.mark.parametrize("interruption", ["tray_exhausted", "stop"])
def test_tray_acquisition_failure_never_starts_next_item_request(monkeypatch, interruption):
    from robot_controller.errors import HeldSuctionLost
    from test_tray_acquisition_pause import acquisition_rig

    node, tray = acquisition_rig(monkeypatch, [None] * 3)
    run = node.auto_run = AutoRunOperation(node, request(2))
    run._pick = Mock(return_value=True)  # Successful Pick's confirmed Tray Detect endpoint.
    original_session = node.managed.session
    node.candidates = SimpleNamespace(request=Mock())
    errors = {"tray_exhausted": OperationCanceled,
              "stop": OperationCanceled, "held_loss": HeldSuctionLost}

    def tray_reply(_future):
        assert run.prefetch is None
        node.candidates.request.assert_not_called()
        assert node.managed.session is original_session and node.holding_item
        assert not any(row[0] in ("move", "output", "pulse") for row in node.log)
        if interruption != "tray_exhausted":
            raise errors[interruption](interruption)

    tray.on_send = tray_reply

    def stop_when_paused():
        assert node.machine.state == "PAUSED" and node.placement.acquisition_paused
        assert tray.client.call_async.call_count == 3
        node.cancel_event.set()
    node.on_wait = stop_when_paused
    try:
        with pytest.raises(errors[interruption]):
            run.run()
        node.candidates.request.assert_not_called()
        assert run.completed == 0
        assert tray.client.call_async.call_count == (3 if interruption == "tray_exhausted" else 1)
        assert node.managed.session is original_session and node.holding_item
        run._pick.assert_called_once()
    finally:
        run.close()
    assert run.prefetch is None


@pytest.mark.parametrize("reply", ["accepted", "rejected", "stop"])
@pytest.mark.parametrize("pending_index", [1, 2, 3])
def test_early_item_result_cannot_append_motion_until_every_placement_ack(reply, pending_index):
    rig = QueueRig()
    node = rig.node
    run = node.auto_run = AutoRunOperation(node, request(2))
    batch = object()
    pending = Future()
    canceled = [False]
    entered = threading.Event()
    rig.order.clear()
    for name in ("MovL", "MovLIO"):
        original_call = rig.transport.clients[name].call_async

        def dispatch(command, original=original_call):
            assert entered.wait(2.)  # Worker is active before the first motion reply.
            response = original(command)
            return pending if len(rig.requests) == pending_index else response

        rig.transport.clients[name].call_async = dispatch
    node.cancel_requested = lambda: canceled[0]

    def awaiting_reply(_seconds):
        assert len(rig.requests) == pending_index and not pending.done()
        assert run.prefetch.future.result(timeout=2.) is batch
        assert run._pick.call_count == 1  # Ready poses cannot enter the next Pick yet.
        node.candidates.request.assert_called_once()
        if reply == "stop":
            canceled[0] = True
        else:
            rig.order.append("placement accepted" if reply == "accepted" else "placement rejected")
            pending.set_result(SimpleNamespace(
                res=0 if reply == "accepted" else -1, robot_return="{4}"))

    node.wait_control = awaiting_reply

    def acquire(configuration, *, save_debug_images, cancel):
        assert configuration is node.configuration and not save_debug_images and not cancel()
        assert not rig.requests and node.placement.phase != "DONE"
        assert run.completed == 0
        rig.order.append("next item request")
        entered.set()
        return batch

    node.candidates = SimpleNamespace(request=Mock(side_effect=acquire))
    # Wait only for the detector thread; the real transport still supervises the
    # accepted placement and hands off without waiting for physical arrival.
    finish = rig.transport.finish_batch

    def finish_placement(motion, *, handoff):
        assert [name for name, _ in rig.requests] == ["MovL", "MovLIO", "MovLIO"]
        assert pending.done() and pending.result().res == 0
        assert run.prefetch.future.result(timeout=2.) is batch
        rig.transport.finish_batch = finish
        return finish(motion, handoff=handoff)

    rig.transport.finish_batch = finish_placement

    def next_pick(found, bridge):
        if found is None:
            return True  # The first Pick has confirmed Tray Detect.
        assert found is batch and bridge is run.bridge
        assert not bridge.completed and bridge.placement.phase != "DONE"
        assert node.managed.session is bridge.old_session
        assert [name for name, _ in rig.requests] == ["MovL", "MovLIO", "MovLIO"]
        assert "placement accepted" in rig.order
        # Append actual next Pick calls before placement arrival; no Home.
        item = np.eye(4)
        item[:3, 3] = [.4, .1, .25]
        plan = pick_targets(node.configuration.home_matrix, item, node.configuration.profile, 1)
        bridge.next_session = PickSession(["next-item"], [plan])
        rig.steps = iter([
            dict(outputs=RELEASE, inputs=OPEN, currentCommandId=2),
            dict(outputs=0, inputs=OPEN, currentCommandId=3),
            dict(outputs=0, inputs=OPEN, currentCommandId=4),
            dict(outputs=(1 << 12) | (1 << 13), inputs=1, currentCommandId=7),
        ])
        node.hardware.move_batch(
            (plan[0], plan[2], plan[3]), stop_on_suction=True,
            pick_settling_sec=10., return_terminal_pose=True,
            confirmed_start_pose=bridge.origin, placement_bridge=bridge)
        raise OperationCanceled("test reached next Pick handoff")

    run._pick = Mock(side_effect=next_pick)
    try:
        with pytest.raises(CommandRejected if reply == "rejected" else OperationCanceled):
            run.run()
        names = [name for name, _ in rig.requests]
        if reply == "accepted":
            assert names == ["MovL", "MovLIO", "MovLIO", "MovLIO", "MovL", "MovLIO",
                             "Stop"]
            entry_dispatch = [i for i, name in enumerate(rig.order) if name == "MovLIO"][2]
            assert rig.order.index("placement accepted") < entry_dispatch
            assert rig.order.index("feedback") > entry_dispatch
            assert run.completed == 1
        else:
            assert names[:pending_index] == ["MovL", "MovLIO", "MovLIO"][:pending_index]
            assert names[pending_index:] and set(names[pending_index:]) == {"Stop"}
            assert "feedback" not in rig.order and run.completed == 0
        node.candidates.request.assert_called_once()
        assert run._pick.call_count == (2 if reply == "accepted" else 1)
    finally:
        if not pending.done():
            pending.set_result(SimpleNamespace(res=0))
        run.close()


def test_stop_after_placement_admission_discards_prefetch_without_next_pick(monkeypatch):
    run, node, order, workers = cycle_rig(monkeypatch, 2)
    original = auto_module.PlacementOperation.run

    def place_then_stop(placement, owner, **kwargs):
        original(placement, owner, **kwargs)
        owner.raise_if_cancelled = Mock(side_effect=OperationCanceled("operator Stop"))

    monkeypatch.setattr(auto_module.PlacementOperation, "run", place_then_stop)
    try:
        with pytest.raises(OperationCanceled):
            run.run()
        assert order == ["Pick", "tray acquisition", "prefetch", "place queued"]
        assert len(workers) == 1 and run.prefetch is workers[0]
        assert node.placement.pending_motion is not None
    finally:
        run.close()
    assert node.placement.pending_motion is None
    assert run.prefetch is None and workers[0].closed


@pytest.mark.parametrize("interruption", ["rejected", "timeout", "stop"])
def test_failed_placement_admission_discards_inflight_item_request(monkeypatch, interruption):
    rig = QueueRig()
    node = rig.node
    run = node.auto_run = AutoRunOperation(node, request(2))
    run._pick = Mock(return_value=True)
    entered, released = threading.Event(), threading.Event()
    pending = Future()
    canceled = [False]
    node.cancel_requested = lambda: canceled[0]
    old_session = node.managed.session

    def acquire(_configuration, *, save_debug_images, cancel):
        entered.set()
        assert released.wait(2.)
        assert run.prefetch.cancel.wait(2.) and cancel()
        return object()  # This late observation must be discarded.

    node.candidates = SimpleNamespace(request=Mock(side_effect=acquire))
    original = rig.transport.clients["MovLIO"].call_async

    def dispatch(command):
        response = original(command)
        if len(rig.requests) != 3:
            return response
        assert entered.wait(2.)
        if interruption == "rejected":
            pending.set_result(SimpleNamespace(res=-1))
        elif interruption == "stop":
            canceled[0] = True
        return pending

    rig.transport.clients["MovLIO"].call_async = dispatch
    if interruption == "timeout":
        monkeypatch.setattr("robot_controller.hardware.COMMAND_RESPONSE_TIMEOUT_SEC", 0.)
    error = {"rejected": CommandRejected, "timeout": CommandResponseTimeout,
             "stop": OperationCanceled}[interruption]
    try:
        with pytest.raises(error):
            run.run()
        worker = run.prefetch
        assert worker is not None and not worker.future.done()
        assert run.bridge is None and run.completed == 0
        assert node.managed.session is old_session and old_session.held_index == 1
        run._pick.assert_called_once()
        names = [name for name, _ in rig.requests]
        assert names[:3] == ["MovL", "MovLIO", "MovLIO"]
        assert names[3:] and set(names[3:]) == {"Stop"}
    finally:
        if not pending.done():
            pending.set_result(SimpleNamespace(res=0))
        released.set()
        run.close()
    assert run.prefetch is None and not worker.thread.is_alive()
    with pytest.raises(OperationCanceled):
        worker.future.result()


def test_stop_after_tray_reply_before_worker_start_prevents_request():
    rig = QueueRig()
    node = rig.node
    run = node.auto_run = AutoRunOperation(node, request(2))
    run._pick = Mock(return_value=True)
    node.candidates = SimpleNamespace(request=Mock())

    def observe(*_args, **_kwargs):
        node.raise_if_cancelled = Mock(side_effect=OperationCanceled("operator Stop"))
        return np.array([.3, .2, .25])

    node.trays.request.side_effect = observe
    try:
        with pytest.raises(OperationCanceled):
            run.run()
        assert run.prefetch is None and not rig.requests
        node.candidates.request.assert_not_called()
    finally:
        run.close()


@pytest.mark.parametrize("interruption", ["stop"])
def test_executing_placement_failure_discards_inflight_next_item_result(monkeypatch, interruption):
    from robot_controller.errors import HeldSuctionLost

    run, node, order, _workers = cycle_rig(monkeypatch, 2)
    entered, released = threading.Event(), threading.Event()
    monkeypatch.setattr(auto_module, "CandidatePrefetch", CandidatePrefetch)
    node.cancel_requested = lambda: False

    def acquire(_configuration, *, save_debug_images, cancel):
        order.append("prefetch")
        entered.set()
        assert released.wait(2.)
        assert run.prefetch.cancel.wait(2.) and cancel()
        return object()  # A late reply must never become another Pick.

    node.candidates = SimpleNamespace(request=Mock(side_effect=acquire))
    error = OperationCanceled if interruption == "stop" else HeldSuctionLost

    def fail(_pending, **_kwargs):
        assert entered.wait(2.)
        assert not run.prefetch.future.done()
        raise error(interruption)

    node.hardware.finish_batch = fail
    try:
        with pytest.raises(error):
            run.run()
        worker = run.prefetch
        assert run.completed == 0
        node.candidates.request.assert_called_once()
        assert order[:2] == ["Pick", "tray acquisition"]
        assert sorted(order[2:]) == ["place queued", "prefetch"]
    finally:
        released.set()
        run.close()
    assert run.prefetch is None and not worker.thread.is_alive()
    with pytest.raises(OperationCanceled):
        worker.future.result()


@pytest.mark.parametrize("stop_after", [1, 2, 3])
def test_stop_during_direct_next_pick_admission_keeps_old_source(stop_after):
    rig, run, bridge = queue_rig()
    old = bridge.old_session
    pose = np.eye(4)
    pose[:3, 3] = [.4, .1, .25]
    plan = pick_targets(rig.node.configuration.home_matrix, pose,
                        rig.node.configuration.profile, 1)
    bridge.next_session = PickSession(["next-item"], [plan])
    bridge.placement.close_pending()
    canceled = [False]
    rig.on_request = lambda index: canceled.__setitem__(0, index >= 3 + stop_after)
    rig.node.cancel_requested = lambda: canceled[0]

    def check_cancel():
        if canceled[0]:
            raise OperationCanceled("operator Stop")
    rig.node.raise_if_cancelled = check_cancel
    with pytest.raises(OperationCanceled):
        rig.transport.move_batch((plan[0], plan[2], plan[3]), stop_on_suction=True,
                                 confirmed_start_pose=bridge.origin, placement_bridge=bridge)
    names = [name for name, _ in rig.requests[3:]]
    assert names[:stop_after] == ["MovLIO", "MovL", "MovLIO"][:stop_after]
    assert names[stop_after:] and set(names[stop_after:]) == {"Stop"}
    assert rig.node.managed.session is old and old.held_index == 1
    assert run.completed == 0 and not bridge.completed
    run.close()
    assert bridge.next_session.attempts[0].state == "CANCELED"


def test_direct_pick_handoff_waits_for_prepick_id_and_neutral_release_history():
    rig, run, bridge = queue_rig()
    plan = pick_targets(rig.node.configuration.home_matrix, np.eye(4),
                        rig.node.configuration.profile, 1)
    bridge.next_session = PickSession(["next-item"], [plan])
    # The entry's acceptance-only reply must not be parsed as an ID.
    bridge.accepted(0, SimpleNamespace(res=0))
    assert bridge.boundary_id is None
    bridge.accepted(1, SimpleNamespace(res=0, robot_return="{5}"))
    bridge.admitted(plan[0])
    bridge.observe(rig.emit(outputs=0, inputs=0, currentCommandId=3))
    bridge.observe(rig.emit(outputs=1 << 13, inputs=OPEN, currentCommandId=4))
    assert run.completed == 0 and rig.node.managed.session is bridge.old_session
    # Complete from the new-pick sample itself, without a separate idle sample.
    rig.node.managed.observe_continuous(rig.emit(
        outputs=(1 << 12) | (1 << 13), inputs=1, currentCommandId=6))
    assert run.completed == 1 and rig.node.managed.session is bridge.next_session
    assert bridge.next_session.attempts[0].state == "ACTIVE"
    bridge.observe(rig.monitor.snapshot())
    assert run.completed == 1
    bridge.placement.close_pending()


def test_late_execution_feedback_cannot_transfer_source_after_direct_stop():
    rig, run, bridge = queue_rig()
    bridge.accepted(0, SimpleNamespace(res=0, robot_return="{4}"))
    rig.node.cancel_requested = lambda: True
    old = bridge.old_session
    rig.node.managed.observe_continuous(rig.emit(outputs=0, inputs=0, currentCommandId=4))
    assert not bridge.completed and run.completed == 0
    assert rig.node.managed.session is old and old.held_index == 1
    bridge.placement.close_pending()


def test_latched_old_item_drop_cannot_be_counted_as_a_placement_at_handoff():
    from robot_controller.errors import HeldSuctionLost
    rig, run, bridge = queue_rig()
    old = bridge.old_session
    rig.node.managed.held_loss_pending = True
    old.set_state(old.held_index, "DROPPED")
    bridge.accepted(0, SimpleNamespace(res=0, robot_return="{4}"))
    with pytest.raises(HeldSuctionLost):
        bridge.observe(rig.emit(outputs=0, inputs=0, currentCommandId=4))
    assert not bridge.completed and run.completed == 0
    assert rig.node.managed.session is old and old.held_index == 1
    bridge.placement.close_pending()
