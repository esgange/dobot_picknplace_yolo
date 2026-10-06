"""Controller-owned elapsed time uses synthetic clocks, feedback and GUI only."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import robot_controller.autorun as auto_module
from robot_controller.autorun import AutoRunOperation
from robot_controller.controller import RobotController
from robot_controller_interfaces.action import AutoRun
from test_auto_run import request
from test_joint_position import PositionRig
from test_operator_ui import status_node
from test_status_ui import status, window  # noqa: F401


def test_timer_includes_pause_retry_and_final_home_then_freezes_and_resets(monkeypatch):
    clock = [100.]
    monkeypatch.setattr(auto_module, "monotonic", lambda: clock[0])
    node, messages = status_node(PositionRig().monitor)
    run = node.auto_run = AutoRunOperation(node, request(3))
    assert run.elapsed_sec == 0.
    for state, elapsed in (("PICKING", 2.5), ("PAUSED", 25.), ("PICKING", 32.),
                           ("PLACING", 40.), ("HOMING", 43.5)):
        clock[0] = 100. + elapsed
        node.machine.state = state
        run.completed = 3 if state == "HOMING" else 1
        RobotController.publish_status(node)
        assert messages[-1].auto_run_active
        assert messages[-1].auto_run_elapsed_sec == elapsed
        assert messages[-1].auto_run_completed == run.completed
    assert run.finish_timer() == 43.5
    clock[0] = 1000.
    assert run.finish_timer() == run.elapsed_sec == 43.5
    next_run = node.auto_run = AutoRunOperation(node, request(2))
    RobotController.publish_status(node)
    assert next_run.elapsed_sec == messages[-1].auto_run_elapsed_sec == 0.
    assert messages[-1].auto_run_completed == 0 and messages[-1].auto_run_requested == 2


@pytest.mark.parametrize("headless", [False, True])
def test_status_retains_finished_duration_for_reconnecting_clients(headless):
    node, messages = status_node(PositionRig().monitor)
    node.headless = headless
    node.auto_run = None
    node.last_auto_run_result = AutoRun.Result(
        requested_quantity=3, completed_quantity=2, elapsed_sec=65.4)
    RobotController.publish_status(node)
    result = messages[-1]
    assert not result.auto_run_active
    assert (result.auto_run_requested, result.auto_run_completed, result.auto_run_elapsed_sec) == (
        3, 2, 65.4)
    node.last_auto_run_result = None  # A restarted controller has no prior run.
    RobotController.publish_status(node)
    assert messages[-1].auto_run_requested == messages[-1].auto_run_elapsed_sec == 0


def test_action_feedback_publishes_current_controller_duration():
    node = SimpleNamespace(
        active_action="pick", wait_for_resume=Mock(), candidate_index=1, candidate_total=3,
        machine=Mock(), events=Mock(), publish_operator_log=Mock(), publish_status=Mock(),
        active_goal=Mock(), auto_run=SimpleNamespace(quantity=3, completed=1, elapsed_sec=12.5))
    RobotController.operation_progress(node, "DETECT", "Retrying detection")
    feedback = node.active_goal.publish_feedback.call_args.args[0]
    assert isinstance(feedback, AutoRun.Feedback)
    assert feedback.elapsed_sec == 12.5
    assert (feedback.completed_quantity, feedback.requested_quantity) == (1, 3)


@pytest.mark.parametrize("active,state", [
    (True, "PLACING"), (True, "PAUSED"), (False, "READY"), (False, "FAULT")])
def test_gui_shows_live_or_frozen_controller_seconds(window, active, state):  # noqa: F811
    window.node.status = status(
        state=state, auto_run_active=active, auto_run_requested=3, auto_run_completed=2,
        auto_run_elapsed_sec=65.4)
    window._refresh()
    label = "Elapsed" if active else "Total"
    assert window.auto_progress.text() == f"2/3 completed · {label}: 65.4 s"
    window._refresh()
    assert window.auto_progress.text().endswith("65.4 s")  # No local drifting clock.
    window.node.status = None
    window._refresh()
    assert window.auto_progress.text() == "Auto Run status unavailable"
    window.node.status = status()
    window._refresh()
    assert window.auto_progress.text() == "0 completed · Elapsed: 0.0 s"
