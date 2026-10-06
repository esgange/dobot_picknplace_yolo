"""Failed Auto Run tray acquisition retains its owner and normal Pause choices."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import robot_controller.controller as controller_module
from robot_controller.autorun import AutoRunOperation
from robot_controller.controller import RobotController
from robot_controller.errors import FeedbackFailure, OperationCanceled
from robot_controller.operator_ui import presentation
from robot_controller_interfaces.srv import Command
from test_auto_run import request
import test_status_ui
from test_status_ui import acquisition_paused_status
from test_tray_acquisition_pause import acquisition_rig


window = test_status_ui.window


def paused_run(monkeypatch, replies):
    node, tray = acquisition_rig(monkeypatch, replies)
    run = node.auto_run = AutoRunOperation(node, request(3))
    run.completed = 2
    run._pick = Mock(return_value=True)
    node.candidates = SimpleNamespace(request=Mock())
    return node, tray, run


@pytest.mark.parametrize('reply', [None, 'timeout'])
@pytest.mark.parametrize('rounds', [1, 2])
def test_continue_retries_same_held_item_and_count_after_confirmed_pause(
        monkeypatch, reply, rounds):
    node, tray, run = paused_run(monkeypatch, [reply] * (3 * rounds) + ['valid'])
    session, outputs = node.managed.session, dict(node.expected_outputs)
    pauses = []
    node.hardware.move_batch = Mock(return_value=None)

    def continue_paused():
        pauses.append(node.placement)
        assert node.machine.state == 'PAUSED' and node.phase == 'TRAY_ACQUISITION_PAUSED'
        assert node.operation_lock.locked() and node.auto_run is run
        assert node.managed.session is session and node.holding_item
        assert node.expected_outputs == outputs and run.completed == 2
        assert tray.client.call_async.call_count == len(pauses) * 3
        assert run.prefetch is None
        node.candidates.request.assert_not_called()
        node.hardware.move_batch.assert_not_called()
        assert any(row[0] == 'stop_confirmed' for row in node.log)
        response = RobotController._continue(node, None, Command.Response())
        assert response.success
    node.on_wait = continue_paused

    def finish(bridge):
        # Geometry/execution are covered by transport tests; only this terminal
        # success, after the operator's retry, may increment production count.
        assert node.hardware.move_batch.call_count == 1
        assert node.placement is pauses[0] and bridge.placement is pauses[0]
        assert node.placement.tray_attempts.count == 1
        run.placement_completed()
    run.finish_home = finish
    assert run.run()
    assert len(pauses) == rounds and all(p is pauses[0] for p in pauses)
    assert run.completed == 3 and tray.client.call_async.call_count == 3 * rounds + 1
    run._pick.assert_called_once()
    assert node.managed.kind is None and node.managed.session is session
    run.close()


def test_return_from_auto_pause_finishes_ready_and_cancels_run_without_counting(monkeypatch):
    node, tray, run = paused_run(monkeypatch, [None] * 3)
    node._record_auto_run_result = lambda *args: RobotController._record_auto_run_result(
        node, *args)
    monkeypatch.setattr(controller_module, 'AutoRunOperation', lambda *_args: run)
    goal = SimpleNamespace(request=request(3), succeed=Mock(), abort=Mock())
    node._action_failure = Mock(side_effect=AssertionError('Return is not a controller fault'))

    def return_paused():
        assert node.managed.can_return_item()
        response = RobotController._return_item(node, None, Command.Response())
        assert response.success
    node.on_wait = return_paused
    node._managed_request = lambda kind, response: RobotController._managed_request(
        node, kind, response)
    result = RobotController._execute_auto_run_action(node, goal)
    assert result.outcome == result.CANCELED and result.final_state == 'READY'
    assert result.completed_quantity == 2 and result.requested_quantity == 3
    assert 'Return Item' in result.message
    assert result.elapsed_sec == run.elapsed_sec
    assert node.last_auto_run_result is result
    assert node.managed.session.attempts[0].state == 'RETURNED'
    assert not node.holding_item and not node.operation_lock.locked()
    assert node.auto_run is None and node.placement is None
    assert tray.client.call_async.call_count == 3
    assert not any(row[0] == 'move' and 'p2_pick' in row[1] for row in node.log)
    run._pick.assert_called_once()
    node.candidates.request.assert_not_called()
    goal.abort.assert_called_once()
    goal.succeed.assert_not_called()


@pytest.mark.parametrize('failure', ['stop', 'suction', 'position', 'outputs', 'source'])
def test_paused_auto_fault_or_stop_cannot_retry_or_start_another_pick(monkeypatch, failure):
    node, tray, run = paused_run(monkeypatch, [None] * 3)
    node.managed.continue_after_loss = Mock(
        side_effect=AssertionError('A paused failure cannot resume Auto Run'))

    def fail():
        if failure == 'stop':
            node.cancel_event.set()
        elif failure == 'suction':
            node.lose_suction()
        elif failure == 'position':
            node.feed['tool_vector_actual'][0] += 20.
        elif failure == 'outputs':
            node.feed['digital_outputs'] ^= 2
        else:
            node.configuration.validate_sources.side_effect = FeedbackFailure('source changed')
            response = RobotController._continue(node, None, Command.Response())
            assert not response.success
            node.cancel_event.set()
    node.on_wait = fail
    with pytest.raises((FeedbackFailure, OperationCanceled)):
        run.run()
    assert run.completed == 2 and tray.client.call_async.call_count == 3
    assert not any(row[0] in ('move', 'output', 'pulse') for row in node.log)
    run._pick.assert_called_once()
    node.candidates.request.assert_not_called()
    run.close()


@pytest.mark.parametrize('auto', [False, True])
def test_failure_pause_exposes_continue_return_and_stop_with_pending_action(window, auto):
    window.node.status = acquisition_paused_status(
        operation='auto_run' if auto else 'place', auto_run_active=auto,
        auto_run_requested=3, auto_run_completed=2)
    window.result_future = SimpleNamespace(done=lambda: False)
    window._refresh()
    assert presentation(window.node.status)[0] == 'PAUSED'
    assert window.pause.menu() is window.pause_menu
    assert window.managed_actions['continue'].isEnabled()
    assert window.managed_actions['return_item'].isEnabled()
    assert window.place_item.isEnabled() and window.stop.isEnabled()
    assert not window.pick_item.isEnabled() and not window.home_button.isEnabled()
    assert not window.auto_run_button.isEnabled()
    commands = []
    window._command = lambda name: commands.append(name) or True
    window.managed_actions['continue'].trigger()
    assert commands == ['continue']


def test_missing_tray_provider_blocks_auto_continue_but_allows_return(window):
    window.node.status = acquisition_paused_status(
        operation='auto_run', auto_run_active=True, tray_detector_ready=False,
        can_continue=False, continue_block_reason='Arm Tray Teach or start Tray Detect')
    window._refresh()
    assert not window.managed_actions['continue'].isEnabled()
    assert not window.place_item.isEnabled()
    assert window.managed_actions['return_item'].isEnabled() and window.stop.isEnabled()
