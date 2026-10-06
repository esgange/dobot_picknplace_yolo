"""Live global CP uses the controller owner and preserves confirmed values."""

from types import SimpleNamespace
from unittest.mock import Mock
import threading

from PyQt5 import QtWidgets
import pytest
from rclpy.task import Future

from robot_controller.controller import RobotController
from robot_controller.errors import (CommandRejected, CommandResponseTimeout,
                                     FeedbackFailure, HeldUnknown, OperationCanceled)
from robot_controller.hardware import DobotTransport
from robot_controller.state_machine import ControllerStateMachine
from robot_controller_interfaces.srv import SetGlobalCP
from test_status_ui import status, window  # noqa: F401
from test_transport_v2 import EventLog, snapshot


def cp_transport(*, held=False):
    transport = object.__new__(DobotTransport)
    transport.moving = False
    sample = snapshot(di1=held, outputs=(1 << 12) if held else 0)
    transport.node = SimpleNamespace(
        holding_item=held, expected_outputs={13: held}, global_cp_percent=100,
        publish_status=Mock())
    transport.monitor = SimpleNamespace(snapshot=lambda **_kw: sample)
    transport.call = Mock()
    return transport, sample


@pytest.mark.parametrize('percent', [0, 1, 37, 100])
@pytest.mark.parametrize('held', [False, True])
def test_live_cp_changes_only_global_blending_and_retains_outputs(percent, held):
    transport, sample = cp_transport(held=held)
    before = sample.feed['digital_outputs']
    transport.set_global_cp(percent)
    fields = {'r': percent, 'progress': transport._validate_held_snapshot if held else None}
    transport.call.assert_called_once_with('CP', **fields)
    assert transport.node.global_cp_percent == percent
    assert sample.feed['digital_outputs'] == before
    transport.node.publish_status.assert_called_once()


@pytest.mark.parametrize('percent', [-1, 101, 255, 0.5, True])
def test_invalid_cp_never_sends_a_command(percent):
    transport, _sample = cp_transport()
    with pytest.raises(CommandRejected, match='0 through 100'):
        transport.set_global_cp(percent)
    transport.call.assert_not_called()


@pytest.mark.parametrize('field,value', [
    ('isRunQueuedCmd', 1), ('RunningStatus', 1), ('EnableStatus', 0),
    ('ErrorStatus', 1), ('CollisionStates', 1), ('userCoordinate', 1), ('toolCoordinate', 1),
])
def test_cp_requires_idle_safe_feedback(field, value):
    transport, sample = cp_transport()
    sample.feed[field] = value
    with pytest.raises(FeedbackFailure):
        transport.set_global_cp(50)
    transport.call.assert_not_called()


def test_cp_rejects_an_active_motion_group_and_unknown_suction():
    transport, sample = cp_transport()
    transport.moving = True
    with pytest.raises(FeedbackFailure):
        transport.set_global_cp(50)
    transport.moving = False
    sample.suction_present = True
    sample.feed['digital_input_bits'] = 1
    with pytest.raises(HeldUnknown):
        transport.set_global_cp(50)
    transport.call.assert_not_called()


@pytest.mark.parametrize('error', [CommandRejected, CommandResponseTimeout, OperationCanceled])
def test_failed_cp_response_keeps_last_confirmed_value(error):
    transport, _sample = cp_transport()
    transport.node.global_cp_percent = 0
    transport.call.side_effect = error('test failure')
    with pytest.raises(error):
        transport.set_global_cp(50)
    assert transport.node.global_cp_percent == 0
    transport.node.publish_status.assert_not_called()


def test_accepted_cp_is_retained_even_if_the_following_output_check_fails():
    transport, sample = cp_transport(held=True)
    transport.call.side_effect = lambda *_args, **_kw: sample.feed.update(digital_outputs=0)
    with pytest.raises(HeldUnknown):
        transport.set_global_cp(0)
    assert transport.node.global_cp_percent == 0
    transport.node.publish_status.assert_called_once()


def cp_controller(state='READY'):
    node = SimpleNamespace(
        machine=ControllerStateMachine(initial=state), startup_complete=True,
        global_cp_percent=100, stop_guard=threading.RLock(),
        operation_lock=threading.Lock(), cancel_event=threading.Event(), events=EventLog(),
        hardware=SimpleNamespace(set_global_cp=Mock()))
    node._begin_operation = lambda name: RobotController._begin_operation(node, name)
    node._end_operation = node.operation_lock.release
    node._transition = node.machine.transition
    return node


@pytest.mark.parametrize('state', ['READY', 'HOLDING'])
def test_typed_service_applies_zero_and_reports_it(state):
    node = cp_controller(state)
    response = RobotController._set_global_cp(
        node, SetGlobalCP.Request(percent=0), SetGlobalCP.Response())
    assert response.success and response.confirmed_percent == node.global_cp_percent == 0
    node.hardware.set_global_cp.assert_called_once_with(0)
    assert node.machine.state == state and not node.operation_lock.locked()


@pytest.mark.parametrize('state', ['INACTIVE', 'PICKING', 'PLACING', 'PAUSED', 'FAULT', 'STOPPING'])
def test_cp_service_rejects_non_idle_states(state):
    node = cp_controller(state)
    response = RobotController._set_global_cp(
        node, SetGlobalCP.Request(percent=30), SetGlobalCP.Response())
    assert not response.success and node.machine.state == state
    node.hardware.set_global_cp.assert_not_called()


def test_cp_service_cannot_steal_an_auto_run_or_pending_operation():
    node = cp_controller()
    node.operation_lock.acquire()
    node.cancel_event.set()
    response = RobotController._set_global_cp(
        node, SetGlobalCP.Request(percent=0), SetGlobalCP.Response())
    assert not response.success and 'Another controller operation' in response.message
    assert node.operation_lock.locked() and node.cancel_event.is_set()
    node.hardware.set_global_cp.assert_not_called()


def test_invalid_service_value_does_not_fault_or_acquire_operation():
    node = cp_controller()
    node.global_cp_percent = None
    response = RobotController._set_global_cp(
        node, SetGlobalCP.Request(percent=101), SetGlobalCP.Response())
    assert not response.success and response.confirmed_percent == -1
    assert node.machine.state == 'READY' and not node.operation_lock.locked()
    node.hardware.set_global_cp.assert_not_called()


@pytest.mark.parametrize('stopping', [False, True])
def test_cp_failure_faults_without_overwriting_a_concurrent_stop(stopping):
    node = cp_controller()
    node.global_cp_percent = 0

    def failure(_percent):
        if stopping:
            node.machine.transition('STOPPING', 'Direct Stop')
        raise CommandResponseTimeout('CP unanswered')
    node.hardware.set_global_cp.side_effect = failure
    response = RobotController._set_global_cp(
        node, SetGlobalCP.Request(percent=30), SetGlobalCP.Response())
    assert not response.success and response.confirmed_percent == 0
    assert node.machine.state == ('STOPPING' if stopping else 'FAULT')
    assert not node.operation_lock.locked()


@pytest.mark.parametrize('selected', [None, 0, 43, 100])
def test_recovery_restores_selected_cp_including_zero(selected):
    transport, _sample = cp_transport()
    transport.node.global_cp_percent = selected
    transport.node.check_all_command_owners = Mock()
    transport.node.check_feedback_owners = Mock()
    transport.clients = {'CP': object()}
    for name in ('wait_services', 'ensure_no_pending_response', '_phase', 'request_stop',
                 '_acknowledge_stop', 'confirm_stop', '_clear_errors_if_needed', '_wait_enabled',
                 '_reset_outputs_if_unheld', '_confirm_ready'):
        setattr(transport, name, Mock())
    transport._call_startup = Mock()
    transport.recover(35)
    expected = 100 if selected is None else selected
    assert transport.node.global_cp_percent == expected
    assert transport._call_startup.call_args_list[-1].args == ('CP',)
    assert transport._call_startup.call_args_list[-1].kwargs == {'r': expected}
    assert transport.node.global_speed_percent == 35


def test_startup_settings_reset_cp_to_100():
    transport, _sample = cp_transport()
    transport.node.global_cp_percent = 0
    transport._call_startup = Mock()
    transport._apply_settings(100)
    transport._call_startup.assert_any_call('CP', r=100)
    assert transport.node.global_cp_percent == 100


def test_slider_sends_live_position_once_and_status_does_not_overwrite_edit(window):  # noqa: F811
    future = Future()
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=Mock(return_value=future))
    window.node.service_clients['cp'] = client
    window.node.status = status(global_cp_percent=100)
    window._refresh()
    assert (window.cp_slider.minimum(), window.cp_slider.maximum()) == (0, 100)
    window.cp_slider.setSliderDown(True)
    window.cp_slider.setSliderPosition(0)
    window._refresh()
    assert window.cp_slider.sliderPosition() == 0
    window.cp_slider.setSliderDown(False)  # Release sends exactly one request.
    window._refresh()
    assert window.cp_slider.sliderPosition() == 0 and window.cp_pending_percent == 0
    assert client.call_async.call_count == 1
    assert client.call_async.call_args.args[0].percent == 0
    future.set_result(SetGlobalCP.Response(success=True, confirmed_percent=0))
    window._refresh()
    assert '0% confirmed' in window.cp_label.text()  # Old status cannot erase the reply.
    window.node.status = status(global_cp_percent=0)
    window._refresh()
    assert window.cp_pending_percent is None and window.cp_label.text() == 'Global CP: 0%'
    window._cp()
    assert client.call_async.call_count == 1


def test_rejected_slider_restores_confirmed_zero(window, monkeypatch):  # noqa: F811
    monkeypatch.setattr(QtWidgets.QMessageBox, 'warning', Mock())
    future = Future()
    window.node.service_clients['cp'] = SimpleNamespace(
        service_is_ready=lambda: True, call_async=lambda _request: future)
    window.node.status = status(global_cp_percent=0)
    window.cp_slider.setValue(50)
    window._cp()
    future.set_result(SetGlobalCP.Response(
        success=False, confirmed_percent=0, message='Rejected'))
    window._refresh()
    assert window.cp_slider.value() == 0 and window.cp_pending_percent is None


def test_keyboard_edit_survives_status_refresh(window, monkeypatch):  # noqa: F811
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=Mock(return_value=Future()))
    window.node.service_clients['cp'] = client
    monkeypatch.setattr(window.cp_slider, 'hasFocus', lambda: True)
    window.cp_slider.setValue(21)
    window.cp_slider.setValue(22)
    window._refresh()
    assert window.cp_slider.value() == 22 and window.cp_debounce.isActive()
    client.call_async.assert_not_called()
    window.cp_debounce.timeout.emit()
    client.call_async.assert_called_once()
    assert client.call_async.call_args.args[0].percent == 22


def test_unknown_cp_and_missing_service_are_visible_without_dispatch(window):  # noqa: F811
    window.node.status = status(global_cp_percent=-1)
    window.node.service_clients['cp'] = SimpleNamespace(service_is_ready=lambda: False)
    window._refresh()
    assert window.cp_label.text() == 'Global CP: unknown'
    assert not window.cp_slider.isEnabled() and window.cp_slider.toolTip() == 'Service unavailable'
    window._cp()


@pytest.mark.parametrize('fields', [
    {'auto_run_active': True}, {'state': 'PICKING', 'operation_active': True},
    {'feedback_fresh': False}, {'state': 'PAUSED'}, {'state': 'FAULT'},
    {'startup_complete': False},
])
def test_cp_cannot_dispatch_when_operator_policy_blocks_it(window, fields):  # noqa: F811
    window.node.status = status(**fields)
    window.cp_slider.setValue(50)
    window._refresh()
    assert not window.cp_slider.isEnabled() and window.stop.isEnabled()
    window._cp()  # Fixture fails if any service command is sent.
