"""Explicit Load prepares once; Stop and Preview cannot cause a delayed startup."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rclpy.task import Future

import robot_controller.controller as controller_module
from robot_controller.controller import RobotController
from robot_controller.errors import FeedbackFailure, HeldUnknown, OperationCanceled
from test_lifecycle_v2 import configuration_node
from test_status_ui import status, window  # noqa: F401


def request():
    return SimpleNamespace(item_teach_file='item.yaml', bin_teach_file='bin.yaml',
                           tray_teach_file='tray.yaml')


def response():
    return SimpleNamespace(success=False, message='', configuration_id='')


@pytest.mark.parametrize('initial', ['UNCONFIGURED', 'INACTIVE', 'READY'])
def test_load_runs_shared_startup_sequence_once_without_releasing_operation(monkeypatch, initial):
    node, _old, calls = configuration_node(initial)
    config = SimpleNamespace(configuration_id='loaded', validate_sources=Mock())
    monkeypatch.setattr(controller_module, 'load_configuration', lambda *_a, **_kw: config)
    phases = []
    transition = node._transition
    node._transition = lambda state, message: (phases.append(state), transition(state, message))
    result = RobotController._configure(node, request(), response())
    assert result.success and result.configuration_id == 'loaded'
    assert phases == ['INACTIVE', 'STARTING', 'READY']
    assert node.startup_complete and node.global_speed_percent == 100
    assert calls == [('begin', 'configure'), ('log', 'configuration_loaded'),
                     ('startup',), ('end',)]
    config.validate_sources.assert_called_once_with(node.root)


@pytest.mark.parametrize('stage', ['loading', 'installed', 'startup'])
def test_stop_during_load_or_preparation_never_enables_after_cancel_or_reports_ready(
        monkeypatch, stage):
    node, old, calls = configuration_node()
    config = SimpleNamespace(configuration_id='loaded', validate_sources=Mock())
    canceled = []

    def stop():
        canceled.append(True)
        node.machine.transition('STOPPING', 'Explicit Stop')

    def check_cancel():
        if canceled:
            raise OperationCanceled('Explicit Stop')

    def load(*_args, **_kwargs):
        if stage == 'loading':
            stop()
        return config

    node.raise_if_cancelled = check_cancel
    node._settle_lifecycle_cancellation = lambda _message: calls.append(('settle_stop',))
    if stage == 'installed':
        node._log_configuration = lambda _event: stop()
    if stage == 'startup':
        node.hardware.startup = lambda: (calls.append(('startup',)), stop())
    monkeypatch.setattr(controller_module, 'load_configuration', load)
    result = RobotController._configure(node, request(), response())
    assert not result.success and 'Explicit Stop' in result.message
    assert node.machine.state == 'STOPPING'
    assert calls.count(('begin', 'configure')) == calls.count(('end',)) == 1
    assert ('settle_stop',) in calls
    assert (('startup',) in calls) is (stage == 'startup')
    if stage == 'loading':
        assert node.configuration is old
    else:
        assert node.configuration is config and not node.startup_complete


@pytest.mark.parametrize('error,state', [
    (FeedbackFailure('Enable feedback unavailable'), 'FAULT'),
    (HeldUnknown('DI1 HIGH without trusted item'), 'HELD_UNKNOWN'),
])
def test_preparation_failure_retains_loaded_configuration_without_claiming_ready(
        monkeypatch, error, state):
    node, _old, _calls = configuration_node('UNCONFIGURED')
    config = SimpleNamespace(configuration_id='loaded', validate_sources=Mock())
    monkeypatch.setattr(controller_module, 'load_configuration', lambda *_a, **_kw: config)
    node.hardware.startup = Mock(side_effect=error)
    result = RobotController._configure(node, request(), response())
    assert not result.success and str(error) in result.message
    assert node.configuration is config and not node.startup_complete
    assert node.machine.state == state
    node.hardware.startup.assert_called_once()


def test_lifecycle_row_contains_only_recover_managed_control_and_stop(window):  # noqa: F811
    row = window.centralWidget().layout().itemAt(1).layout()
    assert row.count() == 3
    assert [row.itemAt(i).widget() for i in range(3)] == [
        window.recover, window.pause, window.stop]
    assert not hasattr(window, 'startup')


@pytest.mark.parametrize('held', [False, True])
def test_paused_continue_uses_same_control_or_its_menu_without_return_or_cancel(
        window, held):  # noqa: F811
    window.node.status = status(state='PAUSED', operation_active=True, operation='pick',
                                holding_item=held, can_return_item=held)
    sent = Mock(return_value=Future())
    window.node.service_clients['continue'] = SimpleNamespace(
        service_is_ready=lambda: True, call_async=sent)
    window.goal_handle = SimpleNamespace(cancel_goal_async=Mock())
    window._refresh()
    if held:
        assert window.pause.text() == 'RETURN ITEM' and window.pause.menu() is window.pause_menu
        window.managed_actions['continue'].trigger()
    else:
        assert window.pause.text() == 'CONTINUE' and window.pause.menu() is None
        window.pause.click()
    sent.assert_called_once()
    window.goal_handle.cancel_goal_async.assert_not_called()
    assert window.stop.isEnabled() and not window.pause.isEnabled()


def test_no_hidden_continue_after_failed_tray_acquisition(window):  # noqa: F811
    window.node.status = status(state='PAUSED', operation_active=True, operation='place',
                                phase='TRAY_ACQUISITION_PAUSED', holding_item=True,
                                can_return_item=True, tray_position_recorded=True)
    window._refresh()
    assert window.pause.text() == 'RETURN ITEM' and window.pause.menu() is None
    assert not window.managed_actions['continue'].isEnabled()
    assert window.place_item.isEnabled() and window.stop.isEnabled()


def test_continue_unavailable_still_allows_known_item_return(window):  # noqa: F811
    window.node.status = status(state='PAUSED', operation_active=True, operation='pick',
                                holding_item=True, can_return_item=True, can_continue=False,
                                continue_block_reason='Retained operation cannot resume')
    window._refresh()
    assert window.pause.text() == 'RETURN ITEM' and window.pause.isEnabled()
    assert not window.managed_actions['continue'].isEnabled()
    assert 'cannot resume' in window.availability_details.text()


def test_preview_blocks_load_preparation_even_when_called_directly(window):  # noqa: F811
    window.item_path.setText('item.yaml')
    window.preview_mode = True
    window._refresh()
    assert not window.configure.isEnabled()
    assert 'Turn Preview OFF' in window.configure.toolTip()
    window._configure()  # The fixture fails on any service dispatch.
    assert not window.pending


def test_gui_load_sends_only_configure_and_waits_for_ready_without_startup_client(
        window):  # noqa: F811
    window.node.service_clients.pop("startup", None)
    window.item_path.setText('item.yaml')
    window.node.status = status(state='UNCONFIGURED', configured=False, startup_complete=False)
    pending = Future()
    sent = Mock(return_value=pending)
    window.node.service_clients['configure'] = SimpleNamespace(
        service_is_ready=lambda: True, call_async=sent)
    window._configure()
    sent.assert_called_once()
    assert set(window.pending) == {'configure'}
    assert window.stop.isEnabled() and not window.home_button.isEnabled()
    assert not window.configure.isEnabled()
