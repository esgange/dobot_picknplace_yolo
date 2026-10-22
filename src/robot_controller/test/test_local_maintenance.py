"""Local preparation and ownership integration using synthetic feedback only."""

from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from robot_controller.local_preparation import LocalPreparation
from test_hmi_modes import modes, ready  # noqa: F401
from test_status_ui import status, window  # noqa: F401


PATHS = ('/offline/item.yaml', '/offline/bin.yaml', '/offline/tray.yaml')


def accepted(configuration='selected'):
    return SimpleNamespace(success=True, configuration_id=configuration, message='Prepared')


def test_one_selection_one_request_requires_matching_fresh_enabled_ready(tmp_path):
    prep = LocalPreparation(tmp_path)
    paths = list(PATHS)
    prep.request(paths)
    paths[0] = '/another/item.yaml'
    send = Mock(return_value=True)
    prep.poll(['Cameras starting'], status(), send)
    send.assert_not_called()
    prep.poll([], status(), send)
    for _ in range(3):
        prep.poll([], status(), send)
    send.assert_called_once_with(PATHS)
    assert prep.state == 'CONFIGURING'
    prep.response(accepted())
    for fields in ({'configuration_id': 'wrong'}, {'robot_enabled': False},
                   {'startup_complete': False}, {'state': 'INACTIVE'}):
        values = {'configuration_id': 'selected', **fields}
        prep.poll([], status(**values), send)
        assert prep.state == 'CONFIRMING'
    prep.poll(['Robot feedback: stale'], status(configuration_id='selected'), send)
    assert prep.state == 'CONFIRMING'
    prep.poll([], status(configuration_id='selected'), send)
    assert prep.state == 'READY' and not prep.busy
    send.assert_called_once()


@pytest.mark.parametrize('phase', ['WAITING', 'CONFIGURING', 'CONFIRMING'])
def test_stop_never_restarts_preparation_and_contains_late_acceptance(tmp_path, phase):
    prep = LocalPreparation(tmp_path)
    prep.request(PATHS)
    send = Mock(return_value=True)
    if phase != 'WAITING':
        prep.poll([], status(), send)
    if phase == 'CONFIRMING':
        prep.response(accepted())
    prep.cancel()
    assert prep.stop_needed is (phase != 'WAITING')
    prep.stop_needed = False
    if phase == 'CONFIGURING':
        prep.response(accepted())
        assert prep.stop_needed  # A second containment after the late reply.
    prep.poll([], status(configuration_id='selected'), send)
    assert prep.state == 'CANCELED'
    assert send.call_count == (0 if phase == 'WAITING' else 1)


@pytest.mark.parametrize('phase', ['WAITING', 'CONFIGURING', 'CONFIRMING'])
def test_timeout_does_not_retry_or_accept_late_ready(tmp_path, phase):
    prep = LocalPreparation(tmp_path)
    prep.request(PATHS)
    send = Mock(return_value=True)
    if phase != 'WAITING':
        prep.poll([], status(), send)
    if phase == 'CONFIRMING':
        prep.response(accepted())
    prep.deadline = -1.
    prep.poll(['health unavailable'], status(), send)
    assert prep.state == 'FAILED'
    assert prep.stop_needed is (phase != 'WAITING')
    if phase == 'CONFIGURING':
        with pytest.raises(ValueError, match='previous local preparation'):
            prep.request(PATHS)
        prep.response(accepted())
        assert prep.stop_needed
    prep.poll([], status(configuration_id='selected'), send)
    assert prep.state == 'FAILED'


@pytest.mark.parametrize('result,error', [
    (None, ''), (None, 'transport failed'),
    (SimpleNamespace(success=False, message='Source changed'), ''),
    (accepted(''), ''),
])
def test_rejected_or_missing_configuration_never_becomes_ready(tmp_path, result, error):
    prep = LocalPreparation(tmp_path)
    prep.request(PATHS)
    prep.poll([], status(), Mock(return_value=True))
    prep.response(result, error=error)
    assert prep.state == 'FAILED' and prep.stop_needed


def test_empty_selection_fails_without_any_configure(tmp_path):
    prep = LocalPreparation(tmp_path)
    prep.request(('', '', ''))
    send = Mock()
    prep.poll([], status(), send)
    assert prep.state == 'FAILED' and not prep.stop_needed
    send.assert_not_called()


@pytest.fixture
def local(window):  # noqa: F811
    window.modes.managed = True
    window.modes.state = 'AUTO'
    window.modes.core.snapshot = Mock(return_value=('RUNNING', ''))
    window.station_startup.state = 'RUNNING'
    window.station_startup.mode = 'AUTO'
    window.node.get_node_names_and_namespaces = lambda: [('robot_controller', '/')]
    window.hmi_graph_at = -float('inf')
    window._health_rows = Mock(return_value=[])
    window._clear_preview = Mock()
    for field, path in zip((window.item_path, window.bin_path, window.tray_path), PATHS):
        field.setText(path)
    future = Future()
    window.node.service_clients['configure'] = SimpleNamespace(
        service_is_ready=lambda: True, call_async=Mock(return_value=future))
    window.node.service_clients['stop'] = SimpleNamespace(
        service_is_ready=lambda: True, call_async=Mock(return_value=Future()))
    yield window, future
    window.local_preparation.cancel()
    window.local_preparation.awaiting_reply = False


def test_local_click_loads_selected_files_once_and_retains_teaching_and_motion(local):
    view, future = local
    assert [b.text() for b in view.mode_buttons.buttons()] == ['Remote Auto', 'Local / Maintenance']
    view._set_mode('AUTO')
    view._poll_local_preparation()
    request = view.node.service_clients['configure'].call_async.call_args.args[0]
    assert (request.item_teach_file, request.bin_teach_file, request.tray_teach_file) == PATHS
    view._poll_local_preparation()
    view.node.service_clients['configure'].call_async.assert_called_once()
    view.node.status = status(configuration_id='selected', tray_position_recorded=True)
    future.set_result(accepted())
    view._collect()
    view._poll_local_preparation()
    view._refresh_controls()
    view._refresh_hmi()
    assert view.local_preparation.state == 'READY'
    assert view.home_button.isEnabled() and view.pick_item.isEnabled()
    assert view.tool_controls['item'][0].isEnabled()
    assert view.tool_controls['tray'][0].isEnabled()
    for field, name in ((view.item_path, 'item.yaml'), (view.bin_path, 'bin.yaml')):
        path = view.node.root / name
        path.write_text('synthetic selection')
        field.setText(str(path))
    view.tools.start = Mock()
    view._tool_clicked('item')
    view.tools.start.call_args.args[1]()
    assert view.modes.state == 'AUTO'


@pytest.mark.parametrize('fields', [{'feedback_fresh': False}, {'holding_item': True},
                                   {'digital_input_bits': 1}, {'operation_active': True},
                                   {'state': 'RECOVERY_REQUIRED'}, {'robot_queue_active': True}])
def test_local_preparation_waits_without_commands_for_unsafe_feedback(local, fields):
    view, _future = local
    view.local_preparation.request(PATHS)
    view.node.status = status(**fields)
    view._poll_local_preparation()
    view.node.service_clients['configure'].call_async.assert_not_called()
    view.node.service_clients['stop'].call_async.assert_not_called()


def test_local_stop_contains_late_success_without_restarting_or_claiming_ready(local):
    view, future = local
    view._set_mode('AUTO')
    view._poll_local_preparation()
    view._immediate_stop()
    assert view.local_preparation.state == 'CANCELED'
    view.pending['stop'].set_result(SimpleNamespace(success=True))
    view._collect()
    future.set_result(accepted())
    view._collect()
    view._poll_local_preparation()
    view._collect()
    assert view.node.service_clients['stop'].call_async.call_count == 2
    assert view.local_preparation.state == 'CANCELED'
    view.node.service_clients['configure'].call_async.assert_called_once()


@pytest.mark.parametrize('key', ['motion', 'gripper', 'calibration'])
def test_direct_tool_open_waits_for_owned_core_and_graph_release(window, modes, key):  # noqa: F811
    ready(modes)
    modes.node = window.node
    window.node.nodes = [('robot_controller', '/')]
    window.node.get_node_names_and_namespaces = lambda: window.node.nodes
    window.node.clear_controller_status = lambda: setattr(window.node, 'status', None)
    window.modes = modes
    window.station_startup.state = 'RUNNING'
    window.station_startup.mode = 'AUTO'
    window.tools.start = Mock()
    window._tool_clicked(key)
    assert window.pending_tool == key
    window.tools.start.assert_not_called()
    modes.poll()
    assert modes.sequence == ['stop detectors', 'stop core']
    modes.core.state = 'OFF'
    modes.poll()
    window._poll_tool_handoff()
    window.tools.start.assert_not_called()  # Graph still owns the command identity.
    window.node.nodes = []
    modes.poll()
    window._poll_tool_handoff()
    assert window.tools.start.call_args.args[0] == key
    window.tools.start.call_args.args[1]()
    assert window.station_startup.mode == 'AUTO'  # Still the same local station mode.
    assert window.local_preparation.state == 'OFF'  # Closing a tool never re-enables.


def test_remote_waits_for_startup_camera_status_without_early_alarm(window):  # noqa: F811
    window.modes.state = 'REMOTE_AUTO'
    window._health_rows = Mock(return_value=[])
    window.node.status = status(state='UNCONFIGURED', configured=False,
                                message='CAMERA ALARM: starting')
    window.auto_detectors.start_runtime_nodes = Mock()
    window._configure_paths = Mock()
    window._request_remote_health()
    window._poll_remote_health()
    assert window.station_startup.checking and not window.station_startup.alarm
    window.auto_detectors.start_runtime_nodes.assert_not_called()
    window.node.status.message = 'No configuration loaded'
    window._poll_remote_health()
    window._poll_remote_health()
    window.auto_detectors.start_runtime_nodes.assert_called_once()
    window._configure_paths.assert_not_called()


def test_explicit_local_click_initializes_and_waits_without_loading_at_gui_open(window):  # noqa: F811
    view = window
    view.modes.managed = True
    view.station_startup.managed = True
    view.station_startup.state = 'OFF'
    view.station_startup.start = Mock()
    view.node.status = None
    view._configure_paths = Mock()
    for field, path in zip((view.item_path, view.bin_path, view.tray_path), PATHS):
        field.setText(path)
    assert view.local_preparation.state == 'OFF'
    view._set_mode('AUTO')
    view.station_startup.start.assert_called_once()
    assert view.station_startup.mode == 'AUTO'
    assert view.local_preparation.paths == PATHS
    view._poll_local_preparation()
    view._configure_paths.assert_not_called()
    view.local_preparation.cancel()


def test_refused_direct_tool_handoff_clears_request_without_launch(window, modes):  # noqa: F811
    ready(modes)
    modes.node = window.node
    window.node.get_node_names_and_namespaces = lambda: [('robot_controller', '/')]
    window.modes = modes
    window.station_startup.state = 'RUNNING'
    window.station_startup.mode = 'AUTO'
    window.tools.start = Mock()
    window._tool_clicked('motion')
    window.node.status = status(holding_item=True)
    modes.poll()
    window._poll_tool_handoff()
    assert window.pending_tool is None
    assert 'refused' in window.station_startup.alarm
    window.tools.start.assert_not_called()
    assert 'stop core' not in modes.sequence


def test_close_during_initial_camera_wait_cancels_without_robot_command(local):
    view, _future = local
    view.node.status = status(state='UNCONFIGURED', configured=False,
                              message='CAMERA ALARM: starting')
    view.local_preparation.request(PATHS)
    view._poll_local_preparation()
    view.close()
    assert view.local_preparation.state == 'CANCELED'
    view.node.service_clients['configure'].call_async.assert_not_called()
    view.node.service_clients['stop'].call_async.assert_not_called()


@pytest.mark.parametrize('previous', ['FAILED', 'CANCELED'])
def test_successful_load_auto_replaces_old_local_error_after_fresh_health(local, previous):
    from robot_controller.station_health import HealthRow

    view, _future = local
    view.local_preparation.transition(previous, 'Item teach schema_version must be exactly 16')
    view.station_startup.fail_health([view.local_preparation.message])
    view.auto_detectors.state = 'RUNNING'  # Supervisor confirmed matching enabled READY.
    view._health_rows.return_value = [HealthRow('tf', 'Robot TF', 'STALE', 'old')]
    view._poll_local_preparation()
    view._refresh_hmi()
    assert view.local_preparation.state == 'OFF'
    assert view.station_startup.checking and view.station_startup.require_perception
    assert view.station_startup.alarm  # Success alone cannot bypass station health.
    deadline = view.station_startup.deadline
    view._poll_local_preparation()
    view._refresh_hmi()
    assert view.station_startup.deadline == deadline  # One bounded recheck, not a retry loop.
    view._health_rows.return_value = []
    view._refresh_hmi()
    view._poll_local_preparation()
    view._refresh_controls()
    assert not view.station_startup.alarm and not view.station_startup.checking
    assert view.status.text() == 'READY'
    view.node.service_clients['configure'].call_async.assert_not_called()
    view.node.service_clients['stop'].call_async.assert_not_called()


@pytest.mark.parametrize('auto_state', [
    'OFF', 'DEPLOYING', 'STARTING', 'DETECTORS_READY', 'PREPARING', 'STOPPING', 'FAILED',
])
def test_unsuccessful_load_auto_cannot_dismiss_local_failure(local, auto_state):
    view, _future = local
    view.local_preparation.fail('Invalid selected profile')
    view.auto_detectors.state = auto_state
    view._poll_local_preparation()
    view.station_startup.update_health([])
    assert view.local_preparation.state == 'FAILED'
    assert 'Invalid selected profile' in view.station_startup.alarm
    assert not view.station_startup.checking
    view.auto_detectors.state = 'OFF'


@pytest.mark.parametrize('pending', ['awaiting_reply', 'stop_needed'])
def test_load_auto_cannot_retire_uncontained_local_preparation(tmp_path, pending):
    prep = LocalPreparation(tmp_path)
    prep.fail('Previous request failed')
    setattr(prep, pending, True)
    assert not prep.superseded_by_load_auto()
    assert prep.state == 'FAILED' and getattr(prep, pending)


def test_load_auto_health_timeout_does_not_restore_old_schema_error(local):
    view, _future = local
    view.local_preparation.fail('Old schema error')
    view._poll_local_preparation()
    view.auto_detectors.state = 'RUNNING'
    view._poll_local_preparation()
    view.station_startup.deadline = -1.
    view.station_startup.update_health(['Item perception: missing'])
    view._poll_local_preparation()
    assert 'Item perception: missing' in view.station_startup.alarm
    assert 'Old schema error' not in view.station_startup.alarm
    assert not view.station_startup.checking
