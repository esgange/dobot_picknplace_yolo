"""Recipe sources, explicit startup and health admission without station hardware."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from robot_controller import auto_deploy, configuration, station_startup
from robot_controller.gui import ControllerWindow
from robot_controller.hmi_modes import HmiModes
from robot_controller.station_health import health_rows, readiness_errors
from robot_controller.station_startup import StationStartup
from test_hmi_modes import modes, ready  # noqa: F401
from test_status_ui import status, window  # noqa: F401
from test_load_auto import preparing  # noqa: F401


@pytest.fixture
def runtime(tmp_path, monkeypatch):
    directory = tmp_path / 'runtime_teach'
    directory.mkdir()
    for name in ('item_teach_a.yaml', 'item_teach_a.pt', 'bin_teach_a.yaml',
                 'tray_teach_a.yaml', 'tray_teach_a.pt'):
        (directory / name).write_text(name)
    # Schema/model/calibration readers have independent suites; keep their
    # invocation here and exercise the actual catalog, hashes and metadata fences.
    for name in ('load_item_profile', 'load_bin_teach', 'load_profile', 'place_bin_roi'):
        monkeypatch.setattr(auto_deploy, name, Mock())
    monkeypatch.setattr(auto_deploy, 'saved_calibration_paths', lambda _: ('p', 'b', 'r'))
    monkeypatch.setattr(auto_deploy, 'load_calibration_selection',
                        lambda *_args, **_kwargs: (SimpleNamespace(platform='p'), 'c'))
    return tmp_path


def test_remote_validation_never_modifies_runtime_or_needs_offline_files(runtime):
    before = {p.name: (p.read_bytes(), p.stat().st_ino, p.stat().st_mtime_ns)
              for p in (runtime / 'runtime_teach').iterdir()}
    bound = auto_deploy.validated_runtime(runtime)
    auto_deploy.check_runtime(runtime, *bound)
    assert not (runtime / 'offline_teach').exists()
    after = {p.name: (p.read_bytes(), p.stat().st_ino, p.stat().st_mtime_ns)
             for p in (runtime / 'runtime_teach').iterdir()}
    assert before == after
    assert all(p.parent == runtime / 'runtime_teach' for p in bound[0])
    assert auto_deploy.load_item_profile.call_args.kwargs['deployment'] is True
    assert auto_deploy.load_profile.call_args.kwargs['deployment'] is True


@pytest.mark.parametrize('change', ['edit', 'replace', 'missing', 'extra', 'symlink'])
def test_changed_runtime_recipe_requires_explicit_reload(runtime, change):
    bound = auto_deploy.validated_runtime(runtime)
    item = bound[0][0]
    if change == 'edit':
        item.write_text('changed')
    elif change == 'replace':
        replacement = item.with_name('.replacement')
        replacement.write_bytes(item.read_bytes())
        replacement.replace(item)
    elif change == 'missing':
        item.unlink()
    elif change == 'extra':
        item.with_name('item_teach_extra.yaml').write_text('extra')
    else:
        item.unlink()
        item.symlink_to(item.with_suffix('.pt'))
    with pytest.raises((ValueError, FileNotFoundError)):
        auto_deploy.check_runtime(runtime, *bound)


def test_runtime_change_during_validation_is_rejected(runtime, monkeypatch):
    real = auto_deploy.selected_files

    def replacing(*args, **kwargs):
        result = real(*args, **kwargs)
        path = next(iter(result))
        other = path.with_name('.replacement')
        other.write_bytes(path.read_bytes())
        other.replace(path)
        return result
    monkeypatch.setattr(auto_deploy, 'selected_files', replacing)
    with pytest.raises(ValueError, match='changed'):
        auto_deploy.validated_runtime(runtime)


def test_runtime_configure_uses_exact_catalog_not_offline_selection(runtime, monkeypatch):
    paths = auto_deploy.runtime_paths(runtime)
    loaded = Mock(return_value='runtime configuration')
    monkeypatch.setattr(configuration, 'load_runtime_configuration', loaded)
    kinematics = object()
    assert configuration.load_configuration(
        paths[0], paths[1], runtime, kinematics, tray_path=paths[2]) == 'runtime configuration'
    loaded.assert_called_once_with(runtime, kinematics)
    with pytest.raises(ValueError, match='complete runtime_teach'):
        configuration.load_configuration(
            paths[0], runtime / 'offline_teach/bin_teach/other.yaml',
            runtime, kinematics, tray_path=paths[2])
    assert loaded.call_count == 1


def test_missing_tray_pair_cannot_prepare_remote_auto(runtime):
    for p in (runtime / 'runtime_teach').glob('tray*'):
        p.unlink()
    with pytest.raises(ValueError, match='complete Tray'):
        auto_deploy.validated_runtime(runtime)


def test_switch_local_to_remote_releases_old_core_before_starting_unconfigured(modes):  # noqa: F811
    ready(modes)
    modes.request('REMOTE_AUTO')
    modes.poll()
    assert modes.sequence == ['stop detectors', 'stop core']
    modes.core.state = 'OFF'
    modes.node.nodes = []
    modes.poll()
    assert modes.state == 'STARTING_AUTO'
    assert modes.node.status is None
    assert modes.sequence == ['stop detectors', 'stop core', 'start core']
    modes.core.state = 'RUNNING'
    modes.node.nodes = [('robot_controller', '/')]
    modes.node.status = status(state='UNCONFIGURED', configured=False)
    modes.poll()
    assert modes.state == 'REMOTE_AUTO'
    assert not modes.node.status.configured


@pytest.mark.parametrize('fields', [{'holding_item': True}, {'state': 'FAULT'},
                                   {'operation_active': True}, {'feedback_fresh': False}])
def test_source_switch_cannot_discard_held_active_or_fault_context(modes, fields):  # noqa: F811
    ready(modes)
    modes.node.status = status(**fields)
    with pytest.raises(ValueError):
        modes.request('REMOTE_AUTO')
    assert not modes.sequence


def test_hmi_opens_without_starting_any_process(window, monkeypatch):  # noqa: F811
    node = SimpleNamespace(**vars(window.node), manage_controller=True, manage_dependencies=True)
    start = Mock(side_effect=AssertionError('Opening HMI must not launch nodes'))
    monkeypatch.setattr('robot_controller.hmi_tools.ToolLaunches.start', start)
    view = ControllerWindow(node)
    view.timer.stop()
    try:
        assert view.station_startup.state == 'OFF'
        assert view.initialize_station.isEnabled()
        assert view.status.text() == 'NOT INITIALIZED'
        assert all('NOT STARTED' == value.text() for _title, value in view.health_labels.values())
        assert not view.load_auto.isEnabled()
        start.assert_not_called()
    finally:
        view.close()


def test_initialize_dependencies_then_core_without_configuration(modes, monkeypatch):  # noqa: F811
    modes.node.manage_dependencies = True
    startup = StationStartup(modes.node, modes)
    startup.dependencies = Mock()
    startup.dependencies.active.return_value = set()
    startup.dependencies.snapshot.return_value = ('STARTING', '')
    monkeypatch.setattr(station_startup, 'check_existing_processes', Mock())
    monkeypatch.setattr('robot_controller.offline_station.OfflineStation.commands', Mock())
    startup.start()
    startup.dependencies.start.assert_called_once()
    assert modes.sequence == []
    startup.poll()
    assert modes.sequence == []
    startup.dependencies.snapshot.return_value = ('RUNNING', '')
    startup.poll()
    assert modes.sequence == []  # The child launch still may be doing its preflight.
    modes.node.nodes = [('dobot_bringup_ros2', '/')]
    startup.poll()
    assert modes.sequence == ['start core']
    assert modes.destination == 'REMOTE_AUTO'
    assert startup.checking
    assert modes.node.status is None  # No recipe preparation/enable call.


def test_dependency_failure_keeps_hmi_and_stops_only_owned_core(modes):  # noqa: F811
    ready(modes)
    modes.node.manage_dependencies = True
    startup = StationStartup(modes.node, modes)
    startup.state = 'RUNNING'
    startup.dependencies = Mock()
    startup.dependencies.snapshot.return_value = ('FAILED', 'driver exited')
    startup.poll()
    assert startup.state == 'FAILED'
    assert 'driver exited' in startup.alarm
    assert modes.sequence == ['stop detectors', 'stop core']


def test_hmi_shutdown_keeps_dependencies_until_core_closed(window):  # noqa: F811
    order = []
    window.auto_detectors.close = lambda: order.append('detectors')
    window.tools.close = lambda: order.append('tools')
    window.modes.close = lambda: order.append('core')
    window.station_startup.close = lambda: order.append('dependencies')
    window.close()
    assert order == ['detectors', 'tools', 'core', 'dependencies']


def test_remote_placeholder_blocks_preparation_and_local_actions(window):  # noqa: F811
    window.modes.state = 'REMOTE_AUTO'
    window.auto_detectors.start_runtime = Mock()
    window.auto_detectors.start = Mock()
    window._refresh()
    assert not window.load_auto.isEnabled()
    assert window.load_auto.isHidden()
    assert not window.item_path.isEnabled()
    assert not window.home_button.isEnabled()
    assert not window.headless_run_button.isEnabled()
    window._toggle_auto(True)
    window.auto_detectors.start_runtime.assert_not_called()
    window.auto_detectors.start.assert_not_called()


def test_remote_cannot_claim_preparation_or_overwrite_local_selection(preparing):  # noqa: F811
    view, future = preparing
    view.modes.state = 'REMOTE_AUTO'
    view.item_path.setText('/offline/item_teach_saved.yaml')
    view._configure_paths = Mock()
    view._refresh()
    view._configure_paths.assert_not_called()
    assert not view.auto_detectors.preparation_claimed
    assert view.item_path.text() == '/offline/item_teach_saved.yaml'


def healthy_rows():
    nodes = [(n, '/') for n in (
        'dobot_bringup_ros2', 'dobot_actual_joint_state_monitor', 'dobot_robot_state_publisher',
        'robot_controller', 'robot_controller_preview', 'supervisor', 'item_detect', 'tray_detect')]
    nodes = [(name, '/camera_watchdog' if name == 'supervisor' else ns) for name, ns in nodes]
    camera = SimpleNamespace(seen=True, unavailable=lambda: '', health_rows=lambda: [
        ('cameras/bin_camera', 'HEALTHY', 'fresh'), ('cameras/robot_camera', 'HEALTHY', 'fresh')])
    return nodes, camera


def test_health_requires_feedback_and_tf_not_just_discovery():
    nodes, camera = healthy_rows()
    rows = health_rows(status(), nodes, camera, (True, 'fresh'), preview_available=True)
    assert readiness_errors(rows) == []
    rows = health_rows(status(feedback_fresh=False), nodes, camera,
                       (False, 'stale transform'), preview_available=True)
    errors = readiness_errors(rows)
    assert any('Robot feedback: stale' in value for value in errors)
    assert any('Robot TF: unverified' in value for value in errors)


def test_perception_not_loaded_is_optional_only_for_initialization():
    nodes, camera = healthy_rows()
    nodes = [(name, ns) for name, ns in nodes if not name.endswith('_detect')]
    rows = health_rows(status(), nodes, camera, (True, 'fresh'), preview_available=True)
    assert readiness_errors(rows, perception=False) == []
    assert len(readiness_errors(rows)) == 2


def test_bounded_readiness_failure_latches_alarm_until_explicit_recheck(modes, monkeypatch):  # noqa: F811
    startup = StationStartup(modes.node, modes)
    monkeypatch.setattr(station_startup.time, 'monotonic', lambda: 10.)
    startup.check_health(perception=True)
    startup.update_health(['Robot TF: missing'])
    assert not startup.alarm and startup.checking
    monkeypatch.setattr(station_startup.time, 'monotonic', lambda: 191.)
    startup.update_health(['Robot TF: missing'])
    assert 'Robot TF' in startup.alarm and not startup.checking
    startup.update_health([])
    assert startup.alarm  # Recovery of a node alone is not acknowledgment.
    startup.check_health(perception=True)
    startup.update_health([])
    assert not startup.alarm and not startup.checking


def test_local_auto_attempt_checks_health_before_sending_goal(window):  # noqa: F811
    window.modes.managed = True
    window.modes.state = 'AUTO'
    window.node.status = status(tray_position_recorded=True)
    client = Mock()
    window.node.action_clients['auto_run'] = client
    window._action('auto_run')
    client.send_goal_async.assert_not_called()
    assert window.status.text() == 'STATION ALARM'
    assert 'Robot' in window.status_detail.text()


def test_remote_selection_explicitly_initializes_but_does_not_prepare(window):
    window.modes.managed = True
    window.station_startup.state = 'OFF'
    window.station_startup.start = Mock()
    window._configure_paths = Mock()
    window._set_mode('REMOTE_AUTO')
    window.station_startup.start.assert_called_once()
    assert window.station_startup.require_perception
    assert window.remote_health_pending
    window._configure_paths.assert_not_called()


def test_remote_health_starts_missing_read_only_detectors_once(window):
    window.modes.state = 'REMOTE_AUTO'
    window._health_rows = Mock(return_value=[])
    window.auto_detectors.start_runtime_nodes = Mock()
    window._configure_paths = Mock()
    window._request_remote_health()
    window._poll_remote_health()
    window._poll_remote_health()
    window.auto_detectors.start_runtime_nodes.assert_called_once()
    window._configure_paths.assert_not_called()
    assert window.station_startup.checking  # Starting a process is not proof of health.


def test_remote_health_waits_for_base_health_before_loading_models(window):
    from robot_controller.station_health import HealthRow
    window.modes.state = 'REMOTE_AUTO'
    window._health_rows = Mock(return_value=[HealthRow('tf', 'Robot TF', 'STALE', 'old')])
    window.auto_detectors.start_runtime_nodes = Mock()
    window._request_remote_health()
    window._poll_remote_health()
    window.auto_detectors.start_runtime_nodes.assert_not_called()


def test_remote_detector_failure_latches_alarm_and_does_not_loop(window):
    window.modes.state = 'REMOTE_AUTO'
    window._health_rows = Mock(return_value=[])
    window.auto_detectors.start_runtime_nodes = Mock()
    window._request_remote_health()
    window._poll_remote_health()
    window.auto_detectors.state, window.auto_detectors.message = 'FAILED', 'Missing runtime Item YAML'
    window._poll_remote_health()
    window._poll_remote_health()
    window.auto_detectors.start_runtime_nodes.assert_called_once()
    assert 'Missing runtime Item YAML' in window.station_startup.alarm
    assert not window.station_startup.checking


def test_remote_nodes_ready_stays_read_only_even_if_toggle_handler_called(window):
    window.modes.state = 'REMOTE_AUTO'
    window.node.status = status(state='UNCONFIGURED', configured=False)
    window.auto_detectors.state = 'DETECTORS_READY'
    window._configure_paths = Mock()
    window._refresh()
    window._configure_paths.assert_not_called()
    assert window.load_auto.isHidden()
    assert window.status.text() == 'WAITING FOR REMOTE'
    assert not window.load_auto.isChecked()
    window._toggle_auto(True)
    assert window.auto_detectors.snapshot[0] == 'DETECTORS_READY'
    window._configure_paths.assert_not_called()


def test_initialization_retry_refuses_incomplete_owned_shutdown(modes):
    startup = StationStartup(modes.node, modes)
    startup.state = 'FAILED'
    modes.core.state = 'STOPPING'
    with pytest.raises(ValueError, match='owned station shutdown'):
        startup.start()
    assert not modes.sequence


def test_health_rejects_foreign_node_namespace():
    nodes, camera = healthy_rows()
    nodes = [(name, '/foreign' if name == 'robot_controller_preview' else ns)
             for name, ns in nodes]
    rows = health_rows(status(), nodes, camera, (True, 'fresh'), preview_available=True)
    assert 'Motion preview: unverified' in readiness_errors(rows)


def test_initial_dependency_timeout_alarms_without_starting_core(modes, monkeypatch):
    modes.node.manage_dependencies = True
    startup = StationStartup(modes.node, modes)
    startup.state = 'STARTING'
    startup.dependencies = Mock()
    startup.dependencies.snapshot.return_value = ('RUNNING', '')
    startup.deadline = 1.
    monkeypatch.setattr(station_startup.time, 'monotonic', lambda: 181.)
    startup.poll()
    assert startup.state == 'FAILED'
    assert 'no canonical nodes' in startup.alarm
    startup.dependencies.stop.assert_called_once_with('station_dependencies')
    assert not modes.sequence


def test_core_start_refusal_keeps_dependencies_available_for_explicit_retry(modes):
    startup = StationStartup(modes.node, modes)
    startup.state = 'STARTING'
    modes.node.status = status()  # A foreign controller is still publishing.
    startup.poll()
    assert startup.state == 'RUNNING'
    assert modes.state == 'FAILED'
    assert 'previous controller status' in startup.alarm
    assert 'start core' not in modes.sequence
