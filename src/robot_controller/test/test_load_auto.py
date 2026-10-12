"""Deployment/lifetime ownership and run-source separation; no robot or ROS nodes."""

from collections import deque
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import os
import signal
import threading

import pytest
from rclpy.action import GoalResponse

from robot_controller import auto_deploy, auto_detection
from robot_controller import gui as gui_module
from robot_controller.auto_detection import AutoDetectors, idle_reason
from robot_controller.controller import RobotController
from robot_controller.errors import FeedbackFailure
from robot_controller.perception_mode import ANY, TEACH, HEADLESS, SERVICES
from robot_controller.tray_client import TrayClient
from robot_controller_interfaces.action import AutoRun
from test_lifecycle_v2 import reservation
from test_status_ui import status, window  # noqa: F401


@pytest.fixture
def catalog(tmp_path, monkeypatch):
    sources = {}
    for directory, names in (
            ('item_teach', ('item_teach_part.yaml', 'item_teach_part.pt')),
            ('bin_teach', ('bin_teach_bin.yaml',)),
            ('tray_teach', ('tray_teach_tray.yaml', 'tray_teach_tray.pt'))):
        folder = tmp_path / 'offline_teach' / directory
        folder.mkdir(parents=True)
        for name in names:
            path = folder / name
            path.write_bytes(('new ' + name).encode())
            sources[path] = auto_deploy.file_sha256(path)
    # Readers have their own schema suites; exercise filesystem failures independently.
    monkeypatch.setattr(auto_deploy, 'selected_files', lambda *_args: sources.copy())
    old = tmp_path / 'runtime_teach'
    old.mkdir()
    (old / 'old.yaml').write_text('old deployment')
    outside = tmp_path / 'preserved'
    outside.mkdir()
    (outside / 'keep').write_text('untouched')
    (old / 'link').symlink_to(outside, target_is_directory=True)
    return tmp_path, sources


def test_replace_catalog_preserves_sources_and_never_follows_old_links(catalog):
    root, sources = catalog
    result = auto_deploy.deploy(root, (), lambda: None)
    assert set(result) == {p.name for p in sources}
    assert set(p.name for p in (root / 'runtime_teach').iterdir()) == set(result)
    for source, digest in sources.items():
        deployed = root / 'runtime_teach' / source.name
        assert auto_deploy.file_sha256(source) == auto_deploy.file_sha256(deployed) == digest
        assert not deployed.is_symlink() and deployed.stat().st_ino != source.stat().st_ino
    assert (root / 'preserved/keep').read_text() == 'untouched'
    assert not list(root.glob('.load_auto-*'))
    (root / 'runtime_teach/obsolete.txt').write_text('remove on next enable')
    auto_deploy.deploy(root, (), lambda: None)
    assert not (root / 'runtime_teach/obsolete.txt').exists()


@pytest.mark.parametrize('failure', ['cancel', 'source_changed', 'copy', 'publish', 'catalog'])
def test_failed_deploy_retains_old_catalog(catalog, monkeypatch, failure):
    root, sources = catalog
    calls = 0

    def check():
        nonlocal calls
        calls += 1
        if calls == 3:
            if failure == 'cancel':
                raise InterruptedError('cancel')
            if failure == 'source_changed':
                next(iter(sources)).write_text('changed')

    if failure in ('copy', 'publish'):
        method = Path.open if failure == 'copy' else Path.rename

        def fail(path, *args, **kwargs):
            if '.load_auto-' in str(path) and (
                    (failure == 'copy' and args == ('xb',)) or
                    (failure == 'publish' and path.name == 'runtime_teach')):
                raise OSError('simulated filesystem failure')
            return method(path, *args, **kwargs)
        monkeypatch.setattr(Path, 'open' if failure == 'copy' else 'rename', fail)
    if failure == 'catalog':
        monkeypatch.setattr(auto_deploy, 'runtime_teach_catalog',
                            Mock(side_effect=ValueError('invalid catalog')))
    with pytest.raises((ValueError, OSError)):
        auto_deploy.deploy(root, (), check)
    assert (root / 'runtime_teach/old.yaml').read_text() == 'old deployment'
    assert not list(root.glob('.load_auto-*'))
    assert (root / 'preserved/keep').read_text() == 'untouched'


def test_symlink_destination_rejected_before_source_reads(tmp_path, monkeypatch):
    outside = tmp_path / 'protected'
    outside.mkdir()
    (tmp_path / 'runtime_teach').symlink_to(outside, target_is_directory=True)
    validation = Mock()
    monkeypatch.setattr(auto_deploy, 'selected_files', validation)
    with pytest.raises(ValueError, match='symlink'):
        auto_deploy.deploy(tmp_path, (), lambda: None)
    validation.assert_not_called()


@pytest.mark.parametrize('selections', [(), ('', '', ''), ('item', 'bin', '')])
def test_all_three_teach_selections_required(tmp_path, selections):
    with pytest.raises(ValueError, match='Select Item Teach'):
        auto_deploy.selected_files(tmp_path, selections)


@pytest.mark.parametrize('invalid', ['item', 'bin', 'tray', 'calibration', 'bin_binding'])
def test_strict_reader_and_calibration_failures_prevent_deployment(tmp_path, monkeypatch, invalid):
    names = {'item': 'load_item_profile', 'bin': 'load_bin_teach', 'tray': 'load_profile',
             'calibration': 'load_calibration_selection', 'bin_binding': 'place_bin_roi'}
    for name in names.values():
        monkeypatch.setattr(auto_deploy, name, Mock(return_value=None))
    monkeypatch.setattr(auto_deploy, 'saved_calibration_paths', lambda _: ('p', 'b', 'r'))
    auto_deploy.load_calibration_selection.return_value = (SimpleNamespace(platform='p'), 'r')
    getattr(auto_deploy, names[invalid]).side_effect = ValueError('bad ' + invalid)
    with pytest.raises(ValueError, match='bad ' + invalid):
        auto_deploy.selected_files(tmp_path, ('item.yaml', 'bin.yaml', 'tray.yaml'))


def graph(node, owners):
    node.get_node_names_and_namespaces = lambda: list(owners)
    node.get_service_names_and_types_by_node = lambda name, ns: [(
        SERVICES['pick' if name.startswith('item') else 'place'], [])]


@pytest.mark.parametrize('mode', [TEACH, HEADLESS])
def test_run_buttons_share_quantity_target_and_debug_but_pin_source(window, mode):  # noqa: F811
    node = window.node
    suffix = 'teach' if mode == TEACH else 'detect'
    graph(node, [(f'item_{suffix}', '/'), (f'tray_{suffix}', '/')])
    window.auto_detectors.state = 'RUNNING' if mode == HEADLESS else 'OFF'
    node.status = status(tray_position_recorded=True)
    client = Mock(server_is_ready=Mock(return_value=True))
    node.action_clients['auto_run'] = client
    window.auto_quantity.setValue(95)
    window.place_x.setText('35')
    window.place_y.setText('45')
    window.place_rotation.setValue(15)
    window.debug_images.setChecked(True)
    window._refresh()
    assert window.auto_run_button.text() == 'Manual Run'
    assert window.headless_run_button.text() == 'Auto Run'
    assert window.auto_run_button.isEnabled() is (mode == TEACH)
    assert window.headless_run_button.isEnabled() is (mode == HEADLESS)
    (window.auto_run_button if mode == TEACH else window.headless_run_button).click()
    goal = client.send_goal_async.call_args.args[0]
    assert goal.detector_mode == mode and goal.quantity == 95
    assert (goal.x_mm, goal.y_mm, goal.rotation_deg) == (35, 45, 15)
    assert goal.save_debug_images
    assert not window.auto_quantity.isEnabled()
    assert not window.auto_run_button.isEnabled() and not window.headless_run_button.isEnabled()
    assert window.stop.isEnabled()


@pytest.mark.parametrize('auto_state', [
    'OFF', 'DEPLOYING', 'STARTING', 'PREPARING', 'STOPPING', 'FAILED'])
def test_headless_run_requires_owned_ready_pair(window, auto_state):  # noqa: F811
    window.node.status = status(tray_position_recorded=True)
    client = Mock(server_is_ready=Mock(return_value=True))
    window.node.action_clients['auto_run'] = client
    graph(window.node, [('item_detect', '/'), ('tray_detect', '/')])
    window.auto_detectors.state = auto_state
    window.auto_detectors.preparation_claimed = True  # Admission-only fixture.
    window._refresh()
    assert not window.headless_run_button.isEnabled()
    window._action('auto_run', detector_mode=HEADLESS)
    client.send_goal_async.assert_not_called()


def test_load_auto_default_off_waits_for_detectors_before_preparing(window):  # noqa: F811
    assert not window.load_auto.isChecked()
    for field, text in zip((window.item_path, window.bin_path, window.tray_path), ('i', 'b', 't')):
        field.setText(text)
    started = []

    def start(paths):
        started.append(paths)
        window.auto_detectors.state = 'STARTING'
    window.auto_detectors.start = start
    window._refresh()
    window.load_auto.click()
    assert started == [('i', 'b', 't')]
    assert window.load_auto.isChecked() and window.load_auto.isEnabled()  # Cancellation allowed.
    assert not window.item_path.isEnabled() and not window.configure.isEnabled()
    window.auto_detectors.state = 'RUNNING'
    window._refresh()
    assert window.configure.isEnabled()  # Explicit reload remains available.
    assert not window.item_path.isEnabled()
    window.auto_detectors.stop = Mock()
    window.load_auto.click()
    window.auto_detectors.stop.assert_called_once()
    assert not window.pending  # Fixture fails if any robot/configuration call is made.


@pytest.fixture
def preparing(window, monkeypatch):  # noqa: F811
    graph(window.node, [('item_detect', '/'), ('tray_detect', '/')])
    window.auto_detectors.selections = ('item.yaml', 'bin.yaml', 'tray.yaml')
    window.auto_detectors._state('PREPARING', 'Preparing robot')
    window._clear_preview = Mock()
    monkeypatch.setattr(gui_module, 'save_state', Mock())
    monkeypatch.setattr(gui_module.QtWidgets.QMessageBox, 'warning', Mock())
    future = Future()
    window.node.service_clients['configure'] = Mock(
        service_is_ready=Mock(return_value=True), call_async=Mock(return_value=future))
    window.node.service_clients['stop'] = Mock(
        service_is_ready=Mock(return_value=True), call_async=Mock(side_effect=lambda _: Future()))

    def stop():
        window.auto_detectors.cancel.set()
        window.auto_detectors._state('STOPPING', 'Canceled')
    window.auto_detectors.stop = Mock(side_effect=stop)
    yield window, future
    # Fixture shutdown is not another operator action.
    window.auto_preparing = False


def test_ready_detectors_prepare_exact_frozen_selection_once(preparing):
    view, future = preparing
    # Browsed fields cannot replace the deployment's frozen selection.
    view.item_path.setText('other.yaml')
    view.node.status = status(state='UNCONFIGURED', configured=False, startup_complete=False)
    view._refresh()
    client = view.node.service_clients['configure']
    request = client.call_async.call_args.args[0]
    assert (request.item_teach_file, request.bin_teach_file, request.tray_teach_file) == (
        'item.yaml', 'bin.yaml', 'tray.yaml')
    assert view.load_auto.isEnabled() and view.load_auto.isChecked()
    assert not view.configure.isEnabled() and not view.headless_run_button.isEnabled()
    assert not view.item_path.isEnabled() and view.stop.isEnabled()
    view._refresh()
    client.call_async.assert_called_once()
    future.set_result(SimpleNamespace(success=True, configuration_id='new', message='Ready'))
    view._refresh()
    assert view.auto_detectors.preparation_result == ('new', '')
    assert view.auto_detectors.state == 'PREPARING'  # Supervisor must verify matching READY.
    view._refresh()
    client.call_async.assert_called_once()
    view.node.service_clients['stop'].call_async.assert_not_called()


@pytest.mark.parametrize('failure', ['rejected', 'empty', 'exception', 'unavailable'])
def test_failed_preparation_cannot_make_load_auto_running(preparing, failure):
    view, future = preparing
    client = view.node.service_clients['configure']
    if failure == 'unavailable':
        client.service_is_ready.return_value = False
    view._refresh()
    if failure == 'exception':
        future.set_exception(RuntimeError('broken reply'))
    elif failure != 'unavailable':
        future.set_result(None if failure == 'empty' else SimpleNamespace(
            success=False, message='Preparation refused'))
    view._refresh()
    assert view.auto_detectors.preparation_result[1]
    assert not view.headless_run_button.isEnabled()
    assert not view.auto_preparing
    if failure == 'exception':
        view.node.service_clients['stop'].call_async.assert_called_once()
    if failure == 'unavailable':
        client.call_async.assert_not_called()


@pytest.mark.parametrize('cancel', ['off', 'stop', 'close', 'detector_failure'])
def test_preparation_cancel_contains_issued_request_and_ignores_late_success(preparing, cancel):
    view, future = preparing
    view._refresh()
    if cancel == 'off':
        view.load_auto.click()
    elif cancel == 'stop':
        view._immediate_stop()
    elif cancel == 'close':
        view.close()
    else:
        view.auto_detectors._state('FAILED', 'Detector exited')
        view._refresh()
    assert view.auto_prepare_canceled
    stop_client = view.node.service_clients['stop']
    stop_client.call_async.assert_called_once()
    first_stop = view.pending['stop']
    future.set_result(SimpleNamespace(success=True, configuration_id='late', message='Ready'))
    view._refresh()
    assert view.auto_detectors.preparation_result is None
    assert view.auto_stop_again
    first_stop.set_result(SimpleNamespace(success=True))
    view._refresh()
    assert stop_client.call_async.call_count == 2
    assert not view.auto_stop_again and 'stop' in view.pending
    assert view.auto_detectors.state != 'RUNNING'


@pytest.mark.parametrize('stage', ['DEPLOYING', 'STARTING', 'PREPARING'])
def test_stop_before_preparation_blocks_deferred_configure(preparing, stage):
    view, _future = preparing
    view.auto_detectors.state = stage
    view._immediate_stop()
    view._refresh()
    view.node.service_clients['configure'].call_async.assert_not_called()
    view.node.service_clients['stop'].call_async.assert_called_once()
    assert view.auto_detectors.take_preparation() is None


@pytest.mark.parametrize('fields', [
    dict(holding_item=True), dict(operation_active=True),
    dict(feedback_fresh=False), dict(digital_input_bits=1)])
def test_preparation_rechecks_robot_admission_after_detector_start(preparing, fields):
    view, _future = preparing
    view.node.status = status(**fields)
    view._refresh()
    view.node.service_clients['configure'].call_async.assert_not_called()
    assert view.auto_detectors.preparation_result[1]


@pytest.mark.parametrize('fields', [
    dict(operation_active=True), dict(holding_item=True), dict(state='PAUSED'),
    dict(digital_input_bits=1), dict(feedback_fresh=False), dict(robot_running=True),
    dict(robot_queue_active=True), dict(configuration_editable=False)])
def test_busy_or_held_controller_blocks_detector_changes(window, fields):  # noqa: F811
    window.node.status = status(**fields)
    window.auto_detectors.state = 'RUNNING'
    window.auto_detectors.stop = Mock()
    window._refresh()
    assert not window.load_auto.isEnabled()
    window._toggle_auto(False)
    window.auto_detectors.stop.assert_not_called()
    assert idle_reason(window.node.status)


def test_gui_close_stops_owned_pair(window):  # noqa: F811
    window.auto_detectors.close = Mock()
    window.close()
    window.auto_detectors.close.assert_called_once()


@pytest.mark.parametrize('mode,suffix,accept', [
    (ANY, 'teach', True), (TEACH, 'teach', True), (HEADLESS, 'detect', True),
    (TEACH, 'detect', False), (HEADLESS, 'teach', False), (42, 'teach', False)])
def test_goal_enforces_source_before_reserving_robot(mode, suffix, accept):
    node, calls = reservation('READY')
    node._reserve_goal = lambda *args: RobotController._reserve_goal(node, *args)
    node._service_providers = lambda service: [(
        ('item_' if service == SERVICES['pick'] else 'tray_') + suffix, '/')]
    goal = AutoRun.Goal(configuration_id='active', quantity=1, x_mm=30., y_mm=40.,
                        detector_mode=mode)
    assert RobotController._auto_run_goal(node, goal) == (
        GoalResponse.ACCEPT if accept else GoalResponse.REJECT)
    assert calls == (['auto_run'] if accept else [])


@pytest.mark.parametrize('kind', ['pick', 'place'])
@pytest.mark.parametrize('mode,suffix', [(TEACH, 'teach'), (HEADLESS, 'detect')])
def test_active_run_rechecks_source_on_retry_and_continue(kind, mode, suffix):
    prefix = 'item' if kind == 'pick' else 'tray'
    owners = [(prefix + '_' + suffix, '/')]
    node = SimpleNamespace(auto_run=SimpleNamespace(detector_mode=mode),
                           _service_providers=lambda _: owners)
    check = (lambda: RobotController.check_detector_owner(node)) if kind == 'pick' else (
        lambda: TrayClient.check_owner(SimpleNamespace(node=node)))
    check()
    owners[:] = [(prefix + ('_detect' if mode == TEACH else '_teach'), '/')]
    with pytest.raises(FeedbackFailure, match='This run requires'):
        check()
    owners[:] = []
    with pytest.raises(FeedbackFailure):
        check()


@pytest.fixture
def manager(tmp_path):
    node = SimpleNamespace(root=tmp_path, status=status())
    graph(node, [])
    return AutoDetectors(node)


def test_armed_foreign_provider_blocks_start_without_stopping_it(manager):
    graph(manager.node, [('item_teach', '/')])
    with pytest.raises(ValueError, match='Disarm Item Teach'):
        manager._check()
    assert not manager.children


def test_unarmed_headless_process_blocks_deployment(manager):
    manager.node.get_node_names_and_namespaces = lambda: [('item_detect', '/')]
    manager.node.get_service_names_and_types_by_node = lambda *_: []
    with pytest.raises(ValueError, match='already running'):
        manager._check()


@pytest.mark.parametrize('failure', [
    'second_spawn', 'child_exit', 'conflict', 'service_lost', 'timeout', 'cancel',
    'prepare_rejected', 'prepare_timeout', 'ready_timeout'])
def test_supervisor_cleans_owned_pair_without_restart(manager, monkeypatch, failure):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    manager._check = lambda: None
    for kind, package in auto_detection.PACKAGES.items():
        exe = manager.root / 'install' / package / 'lib' / package / auto_detection.NODES[kind]
        exe.parent.mkdir(parents=True)
        exe.write_text('mock executable, never run')
        exe.chmod(0o700)
    deployed = Mock(return_value={'validated': 'digest'})
    monkeypatch.setattr(auto_detection, 'deploy', deployed)
    children, signals = {}, []
    monkeypatch.setattr(auto_detection.os, 'killpg', lambda pid, sig: signals.append((pid, sig)))
    ticks = 0

    def spawn(kind, _exe):
        if failure == 'second_spawn' and kind == 'place':
            raise OSError('second child failed')
        process = Mock(pid=100 + len(children), returncode=1)
        process.poll.side_effect = lambda: 1 if failure == 'child_exit' else None
        process.wait.return_value = 0
        children[kind] = process
        manager.children[kind] = (process, deque(['native stderr']))

    def wait(_duration):
        nonlocal ticks
        ticks += 1
        assert ticks < 8
        if ticks == 2:
            if failure in ('cancel', 'service_lost', 'ready_timeout'):
                manager.prepared('test-configuration')
            elif failure == 'prepare_rejected':
                manager.prepared(error='Configure rejected')
        return (failure == 'cancel' and ticks == 3) or ticks == 5

    def available(_node, kind):
        if failure == 'timeout' or (failure == 'service_lost' and ticks > 2):
            return []
        return [('item_teach', '/')] if failure == 'conflict' else [
            (auto_detection.NODES[kind], '/')]
    manager._spawn = spawn
    manager.cancel = SimpleNamespace(is_set=lambda: False, wait=wait)
    monkeypatch.setattr(auto_detection, 'providers', available)
    if failure in ('timeout', 'prepare_timeout', 'ready_timeout'):
        timer = iter(range(0, 10000, 200))
        monkeypatch.setattr(auto_detection.time, 'monotonic', lambda: next(timer))
    if failure == 'ready_timeout':
        manager.node.status.configuration_id = 'old'
    manager._run(('item', 'bin', 'tray'))
    assert manager.snapshot[0] == ('OFF' if failure == 'cancel' else 'FAILED')
    assert not manager.children
    deployed.assert_called_once()
    assert signals and {pid for pid, _sig in signals} == {p.pid for p in children.values()}
    assert all(p.wait.called for p in children.values())
    states = [state for state, _message in manager.take_messages()]
    assert states.count('STARTING') == 1
    if failure in ('cancel', 'service_lost'):
        assert 'RUNNING' in states
    elif failure.startswith('prepare') or failure == 'ready_timeout':
        assert 'PREPARING' in states and 'RUNNING' not in states


def test_cancel_wins_over_concurrent_ready_transition(manager):
    manager.state = 'PREPARING'
    manager.cancel.set()
    manager._state('RUNNING', 'Must not publish readiness', expected='PREPARING')
    manager.prepared('late')
    assert manager.state == 'PREPARING' and manager.preparation_result is None
    assert manager.take_preparation() is None


def test_owned_process_group_cleanup_with_harmless_executable(manager):
    # Real process plumbing only: no ROS, camera, model or robot executable.
    executable = manager.root / 'harmless_child'
    executable.write_text('#!/usr/bin/python3\nimport time\ntime.sleep(60)\n')
    executable.chmod(0o700)
    manager._spawn('pick', executable)
    child = manager.children['pick'][0]
    try:
        assert os.getpgid(child.pid) == child.pid
        manager._stop_children()
        assert child.poll() is not None
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait(timeout=2)


def test_close_cancels_supervisor_thread(manager):
    manager.thread = threading.Thread(target=manager.cancel.wait)
    manager.thread.start()
    manager.close()
    assert manager.cancel.is_set() and not manager.thread.is_alive()
