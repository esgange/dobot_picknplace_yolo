"""HMI layout, launch admission and owned-process cleanup without real applications."""

import io
import json
import os
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

from PyQt5 import QtWidgets
import pytest

from robot_controller import hmi_tools
from robot_controller.hmi_tools import BY_KEY, ToolLaunches, change_reason
from test_status_ui import status, window  # noqa: F401


def test_hmi_pages_keep_stop_visible_and_do_not_start_tools(window):  # noqa: F811
    window.show()
    assert [window.pages.tabText(i) for i in range(window.pages.count())] == [
        'Production', 'Recipe', 'Teach / Test', 'Diagnostics']
    for index in range(window.pages.count()):
        window.pages.setCurrentIndex(index)
        QtWidgets.QApplication.processEvents()
        assert window.stop.isVisible() and window.pause.isVisible()
        assert window.robot_panel.isVisible() and window.gripper_panel.isVisible()
    assert window.tools.active() == set()
    assert 'No server connection is needed' in ' '.join(
        label.text() for label in window.pages.findChildren(QtWidgets.QLabel))
    production, recipe, testing, diagnostics = (window.pages.widget(i) for i in range(4))
    assert production.isAncestorOf(window.headless_run_button)
    assert production.isAncestorOf(window.load_auto)
    assert recipe.isAncestorOf(window.auto_quantity)
    assert testing.isAncestorOf(window.auto_run_button)
    assert testing.isAncestorOf(window.configure)
    assert diagnostics.isAncestorOf(window.speed_slider)
    original_size = window.size()
    window.log_toggle.setChecked(True)
    QtWidgets.QApplication.processEvents()
    assert window.size() == original_size
    assert window.stop.isVisible() and window.pause.isVisible()
    assert window.tools.active() == set()


@pytest.mark.parametrize('key', ['item', 'tray', 'motion', 'gripper', 'remote'])
def test_hmi_external_windows_are_never_owned_or_closed(window, key):  # noqa: F811
    window.node.get_node_names_and_namespaces = lambda: [(BY_KEY[key].nodes[0], '/')]
    window.hmi_graph_at = -float('inf')
    window._refresh_hmi()
    button, label = window.tool_controls[key]
    assert not button.isEnabled() and button.text() == 'External'
    assert 'outside' in label.text()
    window.tools.stop = Mock()
    button.click()
    window.tools.stop.assert_not_called()


def test_hmi_blocks_maintenance_with_controller_and_allows_passive_rviz(window):  # noqa: F811
    window.node.status = status(operation_active=True, holding_item=True)
    window._refresh_hmi()
    assert not window.tool_controls['motion'][0].isEnabled()
    assert not window.tool_controls['calibration'][0].isEnabled()
    assert window.tool_controls['rviz'][0].isEnabled()
    with pytest.raises(ValueError, match='operation'):
        window._tool_start_check('motion')
    window.node.status = status()
    with pytest.raises(ValueError, match='Close the robot controller'):
        window._tool_start_check('motion')


def test_hmi_rechecks_external_launch_on_click(window):  # noqa: F811
    window.node.get_node_names_and_namespaces = lambda: []
    window.hmi_graph_at = -float('inf')
    window._refresh_hmi()
    assert window.tool_controls['item'][0].isEnabled()
    window.node.get_node_names_and_namespaces = lambda: [('item_teach', '/')]
    with pytest.raises(ValueError, match='already running'):
        window._tool_start_check('item')


def test_load_auto_requires_owned_teaching_windows_closed_but_allows_cameras(window):  # noqa: F811
    for field in (window.item_path, window.bin_path, window.tray_path):
        field.setText('selected.yaml')
    window.tools.active = lambda: {'item', 'cameras'}
    assert 'Item Teach' in window._auto_change_reason()
    window.tools.active = lambda: {'cameras', 'rviz'}
    assert window._auto_change_reason() == ''


def test_hmi_exit_stops_active_operation_before_owned_providers(window):  # noqa: F811
    sequence = []
    window.node.status = status(operation_active=True)
    window.tools.active = lambda: {'item'}
    window._immediate_stop = lambda: sequence.append('stop')
    window.auto_detectors.close = lambda: sequence.append('detectors')
    window.tools.close = lambda: sequence.append('tools')
    window.close()
    assert sequence == ['stop', 'detectors', 'tools']
    window.tools.active = lambda: set()


@pytest.mark.parametrize('key,controller,auto,nodes,expected', [
    ('rviz', status(operation_active=True), 'RUNNING', [], ''),
    ('item', status(), 'RUNNING', [], 'Load Auto OFF'),
    ('tray', status(holding_item=True), 'OFF', [], 'held item'),
    ('cameras', status(robot_queue_active=True), 'OFF', [], 'operation'),
    ('item', None, 'OFF', [('robot_controller', '/')], 'fresh controller'),
    ('item', None, 'OFF', [('item_detect', '/')], 'headless'),
    ('motion', None, 'OFF', [('gripper_control_gui', '/')], 'competing'),
    ('gripper', status(), 'OFF', [], 'Close the robot controller'),
    ('motion', None, 'OFF', [], ''),
    ('calibration', status(), 'OFF', [], ''),
])
def test_tool_admission(key, controller, auto, nodes, expected):
    reason = change_reason(BY_KEY[key], controller, nodes, auto)
    assert expected in reason if expected else reason == ''


@pytest.fixture
def manager(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    supervisor = ToolLaunches(SimpleNamespace(root=tmp_path))
    for tool in hmi_tools.TOOLS:
        path = (tmp_path / 'install' / tool.package / 'share' / tool.package
                / 'launch' / tool.launch)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('# unused fixture; real application must never start\n')
    yield supervisor
    supervisor.close()


def wait_until(predicate):
    deadline = time.monotonic() + 5.
    while not predicate():
        assert time.monotonic() < deadline, 'Timed out waiting for owned fixture process'
        time.sleep(.01)


def test_tool_rechecks_before_spawn_and_reports_failure_without_start(manager, monkeypatch):
    spawn = Mock(side_effect=AssertionError('An application must not start'))
    monkeypatch.setattr(hmi_tools.subprocess, 'Popen', spawn)
    checks = Mock(side_effect=[None, ValueError('Controller became active')])
    manager.start('item', checks)
    wait_until(lambda: manager.snapshot('item')[0] == 'FAILED')
    assert 'Controller became active' in manager.snapshot('item')[1]
    spawn.assert_not_called()
    assert checks.call_count == 2


def test_failed_tool_only_cleans_owned_group_and_never_restarts(manager, monkeypatch):
    signals = []
    process = SimpleNamespace(pid=12345, stdout=io.BytesIO(b'fixture failure'),
                              poll=lambda: 2, wait=lambda **_kw: 2)
    spawn = Mock(return_value=process)
    monkeypatch.setattr(hmi_tools.subprocess, 'Popen', spawn)
    monkeypatch.setattr(hmi_tools.os, 'killpg', lambda pid, sig: signals.append((pid, sig)))
    manager.start('tray', lambda: None)
    wait_until(lambda: manager.snapshot('tray')[0] == 'FAILED')
    assert 'exited (2)' in manager.snapshot('tray')[1]
    assert spawn.call_count == 1
    assert signals and all(pid == 12345 for pid, _sig in signals)
    command = spawn.call_args.args[0]
    assert command[-3:-1] == ['launch', '--noninteractive']
    assert command[-1].endswith('/tray_perception/launch/tray_teach.launch.py')


def test_cancel_during_start_prevents_spawn(manager, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    spawn = Mock(side_effect=AssertionError('Canceled launch must not start'))
    monkeypatch.setattr(hmi_tools.subprocess, 'Popen', spawn)
    checks = 0

    def check():
        nonlocal checks
        checks += 1
        if checks == 2:
            entered.set()
            assert release.wait(3.)
    manager.start('item', check)
    assert entered.wait(3.)
    manager.stop('item')
    release.set()
    wait_until(lambda: manager.snapshot('item')[0] == 'OFF')
    spawn.assert_not_called()


def test_real_owned_dummy_receives_arguments_and_closes_without_touching_foreign_pid(
        manager, monkeypatch):
    # Substitute ros2 with a harmless executable. No ROS node, model or robot starts.
    executable = manager.root / 'dummy_ros2'
    executable.write_text(
        '#!' + sys.executable + '\nimport json, os, signal, sys, time\n'
        'from pathlib import Path\n'
        'Path("started.json").write_text(json.dumps([os.getpid(), sys.argv[1:]]))\n'
        'signal.signal(signal.SIGINT, lambda *_: sys.exit(0))\n'
        'while True: time.sleep(.05)\n')
    executable.chmod(0o700)
    monkeypatch.setattr(hmi_tools.shutil, 'which', lambda _name: str(executable))
    manager.start('rviz', lambda: None)
    marker = manager.root / 'started.json'
    wait_until(marker.exists)
    pid, arguments = json.loads(marker.read_text())
    assert arguments[:2] == ['launch', '--noninteractive']
    assert arguments[2].endswith('/dobot_rviz/launch/dobot_rviz.launch.py')
    with pytest.raises(ValueError, match='already active'):
        manager.start('rviz', lambda: None)
    manager.close()
    assert manager.snapshot('rviz')[0] == 'OFF'
    with pytest.raises(ProcessLookupError):
        os.kill(pid, 0)
    os.kill(os.getpid(), 0)  # The unrelated test runner survived.


def test_tool_lock_prevents_second_hmi_from_spawning(manager, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    count = 0

    def check():
        nonlocal count
        count += 1
        if count == 2:
            entered.set()
            assert release.wait(3.)
    monkeypatch.setattr(hmi_tools.subprocess, 'Popen', Mock(
        side_effect=AssertionError('No application should start in this test')))
    manager.start('item', check)
    assert entered.wait(3.)
    second = ToolLaunches(manager.node)
    try:
        second.start('item', lambda: None)
        wait_until(lambda: second.snapshot('item')[0] == 'FAILED')
        assert 'temporarily unavailable' in second.snapshot('item')[1]
    finally:
        manager.stop('item')
        release.set()
        second.close()
    wait_until(lambda: manager.snapshot('item')[0] == 'OFF')


def test_remote_setup_allowed_in_auto_and_during_run_without_command_ownership(window):  # noqa: F811
    window.node.status = status(operation_active=True, holding_item=True)
    window.modes.managed = True
    window.modes.state = 'AUTO'
    window.node.get_node_names_and_namespaces = lambda: [('robot_controller', '/')]
    window._refresh_hmi()
    assert window.tool_controls['remote'][0].isEnabled()
    window._tool_start_check('remote')
    assert window.tools.active() == set()  # Merely inspecting never launches it.
    window.node.status = status()
    window.tools.active = lambda: {'remote'}
    for field in (window.item_path, window.bin_path, window.tray_path):
        field.setText('selected.yaml')
    assert window._auto_change_reason() == ''
    window.tools.active = lambda: set()
