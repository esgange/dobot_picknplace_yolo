"""Offline startup and ordered cleanup with dummy sessions, never station hardware."""

import fcntl
import os
from pathlib import Path
import runpy
import signal
import subprocess
import sys
import threading
import time
from unittest.mock import Mock

from launch import LaunchContext
from launch.actions import LogInfo
import pytest

from robot_controller import offline_station as station


@pytest.fixture
def owner(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    result = station.OfflineStation(tmp_path)
    for _role, package, filename, _arguments in station.LAUNCHES:
        launch = tmp_path / 'install' / package / 'share' / package / 'launch' / filename
        launch.parent.mkdir(parents=True, exist_ok=True)
        launch.touch()
    monkeypatch.setattr(station, 'check_existing_processes', lambda: None)
    yield result
    result.cancel.set()
    result.close()


@pytest.mark.parametrize('arguments', [
    ['/install/dobot/lib/dobot_bringup_v4_node'],
    ['dobot_bringup_v4_node'],
    ['/usr/bin/python3', '/install/controller/lib/robot_controller'],
    ['/usr/bin/python3', '/install/item/lib/item_detect'],
    ['/usr/bin/python3', '/install/camera/lib/camera_watchdog'],
    ['/usr/bin/python3', '/install/monitor/actual_joint_state_monitor.py'],
    ['/opt/ros/bin/ros2', 'launch', 'dobot', 'dobot_bringup_ros2.launch.py'],
    ['/usr/bin/python3', '/opt/ros/bin/ros2', 'launch', '/install/camera_headless.launch.py'],
    ['/opt/ros/lib/component_container', '--ros-args', '-r', '__node:=camera_container'],
])
def test_preflight_refuses_foreign_owners_without_signalling(tmp_path, arguments, monkeypatch):
    proc = tmp_path / '987654'
    proc.mkdir()
    (proc / 'cmdline').write_bytes(b'\0'.join(os.fsencode(arg) for arg in arguments))
    kill = Mock(side_effect=AssertionError('Foreign processes must never be signalled'))
    monkeypatch.setattr(station.os, 'killpg', kill)
    with pytest.raises(RuntimeError, match='already running.*987654'):
        station.check_existing_processes(tmp_path)
    kill.assert_not_called()


def test_preflight_ignores_shell_text_and_own_outer_launch(tmp_path):
    for pid, args in enumerate((
        ['/bin/bash', '-c', 'ps | rg robot_controller dobot_bringup_v4_node'],
        ['/usr/bin/python3', '/opt/ros/bin/ros2', 'launch', 'robot_controller',
         'robot_controller.launch.py'],
        ['/opt/ros/bin/rviz2'],
    ), 987650):
        path = tmp_path / str(pid)
        path.mkdir()
        (path / 'cmdline').write_bytes(b'\0'.join(os.fsencode(arg) for arg in args))
    station.check_existing_processes(tmp_path)


def test_missing_install_refuses_before_spawning_any_dependency(owner, monkeypatch):
    package = owner.root / 'install/robot_controller/share/robot_controller/launch'
    (package / 'robot_controller.launch.py').unlink()
    spawn = Mock(side_effect=AssertionError('Preflight must complete before launch'))
    monkeypatch.setattr(station.subprocess, 'Popen', spawn)
    assert owner.run() == 1
    spawn.assert_not_called()


def test_station_lock_refuses_second_owner(owner, monkeypatch):
    directory = owner.root / 'logs/robot_controller'
    directory.mkdir(parents=True, exist_ok=True)
    spawn = Mock(side_effect=AssertionError('A second station must never start'))
    monkeypatch.setattr(station.subprocess, 'Popen', spawn)
    with (directory / 'offline_station.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert owner.run() == 1
    spawn.assert_not_called()


def test_pre_canceled_station_never_starts(owner, monkeypatch):
    spawn = Mock(side_effect=AssertionError('Canceled startup must not start anything'))
    monkeypatch.setattr(station.subprocess, 'Popen', spawn)
    owner.cancel.set()
    assert owner.run() == 0
    spawn.assert_not_called()


DUMMY_ROS2 = '''
import json, os, signal, sys, time
from pathlib import Path
role = {'dobot_bringup_ros2.launch.py': 'bringup',
        'camera_headless.launch.py': 'cameras',
        'robot_controller.launch.py': 'controller'}[Path(sys.argv[3]).name]
assert sys.argv[1:3] == ['launch', '--noninteractive']
if role == 'controller':
    assert sys.argv[4:] == ['start_dependencies:=false']
Path(role + '.started').write_text(str(os.getpid()))
def stop(*_):
    if role == 'controller':
        # Simulate the controller's bounded Stop cleanup while bringup stays alive.
        time.sleep(.15)
        if not Path('bringup.stopped').exists():
            Path('driver_available_for_stop').touch()
    raise SystemExit(0)
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
try:
    trigger = Path(role + '.exit')
    while not trigger.exists(): time.sleep(.02)
    raise SystemExit(int(trigger.read_text()))
finally:
    Path(role + '.stopped').touch()
    with Path('order').open('a') as log: log.write(role + '\\n')
'''


def wait_until(predicate):
    deadline = time.monotonic() + 5.
    while not predicate():
        assert time.monotonic() < deadline, 'Dummy station did not reach expected state'
        time.sleep(.01)


@pytest.mark.parametrize('trigger', ['controller', 'cameras', 'bringup', 'cancel'])
def test_owned_station_exit_cleanup_and_driver_order(owner, monkeypatch, trigger):
    executable = owner.root / 'dummy_ros2'
    executable.write_text('#!' + sys.executable + '\n' + DUMMY_ROS2)
    executable.chmod(0o700)
    monkeypatch.setattr(station.shutil, 'which', lambda _name: str(executable))
    results = []
    worker = threading.Thread(target=lambda: results.append(owner.run()), daemon=True)
    worker.start()
    try:
        wait_until(lambda: all((owner.root / (role + '.started')).exists()
                               for role in ('bringup', 'cameras', 'controller')))
        if trigger == 'cancel':
            owner.cancel.set()
        else:
            (owner.root / (trigger + '.exit')).write_text('0')
        worker.join(5.)
        assert not worker.is_alive()
        assert results == [0 if trigger in ('controller', 'cancel') else 1]
        order = (owner.root / 'order').read_text().splitlines()
        assert set(order) == {'controller', 'cameras', 'bringup'}
        if trigger != 'bringup':
            assert order[-1] == 'bringup'
        if trigger in ('cancel', 'cameras'):
            assert (owner.root / 'driver_available_for_stop').is_file()
        for role in order:
            pid = int((owner.root / (role + '.started')).read_text())
            with pytest.raises(ProcessLookupError):
                os.kill(pid, 0)
        os.kill(os.getpid(), 0)  # The unrelated test runner remains alive.
    finally:
        owner.cancel.set()
        worker.join(5.)


def test_spawn_failure_cleans_only_already_owned_dependency(owner, monkeypatch):
    process = Mock(pid=987654)
    spawn = Mock(side_effect=[process, OSError('fixture camera startup failed')])
    kill = Mock()
    monkeypatch.setattr(station.subprocess, 'Popen', spawn)
    monkeypatch.setattr(station.os, 'killpg', kill)
    assert owner.run() == 1
    assert spawn.call_count == 2
    assert all(call.args[0] == process.pid for call in kill.call_args_list)
    assert kill.call_args_list[0].args[1] == signal.SIGINT


def test_real_outer_launch_ctrl_c_preserves_driver_during_controller_cleanup(owner):
    executable = owner.root / 'dummy_ros2'
    executable.write_text('#!' + sys.executable + '\n' + DUMMY_ROS2)
    executable.chmod(0o700)
    launch = Path(__file__).parents[1] / 'launch/robot_controller.launch.py'
    harness = owner.root / 'launch_fixture.py'
    harness.write_text('''
import runpy, sys
from pathlib import Path
root = Path(__file__).parent
if sys.argv[1] == 'station':
    from robot_controller import offline_station as module
    module.workspace_root = lambda: root
    module.check_existing_processes = lambda: None
    module.shutil.which = lambda _name: str(root / 'dummy_ros2')
    raise SystemExit(module.main())
from launch import LaunchService
from launch.actions import ExecuteProcess
module = runpy.run_path(sys.argv[2])
def substitute(**kwargs):
    assert kwargs.pop('executable') == 'robot_controller_station'
    kwargs.pop('package')
    return ExecuteProcess(cmd=[sys.executable, __file__, 'station'], **kwargs)
module['_launch'].__globals__['Node'] = substitute
service = LaunchService(noninteractive=True)
service.include_launch_description(module['generate_launch_description']())
raise SystemExit(service.run())
''')
    with (owner.root / 'outer.log').open('w') as output:
        process = subprocess.Popen(
            [sys.executable, str(harness), 'outer', str(launch)],
            stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            wait_until(lambda: all((owner.root / (role + '.started')).is_file()
                                   for role in ('bringup', 'cameras', 'controller')))
            os.killpg(process.pid, signal.SIGINT)
            assert process.wait(timeout=8.) == 0
            assert (owner.root / 'driver_available_for_stop').is_file()
            assert (owner.root / 'order').read_text().splitlines() == [
                'controller', 'cameras', 'bringup']
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=8.)


@pytest.mark.parametrize('headless,dependencies,executables', [
    ('false', 'true', ['robot_controller_station']),
    ('false', 'false', ['robot_controller', 'robot_controller_preview', 'robot_controller_gui']),
    ('true', 'true', ['robot_controller']),
    ('true', 'false', ['robot_controller']),
])
def test_launch_routes_default_offline_and_preserves_core_only(
        monkeypatch, headless, dependencies, executables):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    path = Path(__file__).parents[1] / 'launch/robot_controller.launch.py'
    module = runpy.run_path(str(path))
    context = LaunchContext()
    context.launch_configurations.update(headless=headless, start_dependencies=dependencies)
    nodes = []

    def capture(**kwargs):
        condition = kwargs.get('condition')
        if condition is None or condition.evaluate(context):
            nodes.append(kwargs)
        return LogInfo(msg='fixture')
    module['_launch'].__globals__['Node'] = capture
    module['_launch'](context)
    assert [node['executable'] for node in nodes] == executables
    assert all('item_detect' not in node['executable'] for node in nodes)
