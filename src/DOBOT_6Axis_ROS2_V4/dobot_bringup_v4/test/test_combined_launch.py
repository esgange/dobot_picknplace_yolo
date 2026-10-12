"""Exercise the real launch lifecycles with local dummy processes, never hardware."""

import os
from contextlib import contextmanager
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[4]
VENDOR_ROOT = PROJECT_ROOT / 'src' / 'DOBOT_6Axis_ROS2_V4'
HARNESS = '''
import os
from pathlib import Path
import runpy
import signal
import sys
import time
from unittest.mock import patch

root = Path(__file__).resolve().parent
role = sys.argv[1]
if role == 'dummy':
    name = sys.argv[2]
    def stop(signum, frame):
        raise SystemExit(0)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        with (root / (name + '.started')).open('a') as marker:
            marker.write(str(os.getpid()) + '\\n')
        trigger = root / (name + '.exit')
        while not trigger.exists():
            time.sleep(0.02)
        raise SystemExit(int(trigger.read_text()))
    finally:
        (root / (name + '.stopped')).touch()

from launch import LaunchContext, LaunchService
from launch.actions import ExecuteProcess
from launch.utilities import perform_substitutions

def dummy_node(**kwargs):
    return ExecuteProcess(
        cmd=[sys.executable, __file__, 'dummy', kwargs['executable']],
        name=kwargs['name'], output='screen', on_exit=kwargs['on_exit'])

def state_process(**kwargs):
    context = LaunchContext()
    command = [part if isinstance(part, str) else perform_substitutions(context, [part])
               for part in kwargs['cmd']]
    assert Path(command[0]).name == 'ros2'
    assert command[1:] == ['launch', '--noninteractive', 'dobot_rviz', 'robot_state.launch.py']
    kwargs['cmd'] = [sys.executable, __file__, 'state', command[2]]
    return ExecuteProcess(**kwargs)

launch_file = root / 'src' / {
    'parent': 'dobot_bringup_ros2.launch.py',
    'state': 'robot_state.launch.py',
    'viewer': 'dobot_rviz.launch.py',
}[role]
with patch('launch_ros.actions.Node', dummy_node):
    module = runpy.run_path(str(launch_file))
    if role == 'parent':
        module['generate_launch_description'].__globals__['ExecuteProcess'] = state_process
    description = module['generate_launch_description']()
service = LaunchService(noninteractive=role == 'parent' or '--noninteractive' in sys.argv)
service.include_launch_description(description)
raise SystemExit(service.run())
'''

STATE_PROCESSES = ('robot_state_publisher', 'actual_joint_state_monitor.py')
DRIVER = 'dobot_bringup_v4_node'


def wait_for(process, log_path, predicate):
    deadline = time.monotonic() + 10.0
    while not predicate():
        if process.poll() is not None or time.monotonic() >= deadline:
            pytest.fail(log_path.read_text(encoding='utf-8'))
        time.sleep(0.02)


@pytest.fixture
def launch_workspace(tmp_path):
    source = tmp_path / 'src'
    source.mkdir()
    for package, filename in (
        ('dobot_bringup_v4', 'dobot_bringup_ros2.launch.py'),
        ('dobot_rviz', 'dobot_rviz.launch.py'),
        ('dobot_rviz', 'robot_state.launch.py'),
    ):
        shutil.copyfile(VENDOR_ROOT / package / 'launch' / filename, source / filename)
    (source / 'package.xml').write_text(
        '<package><name>launch_fixture</name></package>', encoding='utf-8')
    for name in ('.env.example', '.env'):
        shutil.copyfile(PROJECT_ROOT / '.env.example', tmp_path / name)
    harness = tmp_path / 'harness.py'
    harness.write_text(HARNESS, encoding='utf-8')
    return tmp_path


@contextmanager
def launch_process(root, role):
    log_path = root / (role + '.log')
    harness = root / 'harness.py'
    environment = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(root / 'ros_logs'))
    with log_path.open('w', encoding='utf-8') as log:
        process = subprocess.Popen(
            [sys.executable, str(harness), role], env=environment,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            yield process, log_path
        finally:
            # Only this fixture's process group can be signalled, including on failure.
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5)
            finally:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


@pytest.fixture
def running_launch(launch_workspace):
    root = launch_workspace
    with launch_process(root, 'parent') as (process, log_path):
        wait_for(process, log_path, lambda: all(
            (root / (name + '.started')).exists() for name in (*STATE_PROCESSES, DRIVER)))
        assert not (root / 'rviz2.started').exists(), 'Bringup must never launch RViz'
        yield process, root, log_path


@pytest.mark.parametrize('name,code', [
    ('robot_state_publisher', 0),
    ('robot_state_publisher', 1),
    ('actual_joint_state_monitor.py', 1),
])
def test_tf_failure_stops_tf_session_but_preserves_driver(running_launch, name, code):
    process, root, log_path = running_launch
    (root / (name + '.exit')).write_text(str(code), encoding='utf-8')
    wait_for(process, log_path, lambda: all(
        (root / (name + '.stopped')).exists() for name in STATE_PROCESSES))
    wait_for(process, log_path, lambda: 'Dobot robot TF session exited' in log_path.read_text())
    assert process.poll() is None
    assert not (root / (DRIVER + '.stopped')).exists()
    # Prove the surviving driver still owns parent lifetime and can exit normally.
    (root / (DRIVER + '.exit')).write_text('0', encoding='utf-8')
    assert process.wait(timeout=10) == 0


@pytest.mark.parametrize('exit_driver', [False, True])
def test_parent_shutdown_stops_tf_and_driver(running_launch, exit_driver):
    process, root, log_path = running_launch
    if exit_driver:
        (root / (DRIVER + '.exit')).write_text('1', encoding='utf-8')
    else:
        os.killpg(process.pid, signal.SIGINT)
    assert process.wait(timeout=10) == 0, log_path.read_text()
    assert all((root / (name + '.stopped')).exists()
               for name in (*STATE_PROCESSES, DRIVER)), log_path.read_text()


@pytest.mark.parametrize('exit_code', [0, 1])
def test_separate_viewer_exit_and_reopen_leave_tf_and_driver_running(running_launch, exit_code):
    parent, root, parent_log = running_launch
    for _ in range(2):
        with launch_process(root, 'viewer') as (viewer, viewer_log):
            wait_for(viewer, viewer_log, lambda: (root / 'rviz2.started').exists())
            (root / 'rviz2.exit').write_text(str(exit_code), encoding='utf-8')
            assert viewer.wait(timeout=10) == 0, viewer_log.read_text()
        assert (root / 'rviz2.stopped').exists()
        assert parent.poll() is None, parent_log.read_text()
        for name in (*STATE_PROCESSES, DRIVER):
            assert not (root / (name + '.stopped')).exists()
            assert len((root / (name + '.started')).read_text().splitlines()) == 1
        for suffix in ('exit', 'started', 'stopped'):
            (root / ('rviz2.' + suffix)).unlink()


def test_viewer_alone_never_starts_driver_monitor_or_tf(launch_workspace):
    root = launch_workspace
    with launch_process(root, 'viewer') as (viewer, log_path):
        wait_for(viewer, log_path, lambda: (root / 'rviz2.started').exists())
        (root / 'rviz2.exit').write_text('0', encoding='utf-8')
        assert viewer.wait(timeout=10) == 0, log_path.read_text()
        for name in (*STATE_PROCESSES, DRIVER):
            assert not (root / (name + '.started')).exists()
