"""Exercise the real launch lifecycles with local dummy processes, never hardware."""

import os
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
        (root / (name + '.started')).touch()
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

def viewer_process(**kwargs):
    context = LaunchContext()
    command = [part if isinstance(part, str) else perform_substitutions(context, [part])
               for part in kwargs['cmd']]
    assert Path(command[0]).name == 'ros2'
    assert command[1:] == ['launch', '--noninteractive', 'dobot_rviz', 'dobot_rviz.launch.py']
    kwargs['cmd'] = [sys.executable, __file__, 'viewer', command[2]]
    return ExecuteProcess(**kwargs)

launch_file = root / 'src' / (
    'dobot_bringup_ros2.launch.py' if role == 'parent' else 'dobot_rviz.launch.py')
with patch('launch_ros.actions.Node', dummy_node):
    module = runpy.run_path(str(launch_file))
    if role == 'parent':
        module['generate_launch_description'].__globals__['ExecuteProcess'] = viewer_process
    description = module['generate_launch_description']()
service = LaunchService(noninteractive=role == 'parent' or '--noninteractive' in sys.argv)
service.include_launch_description(description)
raise SystemExit(service.run())
'''

VIEWER_PROCESSES = ('rviz2', 'robot_state_publisher', 'actual_joint_state_monitor.py')
DRIVER = 'dobot_bringup_v4_node'


def wait_for(process, log_path, predicate):
    deadline = time.monotonic() + 10.0
    while not predicate():
        if process.poll() is not None or time.monotonic() >= deadline:
            pytest.fail(log_path.read_text(encoding='utf-8'))
        time.sleep(0.02)


@pytest.fixture
def running_launch(tmp_path):
    source = tmp_path / 'src'
    source.mkdir()
    for package, filename in (
        ('dobot_bringup_v4', 'dobot_bringup_ros2.launch.py'),
        ('dobot_rviz', 'dobot_rviz.launch.py'),
    ):
        shutil.copyfile(VENDOR_ROOT / package / 'launch' / filename, source / filename)
    (source / 'package.xml').write_text(
        '<package><name>launch_fixture</name></package>', encoding='utf-8')
    for name in ('.env.example', '.env'):
        shutil.copyfile(PROJECT_ROOT / '.env.example', tmp_path / name)
    harness = tmp_path / 'harness.py'
    harness.write_text(HARNESS, encoding='utf-8')
    log_path = tmp_path / 'launch.log'
    environment = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(tmp_path / 'ros_logs'))
    with log_path.open('w', encoding='utf-8') as log:
        process = subprocess.Popen(
            [sys.executable, str(harness), 'parent'], env=environment,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            wait_for(process, log_path, lambda: all(
                (tmp_path / (name + '.started')).exists()
                for name in (*VIEWER_PROCESSES, DRIVER)))
            yield process, tmp_path, log_path
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


@pytest.mark.parametrize('name,code', [
    ('rviz2', 0),
    ('rviz2', 1),
    ('actual_joint_state_monitor.py', 1),
])
def test_viewer_exit_stops_only_viewer_processes(running_launch, name, code):
    process, root, log_path = running_launch
    (root / (name + '.exit')).write_text(str(code), encoding='utf-8')
    wait_for(process, log_path, lambda: all(
        (root / (name + '.stopped')).exists() for name in VIEWER_PROCESSES))
    wait_for(process, log_path, lambda: 'Dobot RViz viewer exited' in log_path.read_text())
    assert process.poll() is None
    assert not (root / (DRIVER + '.stopped')).exists()
    # Prove the surviving driver still owns parent lifetime and can exit normally.
    (root / (DRIVER + '.exit')).write_text('0', encoding='utf-8')
    assert process.wait(timeout=10) == 0


@pytest.mark.parametrize('exit_driver', [False, True])
def test_parent_shutdown_stops_viewer_and_driver(running_launch, exit_driver):
    process, root, log_path = running_launch
    if exit_driver:
        (root / (DRIVER + '.exit')).write_text('1', encoding='utf-8')
    else:
        os.killpg(process.pid, signal.SIGINT)
    assert process.wait(timeout=10) == 0, log_path.read_text()
    assert all((root / (name + '.stopped')).exists()
               for name in (*VIEWER_PROCESSES, DRIVER)), log_path.read_text()
