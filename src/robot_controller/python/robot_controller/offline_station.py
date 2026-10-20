"""Own explicitly initialized bringup/TF and cameras; the HMI owns the core."""

import fcntl
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading

from item_perception_yolo.platform_teach_core import workspace_root

from .events import PackageEventLogger


LAUNCHES = (
    ('bringup', 'dobot_bringup_v4', 'dobot_bringup_ros2.launch.py', ()),
    ('cameras', 'orbbec_camera_launcher', 'camera_headless.launch.py', ()),
)
CONFLICT_EXECUTABLES = {
    'dobot_bringup_v4_node', 'robot_state_publisher', 'actual_joint_state_monitor.py',
    'camera_watchdog', 'ob_camera_node', 'robot_controller', 'robot_controller_preview',
    'robot_controller_gui', 'item_detect', 'tray_detect', 'motion_debug',
    'motion_debug_gui', 'gripper_control', 'gripper_control_gui', 'camera_calibration',
}
CONFLICT_LAUNCHES = {
    'dobot_bringup_ros2.launch.py', 'robot_state.launch.py', 'camera_headless.launch.py',
    'gemini_330_series.launch.py',
}


def check_existing_processes(proc=Path('/proc')):
    """Refuse existing owners, including ones not yet discovered or in other ROS domains."""
    ancestors = {os.getpid()}
    current = os.getpid()
    # The explicitly owning HMI is an ancestor of this dependency supervisor.
    # Exclude that exact process, not every HMI or another station's processes.
    while current > 1:
        try:
            fields = (proc / str(current) / 'status').read_text().splitlines()
            current = int(next(line.split()[1] for line in fields if line.startswith('PPid:')))
        except (OSError, StopIteration, ValueError):
            break
        if current in ancestors:
            break
        ancestors.add(current)
    for process in proc.iterdir():
        if not process.name.isdigit() or int(process.name) in ancestors:
            continue
        try:
            args = [os.fsdecode(arg) for arg in (process / 'cmdline').read_bytes().split(b'\0')
                    if arg]
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        # Only executable/script positions, never shell source or ros2 package arguments.
        executables = {Path(arg).name for arg in args[:2]}
        launches = ({Path(arg).name for arg in args[1:]}
                    if args and any(Path(arg).name == 'ros2' for arg in args[:2]) else set())
        camera_container = ('component_container' in executables
                            and '__node:=camera_container' in args)
        conflict = executables.intersection(CONFLICT_EXECUTABLES) or \
            launches.intersection(CONFLICT_LAUNCHES)
        if conflict or camera_container:
            name = sorted(conflict)[0] if conflict else 'camera_container'
            raise RuntimeError(
                f'Offline station cannot start: {name} is already running (PID {process.name}). '
                'Close the existing station processes first. If bringup/cameras are managed '
                'separately, launch with start_dependencies:=false. Nothing was stopped.')


class OfflineStation:
    """No ROS command clients, readiness bypasses, recipe restoration or outer retry loop."""

    def __init__(self, root):
        self.root = Path(root)
        self.cancel = threading.Event()
        self.children = {}
        self.failed_dependencies = set()
        self.events = PackageEventLogger(self.root, 'robot_controller_offline_station')

    def report(self, level, event, message):
        print(f'[Offline station] {message}', flush=True)
        self.events.record(level, event, message)

    def commands(self):
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('Source scripts/source_ros_workspace.bash first')
        ros2 = shutil.which('ros2')
        if ros2 is None:
            raise RuntimeError('ros2 is unavailable; source the canonical workspace')
        result = []
        for role, package, filename, arguments in LAUNCHES:
            launch = self.root / 'install' / package / 'share' / package / 'launch' / filename
            if not launch.is_file():
                raise RuntimeError(f'Build {package} first; missing installed launch: {launch}')
            result.append((role, [ros2, 'launch', '--noninteractive', str(launch), *arguments]))
        return result

    def run(self):
        directory = self.root / 'logs/robot_controller'
        directory.mkdir(parents=True, exist_ok=True)
        try:
            with (directory / 'offline_station.lock').open('a') as lock:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as exc:
                    raise RuntimeError(
                        'Another offline station already owns this workspace') from exc
                commands = self.commands()  # Validate every path before starting any process.
                check_existing_processes()
                try:
                    for role, command in commands:
                        if self.cancel.is_set():
                            return 0
                        self.children[role] = subprocess.Popen(
                            [sys.executable, '-m', 'robot_controller.auto_detector_child',
                             str(os.getpid()), *command], cwd=self.root,
                            stdin=subprocess.DEVNULL, start_new_session=True)
                        self.report('INFO', 'station_child_started', f'Started owned {role}')
                    while not self.cancel.wait(.1):
                        for role, process in self.children.items():
                            if role in self.failed_dependencies:
                                continue
                            code = process.poll()
                            if code is not None:
                                if role == 'cameras':
                                    self.failed_dependencies.add(role)
                                    self.report('ERROR', 'station_camera_alarm',
                                                f'Camera supervisor exited ({code}). '
                                                'Check camera connection and supervisor logs. '
                                                'HMI/bringup retained; controller camera '
                                                'interlock blocks production. Relaunch the '
                                                'camera supervisor after resolving its error.')
                                    continue
                                self.report('ERROR', 'station_child_exit',
                                            f'{role} launch exited ({code}); '
                                            'closing owned dependencies')
                                return 1
                    return 0
                finally:
                    self.close()
        except Exception as exc:
            self.report('ERROR', 'station_start_failed', str(exc))
            return 1

    def close(self):
        # Separate sessions prevent terminal Ctrl-C from killing bringup before
        # the core's existing active-operation Stop/confirmation has completed.
        for role in ('cameras', 'bringup'):
            process = self.children.get(role)
            if process is None:
                continue
            self.report('INFO', 'station_child_stopping', f'Stopping owned {role}')
            grace = 40.
            for signum, timeout in ((signal.SIGINT, grace), (signal.SIGTERM, 5.),
                                    (signal.SIGKILL, 2.)):
                try:
                    os.killpg(process.pid, signum)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    self.report('ERROR', 'station_shutdown_timeout',
                                f'{role} did not exit after {signal.Signals(signum).name}')
                    continue
                # Clean up only this owned group if the launch exited first.
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                break
        self.children.clear()


def main():
    station = OfflineStation(workspace_root())
    previous = {number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)}
    for number in previous:
        signal.signal(number, lambda *_args: station.cancel.set())
    try:
        return station.run()
    finally:
        for number, handler in previous.items():
            signal.signal(number, handler)
