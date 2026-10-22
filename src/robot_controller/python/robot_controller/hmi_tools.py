"""Explicit HMI launches with ownership scoped to the processes it starts."""

from collections import deque
from dataclasses import dataclass
import fcntl
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time

from .events import PackageEventLogger


@dataclass(frozen=True)
class Tool:
    key: str
    title: str
    package: str
    launch: str
    nodes: tuple
    kind: str
    description: str


TOOLS = (
    Tool('remote', 'Remote Setup', 'remote_setup', 'remote_setup.launch.py',
         ('remote_setup',), 'communication',
         'Save server settings and test connections; remote robot commands are not enabled.'),
    Tool('item', 'Item Teach', 'item_perception_yolo', 'item_teach.launch.py',
         ('item_teach',), 'teach',
         'Open with selected Item/Bin files. Inspect detections; arm picking explicitly.'),
    Tool('tray', 'Tray Teach', 'tray_perception', 'tray_teach.launch.py',
         ('tray_teach',), 'teach',
         'Open with the selected Tray file. Inspect its pose; arm placement explicitly.'),
    Tool('bin', 'Bin Teach', 'item_perception_yolo', 'bin_teach.launch.py',
         ('bin_teach',), 'teach', 'Create or inspect the saved bin boundary.'),
    Tool('platform', 'Platform Teach', 'item_perception_yolo', 'platform_teach.launch.py',
         ('platform_teach',), 'teach', 'Create or inspect the calibrated working platform.'),
    Tool('rviz', 'RViz', 'dobot_rviz', 'dobot_rviz.launch.py',
         ('dobot_rviz',), 'viewer', 'View robot TF and available perception data.'),
    Tool('cameras', 'Cameras', 'orbbec_camera_launcher', 'camera_launcher.launch.py',
         ('orbbec_camera_launcher',), 'camera',
         'Open camera setup; starting cameras is explicit.'),
    Tool('calibration', 'Camera Calibration', 'camera_calibration', 'camera_calibration.launch.py',
         ('camera_calibration',), 'calibration', 'Inspect or create station camera calibrations.'),
    Tool('motion', 'Motion Debug', 'motion_debug', 'motion_debug.launch.py',
         ('motion_debug_gui', 'motion_debug'), 'maintenance',
         'Direct robot maintenance; opening this tool initializes/enables the robot.'),
    Tool('gripper', 'Gripper Diagnostics', 'gripper_control',
         'gripper_control.launch.py',
         ('gripper_control_gui', 'gripper_control'), 'maintenance',
         'Direct gripper control; requires exclusive robot command ownership.'),
)
BY_KEY = {tool.key: tool for tool in TOOLS}
BY_KEY['production'] = Tool(
    'production', 'Production controller', 'robot_controller', 'robot_controller_core.launch.py',
    ('robot_controller', 'robot_controller_preview'), 'production',
    'Controller and Preview owned by this HMI; no preparation on start.')
BY_KEY['station_dependencies'] = Tool(
    'station_dependencies', 'Station dependencies', 'robot_controller',
    'station_dependencies.launch.py', (), 'dependencies',
    'Explicitly start owned robot bringup, TF and both supervised cameras.')
MAINTENANCE_NODES = {'motion_debug_gui', 'motion_debug', 'gripper_control_gui',
                     'gripper_control', 'camera_calibration'}


def change_reason(tool, status, nodes, auto_state, pending=False):
    """Guard a tool change without sending any robot command or closing another app."""
    if tool.kind in ('viewer', 'communication'):
        return ''
    if pending or (status is not None and (
            status.operation_active or status.holding_item or status.robot_running
            or status.robot_queue_active or status.digital_input_bits & 1)):
        return 'Finish the operation and clear the held item before changing tools'
    if auto_state not in ('OFF', 'FAILED'):
        return 'Turn Load Auto OFF before changing teaching or setup tools'
    names = {name for name, _namespace in nodes}
    if status is None and 'robot_controller' in names:
        return 'Waiting for fresh controller status before changing tools'
    if tool.kind in ('maintenance', 'calibration'):
        if status is not None or 'robot_controller' in names:
            return 'Close the robot controller before opening direct robot maintenance'
        others = names.intersection(MAINTENANCE_NODES.difference(tool.nodes))
        if others:
            return 'Close competing maintenance tools: ' + ', '.join(sorted(others))
    if tool.kind == 'teach' and names.intersection({'item_detect', 'tray_detect'}):
        return 'Stop headless detectors before opening teaching tools'
    return ''


class ToolLaunches:
    """One bounded supervisor per explicitly opened window; no automatic restarts."""

    def __init__(self, node):
        self.node = node
        self.root = Path(node.root)
        self.lock = threading.Lock()
        self.records = {}
        self.messages = deque()
        self.events = PackageEventLogger(self.root, 'robot_controller_hmi')

    def _state(self, key, state, message):
        with self.lock:
            record = self.records[key]
            record['state'], record['message'] = state, message
            self.messages.append((key, state, message))
        self.events.record('ERROR' if state == 'FAILED' else 'INFO', 'tool_' + state.lower(),
                           message, tool=key)

    def snapshot(self, key):
        with self.lock:
            record = self.records.get(key)
            return ((record['state'], record['message']) if record is not None
                    else ('OFF', 'Not started by this HMI'))

    def active(self):
        with self.lock:
            return {key for key, record in self.records.items()
                    if record['state'] not in ('OFF', 'FAILED')}

    def take_messages(self):
        with self.lock:
            result = tuple(self.messages)
            self.messages.clear()
            return result

    def start(self, key, check, *, launch_arguments=()):
        tool = BY_KEY[key]
        launch_arguments = tuple(launch_arguments)
        check()
        with self.lock:
            old = self.records.get(key)
            if old is not None and old['thread'].is_alive():
                raise ValueError(f'{tool.title} is already active')
            cancel = threading.Event()
            thread = threading.Thread(target=self._run,
                                      args=(tool, cancel, check, launch_arguments), daemon=True)
            self.records[key] = {'state': 'STARTING', 'message': 'Opening…',
                                 'cancel': cancel, 'thread': thread}
            thread.start()

    def stop(self, key):
        with self.lock:
            record = self.records.get(key)
            if record is not None and record['state'] not in ('OFF', 'FAILED'):
                record['cancel'].set()
                record['state'], record['message'] = 'STOPPING', 'Closing owned window…'

    def close(self, timeout=10.):
        with self.lock:
            threads = []
            for record in self.records.values():
                record['cancel'].set()
                threads.append(record['thread'])
        deadline = time.monotonic() + timeout
        for thread in threads:
            thread.join(timeout=max(0., deadline - time.monotonic()))

    @staticmethod
    def _terminate(process, grace=5.):
        for signum, timeout in ((signal.SIGINT, grace), (signal.SIGTERM, 2.),
                                (signal.SIGKILL, 1.)):
            try:
                os.killpg(process.pid, signum)
            except ProcessLookupError:
                return
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                continue
            # The launch may have exited before one of its children; only this
            # owned session may be cleaned up, never another application's PID.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            return

    def _run(self, tool, cancel, check, launch_arguments):
        process = lock_file = None
        failure = None
        tail = deque(maxlen=8)
        try:
            directory = self.root / 'logs/robot_controller_hmi'
            directory.mkdir(parents=True, exist_ok=True)
            lock_file = (directory / (tool.key + '.lock')).open('a')
            fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
                raise ValueError('Source the ROS workspace before opening tools')
            launch = self.root / 'install' / tool.package / 'share' / tool.package / \
                'launch' / tool.launch
            ros2 = shutil.which('ros2')
            if not launch.is_file() or ros2 is None:
                raise ValueError(f'Build and source {tool.package} before opening {tool.title}')
            check()
            if cancel.is_set():
                return
            arguments = (['require_camera_watchdog:=true']
                         if tool.kind == 'production'
                         and getattr(self.node, 'require_camera_watchdog', False) else [])
            process = subprocess.Popen(
                [sys.executable, '-m', 'robot_controller.auto_detector_child',
                 str(os.getpid()), ros2, 'launch', '--noninteractive', str(launch),
                 *arguments, *launch_arguments],
                cwd=self.root, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, start_new_session=True)

            def drain():
                try:
                    while block := process.stdout.read(1024):
                        tail.append(block.decode('utf-8', errors='replace'))
                finally:
                    process.stdout.close()
            threading.Thread(target=drain, daemon=True).start()
            self._state(tool.key, 'RUNNING', 'Launched by this HMI')
            while not cancel.wait(.2):
                code = process.poll()
                if code is not None:
                    if code:
                        raise RuntimeError(f'{tool.title} exited ({code}): '
                                           + ''.join(tuple(tail))[-8192:])
                    break
        except Exception as exc:
            failure = str(exc)
        finally:
            if process is not None:
                if tool.kind == 'production':
                    self._terminate(process, grace=40.)
                elif tool.kind == 'dependencies':
                    self._terminate(process, grace=100.)
                else:
                    self._terminate(process)
            if lock_file is not None:
                lock_file.close()
            self._state(tool.key, 'FAILED' if failure else 'OFF',
                        failure or 'Owned window closed')
