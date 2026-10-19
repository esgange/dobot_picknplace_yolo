"""GUI-owned headless detector pair; never controls the robot or foreign processes."""

from collections import deque
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from .auto_deploy import deploy
from .events import PackageEventLogger
from .perception_mode import SERVICES


NODES = {'pick': 'item_detect', 'place': 'tray_detect'}
PACKAGES = {'pick': 'item_perception_yolo', 'place': 'tray_perception'}


def providers(node, kind):
    service = SERVICES[kind]
    return [(name, namespace) for name, namespace in node.get_node_names_and_namespaces()
            if any(s == service for s, _types in
                   node.get_service_names_and_types_by_node(name, namespace))]


def idle_reason(status):
    if status is None:
        return 'Waiting for fresh controller status'
    if getattr(status, 'message', '').startswith('CAMERA ALARM: '):
        return status.message
    if not status.configuration_editable:
        return 'Load Auto belongs to the GUI controller'
    if (status.operation_active or status.holding_item
            or status.state not in ('UNCONFIGURED', 'INACTIVE', 'READY')):
        return 'Finish or Stop and recover the controller before changing detectors'
    if not status.feedback_fresh or status.robot_running or status.robot_queue_active:
        return 'Load Auto requires fresh stationary robot feedback'
    if status.digital_input_bits & 1:
        return 'Clear the held item before changing detectors'
    return ''


class AutoDetectors:
    """All filesystem/native startup work stays off the Qt and ROS executor threads."""

    def __init__(self, node):
        self.node = node
        self.root = Path(node.root)
        self.events = PackageEventLogger(self.root, 'robot_controller_load_auto')
        self.lock = threading.Lock()
        self.state, self.message = 'OFF', 'Headless detectors stopped'
        self.cancel = threading.Event()
        self.thread = None
        self.children = {}
        self.messages = deque()
        self.selections = ()
        self.preparation_claimed = False
        self.preparation_result = None

    @property
    def snapshot(self):
        with self.lock:
            return self.state, self.message

    def _state(self, state, message, *, expected=None):
        with self.lock:
            if expected is not None and (self.state != expected or self.cancel.is_set()):
                return
            self.state, self.message = state, message
            self.messages.append((state, message))
        self.events.record('ERROR' if state == 'FAILED' else 'INFO',
                           'load_auto_' + state.lower(), message)

    def take_messages(self):
        with self.lock:
            messages = tuple(self.messages)
            self.messages.clear()
        return messages

    def _check(self):
        if self.cancel.is_set():
            raise InterruptedError('Load Auto canceled')
        reason = idle_reason(self.node.status)
        if reason:
            raise ValueError(reason)
        if any(providers(self.node, kind) for kind in NODES):
            raise ValueError('Disarm Item Teach and Tray Teach; stop existing headless detectors')
        if any(name in NODES.values() for name, _ns in
               self.node.get_node_names_and_namespaces()):
            raise ValueError('A headless detector is already running; stop it before Load Auto')
        # Also protect catalogs used by detectors in another ROS domain/startup phase.
        for process in Path('/proc').iterdir():
            if not process.name.isdigit():
                continue
            try:
                args = (process / 'cmdline').read_bytes().split(b'\0')
            except (FileNotFoundError, ProcessLookupError, PermissionError):
                continue
            if any(Path(os.fsdecode(arg)).name in NODES.values() for arg in args if arg):
                raise ValueError('A local headless detector is running; stop it before Load Auto')

    def start(self, selections):
        if self.thread is not None and self.thread.is_alive():
            raise ValueError('Load Auto is already active')
        self.cancel.clear()
        self.selections = tuple(selections)
        self.preparation_claimed = False
        self.preparation_result = None
        self._state('DEPLOYING', 'Validating and copying the selected teach files…')
        self.thread = threading.Thread(target=self._run, args=(tuple(selections),), daemon=True)
        self.thread.start()

    def take_preparation(self):
        """Hand the GUI one Configure request after both detectors are ready."""
        with self.lock:
            if (self.state != 'PREPARING' or self.cancel.is_set()
                    or self.preparation_claimed):
                return None
            self.preparation_claimed = True
            return self.selections

    def prepared(self, configuration_id='', *, error=''):
        with self.lock:
            if self.state == 'PREPARING' and not self.cancel.is_set():
                self.preparation_result = (configuration_id, error or (
                    '' if configuration_id else 'Robot preparation returned no configuration ID'))

    def stop(self):
        if self.thread is not None and self.thread.is_alive():
            self.cancel.set()
            self._state('STOPPING', 'Stopping owned headless detectors…')

    def close(self):
        self.stop()
        if self.thread is not None:
            self.thread.join(timeout=12.)

    def _spawn(self, kind, executable):
        process = subprocess.Popen(
            [sys.executable, '-m', 'robot_controller.auto_detector_child',
             str(os.getpid()), str(executable)], cwd=self.root, start_new_session=True,
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        tail = deque(maxlen=8)
        self.children[kind] = (process, tail)

        def drain():
            try:
                while block := process.stdout.read(1024):
                    tail.append(block.decode('utf-8', errors='replace'))
            finally:
                process.stdout.close()
        threading.Thread(target=drain, daemon=True).start()

    def _stop_children(self):
        # Each direct child owns its own session, including its native worker.
        for signum, timeout in ((signal.SIGINT, 5.), (signal.SIGTERM, 2.), (signal.SIGKILL, 1.)):
            for process, _tail in self.children.values():
                try:
                    os.killpg(process.pid, signum)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + timeout
            for process, _tail in self.children.values():
                try:
                    process.wait(timeout=max(0., deadline - time.monotonic()))
                except subprocess.TimeoutExpired:
                    pass
            if all(process.poll() is not None for process, _tail in self.children.values()):
                # Kill any orphaned native worker in these owned groups only.
                for process, _tail in self.children.values():
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                break
        self.children.clear()

    def _run(self, selections):
        failure = None
        lock_file = None
        try:
            lock_path = self.root / 'logs/robot_controller/load_auto.lock'
            lock_path.parent.mkdir(parents=True, exist_ok=True)
            lock_file = lock_path.open('a')
            fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._check()
            if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
                raise ValueError('Source scripts/source_ros_workspace.bash before Load Auto')
            executables = {kind: self.root / 'install' / package / 'lib' / package / NODES[kind]
                           for kind, package in PACKAGES.items()}
            for executable in executables.values():
                if not executable.is_file() or not os.access(executable, os.X_OK):
                    raise ValueError(f'Build the headless detector first: {executable}')
            deployed = deploy(self.root, selections, self._check)
            self.events.record('INFO', 'load_auto_deployed', 'Replaced runtime_teach catalog',
                               files=deployed)
            self._check()
            self._state('STARTING',
                        'Starting Item Detect and Tray Detect; waiting for both services…')
            for kind, executable in executables.items():
                if self.cancel.is_set():
                    raise InterruptedError('Load Auto canceled')
                self._spawn(kind, executable)
            deadline = time.monotonic() + 180.
            ready_once = False
            preparation_deadline = None
            ready_deadline = None
            while not self.cancel.wait(.2):
                for kind, (process, tail) in self.children.items():
                    if process.poll() is not None:
                        raise RuntimeError(f'{NODES[kind]} exited ({process.returncode}): '
                                           + ''.join(tuple(tail))[-8192:])
                available = {kind: providers(self.node, kind) for kind in NODES}
                if any(value and value != [(NODES[kind], '/')]
                       for kind, value in available.items()):
                    raise RuntimeError('Conflicting detection provider; stopping owned detectors')
                ready = all(available.values())
                if ready and not ready_once:
                    self._state('PREPARING', 'Detectors ready; loading teach files and '
                                'preparing the robot…', expected='STARTING')
                    preparation_deadline = time.monotonic() + 180.
                    ready_once = True
                elif not ready and ready_once:
                    raise RuntimeError('A headless detection service disappeared')
                if not ready_once and time.monotonic() >= deadline:
                    raise TimeoutError('Headless detectors did not become ready within 180 seconds')
                with self.lock:
                    preparing = self.state == 'PREPARING'
                    result = self.preparation_result
                if preparing:
                    if result is not None:
                        configuration_id, error = result
                        if error:
                            raise RuntimeError('Robot preparation failed: ' + error)
                        if ready_deadline is None:
                            ready_deadline = time.monotonic() + 5.
                        status = self.node.status
                        if (not idle_reason(status) and status.state == 'READY'
                                and status.startup_complete and status.robot_enabled
                                and status.configuration_id == configuration_id):
                            self._state('RUNNING', 'Robot prepared; headless Item Detect '
                                        'and Tray Detect are ready', expected='PREPARING')
                        elif time.monotonic() >= ready_deadline:
                            raise TimeoutError('Robot preparation did not confirm matching READY')
                    if self.snapshot[0] == 'PREPARING' and time.monotonic() >= preparation_deadline:
                        raise TimeoutError('Robot preparation did not finish within 180 seconds')
        except InterruptedError:
            pass
        except Exception as exc:
            failure = str(exc)
        finally:
            self._state('STOPPING', 'Stopping owned headless detectors…')
            self._stop_children()
            if lock_file is not None:
                lock_file.close()
            self._state('FAILED' if failure else 'OFF', failure or 'Headless detectors stopped')
