"""Explicit HMI initialization. No recipe, enable, motion or I/O commands."""

import time

from .hmi_tools import ToolLaunches
from .offline_station import check_existing_processes
from .hmi_modes import AUTO_MODES
from .events import PackageEventLogger


class StationStartup:
    def __init__(self, node, modes):
        self.node, self.modes = node, modes
        self.managed = getattr(node, 'manage_controller', False)
        self.owns_dependencies = getattr(node, 'manage_dependencies', False)
        self.dependencies = ToolLaunches(node)
        self.events = PackageEventLogger(node.root, 'robot_controller_station_health')
        self.last_alarm = ''
        self.state = 'OFF' if self.managed else 'EXTERNAL'
        self.mode = 'REMOTE_AUTO'
        self.message = 'Press Initialize Station to start the required nodes'
        self.core_requested = False
        self.alarm = ''
        self.checking = False
        self.deadline = 0.
        self.require_perception = False

    def check_health(self, *, perception=False):
        self.require_perception = perception
        self.checking = True
        self.deadline = time.monotonic() + 180.
        self.message = 'Checking required node health…'

    def update_health(self, errors):
        if self.checking:
            if not errors:
                self.checking = False
                self.alarm = ''
                self.last_alarm = ''
                self.events.record('INFO', 'station_health_ready', 'Required nodes are healthy')
                self.message = 'Required nodes are healthy; recipe preparation remains explicit'
            elif time.monotonic() >= self.deadline:
                self.fail_health(['Required nodes did not become healthy: ' + '; '.join(errors)])

    def fail_health(self, errors):
        self.checking = False
        detail = 'Station is not ready: ' + '; '.join(errors)
        if detail != self.last_alarm:
            self.events.record('ERROR', 'station_health_alarm', detail)
            self.last_alarm = detail
        self.alarm = detail if len(detail) <= 360 else detail[:320] + '… See station health log.'
        self.message = self.alarm

    def select(self, mode):
        if self.state not in ('OFF', 'FAILED'):
            raise ValueError('Initialization has already been requested')
        if mode not in AUTO_MODES | {'MANUAL'}:
            raise ValueError('Unknown station mode')
        self.mode = mode

    def start(self):
        if not self.managed or self.state not in ('OFF', 'FAILED'):
            raise ValueError('Station initialization is already active')
        if self.dependencies.active() or self.modes.core.active():
            raise ValueError('Wait for owned station shutdown before retrying initialization')
        if self.owns_dependencies:
            # Preflight all installed paths before starting a child.
            from .offline_station import OfflineStation
            OfflineStation(self.node.root).commands()
            check_existing_processes()
            self.dependencies.start('station_dependencies', check_existing_processes)
        self.core_requested = False
        self.state = 'STARTING'
        self.message = 'Starting required station nodes; robot preparation remains explicit'
        self.check_health()

    def poll(self):
        if self.state not in ('STARTING', 'RUNNING'):
            return
        if self.owns_dependencies:
            state, detail = self.dependencies.snapshot('station_dependencies')
            if state in ('OFF', 'FAILED'):
                self.state, self.message = 'FAILED', 'Station dependencies failed: ' + detail
                self.fail_health([self.message])
                if self.modes.core.active():
                    self.modes.fail(self.message)
                return
            if state != 'RUNNING':
                return
        if not self.core_requested:
            if self.owns_dependencies:
                # RUNNING means the launch process exists, not that its inner
                # ownership preflight finished. Wait for a dependency identity
                # before spawning the core, which that preflight would reject.
                nodes = self.node.get_node_names_and_namespaces()
                driver = getattr(self.node, 'driver_name', 'dobot_bringup_ros2')
                if not any(identity in nodes for identity in (
                        (driver, '/'), ('dobot_robot_state_publisher', '/'),
                        ('supervisor', '/camera_watchdog'))):
                    if time.monotonic() >= self.deadline:
                        self.fail_health([
                            'Dependency launch published no canonical nodes within 180 seconds'])
                        self.state = 'FAILED'
                        self.dependencies.stop('station_dependencies')
                    return
            self.core_requested = True
            if self.mode in AUTO_MODES:
                try:
                    self.modes.request(self.mode)
                except (ValueError, RuntimeError) as exc:
                    # Keep healthy dependencies available for a guarded core retry.
                    self.state = 'RUNNING'
                    self.modes.fail(str(exc))
                    self.fail_health([str(exc)])
                    return
            else:
                self.checking = False
        if self.state != 'RUNNING':
            self.state = 'RUNNING'
            self.message = 'Nodes started; checking health before loading a recipe'

    def close(self):
        # Caller closes the production core first, retaining bringup for Stop.
        self.dependencies.close(timeout=110.)
