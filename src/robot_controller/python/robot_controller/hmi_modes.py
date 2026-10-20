"""Local HMI ownership handoff between production and manual tools."""

import os
from pathlib import Path
import time

from .auto_detection import idle_reason
from .hmi_tools import BY_KEY, MAINTENANCE_NODES, ToolLaunches
from .events import PackageEventLogger


CORE_NODES = {'robot_controller', 'robot_controller_preview'}
AUTO_MODES = {'AUTO', 'REMOTE_AUTO'}
MODE_NAMES = {'AUTO': 'Local Auto', 'REMOTE_AUTO': 'Remote Auto', 'MANUAL': 'Manual / Maintenance'}


def local_core_or_maintenance():
    """Catch local command applications before ROS discovery, including other domains."""
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            args = (path / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        names = {Path(os.fsdecode(arg)).name for arg in args[:2] if arg}
        conflict = names.intersection(CORE_NODES | MAINTENANCE_NODES)
        if conflict:
            return ', '.join(sorted(conflict))
    return ''


class HmiModes:
    def __init__(self, node, tools, detectors, pending):
        self.node, self.tools, self.detectors, self.pending = node, tools, detectors, pending
        self.managed = getattr(node, 'manage_controller', False)
        self.core = ToolLaunches(node)
        self.events = PackageEventLogger(node.root, 'robot_controller_hmi_modes')
        self.state = 'MANUAL' if self.managed else 'EXTERNAL'
        self.message = 'Manual tools; production controller is stopped'
        self.deadline = 0.
        self.stopping_core = False
        self.destination = 'MANUAL'
        self.previous_mode = 'AUTO'

    def transition(self, state, message):
        self.state, self.message = state, message
        self.events.record('ERROR' if state == 'FAILED' else 'INFO', 'hmi_mode',
                           message, mode=state)

    def names(self):
        return {name for name, _ns in self.node.get_node_names_and_namespaces()}

    def local_owner(self):
        return local_core_or_maintenance()

    def auto_blocker(self):
        names = self.names()
        conflict = names.intersection(CORE_NODES | MAINTENANCE_NODES)
        if conflict:
            return ('Close existing controller/maintenance applications: '
                    + ', '.join(sorted(conflict)))
        if self.node.status is not None:
            return 'Wait for the previous controller status to expire'
        owned = [BY_KEY[key].title for key in self.tools.active()
                 if BY_KEY[key].kind in ('maintenance', 'calibration')]
        if owned:
            return 'Close maintenance tools first: ' + ', '.join(sorted(owned))
        local = local_core_or_maintenance()
        return ('Close existing local controller/maintenance applications: ' + local
                if local else '')

    def reason(self, target):
        if not self.managed:
            return 'Mode switching requires the normal HMI launch'
        if self.state in ('STARTING_AUTO', 'STOPPING_AUTO'):
            return 'Wait for the mode change to finish'
        if self.pending():
            return 'Finish pending requests and turn Preview OFF before changing mode'
        if target == self.state:
            return 'Station is already in this mode'
        if target == 'MANUAL' or target in AUTO_MODES and self.state in AUTO_MODES:
            if self.state not in AUTO_MODES:
                return 'Start Auto and obtain fresh idle feedback before entering Manual'
            if self.core.snapshot('production')[0] != 'RUNNING':
                return 'The owned production controller is unavailable'
            return idle_reason(self.node.status)
        if target in AUTO_MODES:
            if self.core.active():
                return 'Wait for the owned controller to stop'
            if self.detectors.snapshot[0] not in ('OFF', 'FAILED'):
                return 'Wait for headless detectors to stop'
            return self.auto_blocker()
        return 'Unknown station mode'

    def request(self, target):
        reason = self.reason(target)
        if reason:
            raise ValueError(reason)
        self.destination = target
        if self.state in AUTO_MODES:
            self.previous_mode = self.state
            self.detectors.stop()
            self.stopping_core = False
            self.transition('STOPPING_AUTO',
                            'Stopping headless detectors, then releasing production ownership…')
            self.deadline = time.monotonic() + 65.
        else:
            self.transition('STARTING_AUTO',
                            'Starting production controller; robot preparation remains explicit…')
            self.deadline = time.monotonic() + 30.

            def check():
                reason = self.auto_blocker()
                if self.pending() or reason:
                    raise ValueError(reason or 'An HMI request is pending')
            try:
                self.core.start('production', check)
            except Exception as exc:
                self.transition('FAILED', str(exc))
                raise

    def poll(self):
        if not self.managed:
            return
        core_state, core_message = self.core.snapshot('production')
        if self.state == 'STARTING_AUTO':
            if core_state in ('FAILED', 'OFF'):
                self.fail('Production startup failed: ' + core_message)
            elif (core_state == 'RUNNING' and self.node.status is not None
                  and self.node.status.configuration_editable
                  and 'robot_controller' in self.names()):
                self.transition(self.destination, MODE_NAMES[self.destination] +
                                '; explicitly load the recipe to prepare the robot')
            elif time.monotonic() > self.deadline:
                self.fail('Production controller did not publish fresh status within 30 seconds')
        elif self.state == 'STOPPING_AUTO':
            if not self.stopping_core and self.detectors.snapshot[0] in ('OFF', 'FAILED'):
                # Recheck after detector shutdown; never discard a newly active/held context.
                reason = idle_reason(self.node.status)
                if reason:
                    self.transition(self.previous_mode, 'Mode switch refused: ' + reason)
                    return
                self.core.stop('production')
                self.stopping_core = True
            if self.stopping_core and core_state == 'FAILED':
                self.fail('Controller shutdown failed: ' + core_message)
            elif (self.stopping_core and core_state == 'OFF'
                  and not self.names().intersection(CORE_NODES)
                  and not local_core_or_maintenance()):
                self.node.clear_controller_status()
                self.transition('MANUAL', 'Manual tools; production controller is stopped')
                if self.destination in AUTO_MODES:
                    target = self.destination
                    try:
                        self.request(target)
                    except (ValueError, RuntimeError) as exc:
                        self.fail('Mode handoff failed: ' + str(exc))
            elif time.monotonic() > self.deadline:
                self.fail('Timed out releasing production ownership; inspect the command log')
        elif self.state in AUTO_MODES and core_state in ('FAILED', 'OFF'):
            self.fail('Production controller exited: ' + core_message)

    def fail(self, message):
        self.transition('FAILED', message)
        self.detectors.stop()
        self.core.stop('production')

    def close(self):
        if self.managed:
            self.core.close(timeout=45.)
