"""Manual/Auto ownership changes without launching applications or commanding hardware."""

from types import SimpleNamespace
from unittest.mock import Mock
import os
import sys
import time

import pytest

from robot_controller import hmi_modes
from robot_controller.hmi_modes import HmiModes
from robot_controller.hmi_tools import BY_KEY, ToolLaunches
from test_status_ui import status, window  # noqa: F401


class Core:
    def __init__(self, sequence):
        self.state = 'OFF'
        self.sequence = sequence

    def snapshot(self, _key):
        return self.state, 'fixture'

    def active(self):
        return {'production'} if self.state not in ('OFF', 'FAILED') else set()

    def start(self, key, check):
        check()
        self.sequence.append('start core')
        self.state = 'STARTING'

    def stop(self, _key):
        self.sequence.append('stop core')
        if self.active():
            self.state = 'STOPPING'

    def close(self, timeout):
        assert timeout >= 40.
        self.sequence.append('close core')

    def take_messages(self):
        return ()


@pytest.fixture
def modes(tmp_path, monkeypatch):
    monkeypatch.setattr(hmi_modes, 'local_core_or_maintenance', lambda: '')
    node = SimpleNamespace(root=tmp_path, manage_controller=True, status=None, nodes=[])
    node.get_node_names_and_namespaces = lambda: node.nodes
    node.clear_controller_status = lambda: setattr(node, 'status', None)
    sequence = []
    detectors = SimpleNamespace(snapshot=('OFF', 'fixture'))

    def stop_detectors():
        sequence.append('stop detectors')
        if detectors.snapshot[0] not in ('OFF', 'FAILED'):
            detectors.snapshot = ('STOPPING', 'fixture')
    detectors.stop = stop_detectors
    tools = SimpleNamespace(active=lambda: set())
    manager = HmiModes(node, tools, detectors, lambda: False)
    manager.core = Core(sequence)
    manager.sequence = sequence
    return manager


def ready(modes):
    modes.state = 'AUTO'
    modes.core.state = 'RUNNING'
    modes.node.nodes = [('robot_controller', '/'), ('robot_controller_preview', '/')]
    modes.node.status = status()


def test_manual_stops_detectors_then_core_and_waits_for_graph_disappearance(modes):
    ready(modes)
    modes.detectors.snapshot = ('RUNNING', '')
    modes.request('MANUAL')
    assert modes.sequence == ['stop detectors']
    assert modes.state == 'STOPPING_AUTO'
    modes.poll()
    assert modes.sequence == ['stop detectors']
    modes.detectors.snapshot = ('OFF', '')
    modes.poll()
    assert modes.sequence == ['stop detectors', 'stop core']
    modes.core.state = 'OFF'
    modes.poll()
    assert modes.state == 'STOPPING_AUTO'
    modes.node.nodes = []
    modes.poll()
    assert modes.state == 'MANUAL' and modes.node.status is None


@pytest.mark.parametrize('fields', [
    {'operation_active': True}, {'holding_item': True}, {'digital_input_bits': 1},
    {'feedback_fresh': False}, {'robot_running': True}, {'robot_queue_active': True},
    {'state': 'PAUSED'}, {'state': 'FAULT'}, {'configuration_editable': False},
])
def test_manual_refuses_active_held_stale_or_retained_context(modes, fields):
    ready(modes)
    modes.node.status = status(**fields)
    with pytest.raises(ValueError):
        modes.request('MANUAL')
    assert modes.sequence == [] and modes.state == 'AUTO'


def test_manual_rechecks_robot_after_detectors_have_stopped(modes):
    ready(modes)
    modes.request('MANUAL')
    modes.node.status = status(holding_item=True)
    modes.poll()
    assert modes.state == 'AUTO' and 'refused' in modes.message
    assert 'stop core' not in modes.sequence


def test_auto_starts_only_core_and_waits_for_fresh_controller_status(modes):
    modes.request('AUTO')
    assert modes.state == 'STARTING_AUTO' and modes.sequence == ['start core']
    modes.core.state = 'RUNNING'
    modes.poll()
    assert modes.state == 'STARTING_AUTO'
    modes.node.nodes = [('robot_controller', '/')]
    modes.node.status = status(state='UNCONFIGURED', configured=False)
    modes.poll()
    assert modes.state == 'AUTO'
    assert modes.detectors.snapshot[0] == 'OFF'
    assert modes.sequence == ['start core']  # No Configure, preparation, tool or detector start.


@pytest.mark.parametrize('name', ['motion_debug_gui', 'gripper_control_gui',
                                  'camera_calibration', 'robot_controller'])
def test_auto_refuses_foreign_maintenance_and_controller(modes, name):
    modes.node.nodes = [(name, '/')]
    with pytest.raises(ValueError, match='Close existing'):
        modes.request('AUTO')
    assert modes.sequence == []


def test_auto_refuses_owned_starting_maintenance_and_checks_local_processes(modes, monkeypatch):
    modes.tools.active = lambda: {'motion'}
    with pytest.raises(ValueError, match='Close maintenance'):
        modes.request('AUTO')
    modes.tools.active = lambda: set()
    monkeypatch.setattr(hmi_modes, 'local_core_or_maintenance', lambda: 'motion_debug_gui')
    with pytest.raises(ValueError, match='existing local'):
        modes.request('AUTO')
    assert modes.sequence == []


def test_mode_transition_blocks_reentry_and_pending_commands(modes):
    modes.pending = lambda: True
    with pytest.raises(ValueError, match='pending'):
        modes.request('AUTO')
    modes.pending = lambda: False
    modes.request('AUTO')
    with pytest.raises(ValueError, match='Wait for the mode change'):
        modes.request('MANUAL')


@pytest.mark.parametrize('phase', ['STARTING_AUTO', 'STOPPING_AUTO'])
def test_mode_timeout_fails_closed_without_opening_tools_or_restarting(modes, phase):
    modes.state = phase
    modes.core.state = 'RUNNING'
    modes.deadline = -1.
    modes.stopping_core = phase == 'STOPPING_AUTO'
    modes.poll()
    assert modes.state == 'FAILED'
    assert modes.sequence == ['stop detectors', 'stop core']
    modes.poll()
    assert modes.sequence == ['stop detectors', 'stop core']


def test_unexpected_core_exit_leaves_hmi_for_explicit_recovery(modes):
    ready(modes)
    modes.core.state = 'OFF'
    modes.poll()
    assert modes.state == 'FAILED'
    assert 'exited' in modes.message
    assert modes.reason('MANUAL')


def test_manual_tool_button_enabled_only_after_ownership_released(
        window, modes, monkeypatch):  # noqa: F811
    window.modes = modes
    modes.node = window.node
    window.node.status = None
    window.node.get_node_names_and_namespaces = lambda: []
    window.hmi_graph_at = -float('inf')
    window._refresh()
    assert window.tool_controls['motion'][0].isEnabled()
    assert not window.load_auto.isEnabled() and not window.pick_item.isEnabled()
    assert window.status.text() == 'LOCAL DIRECT TOOLS'
    assert 'active manual tool' in window.stop.toolTip()
    window.tools.start = Mock()
    window.tool_controls['motion'][0].click()
    assert window.tools.start.call_args.args[0] == 'motion'
    window.tools.start.call_args.args[1]()  # Recheck immediately before spawn.
    monkeypatch.setattr(hmi_modes, 'local_core_or_maintenance', lambda: 'gripper_control_gui')
    with pytest.raises(ValueError, match='existing local command'):
        window._tool_start_check('motion')
    monkeypatch.setattr(hmi_modes, 'local_core_or_maintenance', lambda: '')
    modes.state = 'AUTO'
    modes.core.state = 'RUNNING'
    window.node.status = status()
    window._refresh()
    assert window.tool_controls['motion'][0].isEnabled()
    # Open may request release, but direct spawn still refuses concurrent control.
    with pytest.raises(ValueError, match='Close the robot controller'):
        window._tool_start_check('motion')


def test_returning_auto_preserves_teach_window_option_for_manual_run(modes):
    modes.tools.active = lambda: {'item', 'tray', 'rviz', 'cameras'}
    modes.node.nodes = [('item_teach', '/'), ('tray_teach', '/')]
    modes.request('AUTO')
    assert modes.sequence == ['start core']


def test_hmi_exit_waits_for_owned_core_after_tools(window, modes):  # noqa: F811
    window.modes = modes
    window.auto_detectors.close = lambda: modes.sequence.append('close detectors')
    window.tools.close = lambda: modes.sequence.append('close tools')
    window.close()
    assert modes.sequence == ['close detectors', 'close tools', 'close core']


def test_owned_core_manual_tool_and_back_use_disjoint_dummy_sessions(modes, monkeypatch):
    from robot_controller import hmi_tools
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    root = modes.node.root
    for key in ('production', 'motion'):
        tool = BY_KEY[key]
        launch = root / 'install' / tool.package / 'share' / tool.package / 'launch' / tool.launch
        launch.parent.mkdir(parents=True, exist_ok=True)
        launch.touch()
    executable = root / 'dummy_ros2'
    executable.write_text('#!' + sys.executable + '''
import os, signal, sys, time
from pathlib import Path
role = 'core' if Path(sys.argv[-1]).name == 'robot_controller_core.launch.py' else 'motion'
assert sys.argv[1:3] == ['launch', '--noninteractive']
Path(role + '.pid').write_text(str(os.getpid()))
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
while True: time.sleep(.02)
''')
    executable.chmod(0o700)
    monkeypatch.setattr(hmi_tools.shutil, 'which', lambda _name: str(executable))
    modes.core = ToolLaunches(modes.node)
    modes.tools = ToolLaunches(modes.node)

    def wait_for(predicate):
        deadline = time.monotonic() + 5.
        while not predicate():
            assert time.monotonic() < deadline
            time.sleep(.01)
    try:
        modes.request('AUTO')
        wait_for(lambda: (root / 'core.pid').exists())
        modes.node.nodes = [('robot_controller', '/')]
        modes.node.status = status()
        modes.poll()
        assert modes.state == 'AUTO'
        modes.request('MANUAL')
        modes.poll()
        wait_for(lambda: modes.core.snapshot('production')[0] == 'OFF')
        core_pid = int((root / 'core.pid').read_text())
        with pytest.raises(ProcessLookupError):
            os.kill(core_pid, 0)
        modes.node.nodes = []
        modes.poll()
        assert modes.state == 'MANUAL'
        modes.tools.start('motion', lambda: None)
        wait_for(lambda: (root / 'motion.pid').exists())
        with pytest.raises(ValueError, match='Close maintenance'):
            modes.request('AUTO')
        modes.tools.stop('motion')
        wait_for(lambda: modes.tools.snapshot('motion')[0] == 'OFF')
        (root / 'core.pid').unlink()
        modes.request('AUTO')
        wait_for(lambda: (root / 'core.pid').exists())
        assert int((root / 'core.pid').read_text()) != core_pid
        assert modes.detectors.snapshot[0] == 'OFF'
    finally:
        modes.tools.close()
        modes.close()
