"""Camera-loss interlock with synthetic diagnostics and a fake robot transport."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from robot_controller.camera_health import CameraHealth, ALARM, RESTORED
from robot_controller.controller import RobotController
from robot_controller.errors import CommandRejected, StopUnconfirmed
from robot_controller.operator_ui import presentation, button_policy
from robot_controller.state_machine import ControllerStateMachine
from test_recovery_return import RecoveryRig
from test_status_ui import status, window  # noqa: F401


EXPECTED = {'orbbec_camera_launcher/bin_camera': 'BIN',
            'orbbec_camera_launcher/robot_camera': 'ROBOT'}
OWNERS = [SimpleNamespace(node_name='supervisor', node_namespace='/camera_watchdog')]


def message(stamp=1, *, healthy=True):
    value = DiagnosticArray()
    value.header.stamp.sec = stamp
    for name, serial in EXPECTED.items():
        value.status.append(DiagnosticStatus(
            name=name, hardware_id=serial,
            level=DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR,
            message='Healthy' if healthy else 'robot_camera disconnected',
            values=[KeyValue(key='phase', value='healthy' if healthy else 'alarm')]))
    return value


def healthy_guard():
    clock = [10.]
    guard = CameraHealth(EXPECTED, required=True, clock=lambda: clock[0])
    guard.update(message(), OWNERS, 1_000_000_000)
    assert not guard.unavailable()
    return guard, clock


def test_required_startup_blocks_preparation_without_issuing_robot_commands():
    rig = RecoveryRig(held=False)
    rig.machine = ControllerStateMachine()
    rig.startup_complete = False
    rig.holding_item = False
    rig.camera_health = CameraHealth(EXPECTED, required=True)
    rig.publish_operator_log = Mock()
    rig._stop_unexpected_idle_motion = Mock()
    RobotController._supervise_cameras(rig)
    assert rig.camera_health.latched
    rig._stop_unexpected_idle_motion.assert_not_called()
    with pytest.raises(CommandRejected, match='Check camera connection'):
        rig._begin_operation('configure')
    assert not rig.operation_lock.locked()


@pytest.mark.parametrize('state', ['READY', 'PICKING', 'PLACING', 'PAUSED', 'RECOVERING'])
@pytest.mark.parametrize('held', [False, True])
def test_loss_stops_once_preserving_outputs_source_and_requires_explicit_recovery(state, held):
    rig = RecoveryRig(held=held)
    rig.machine = ControllerStateMachine(state)
    rig.startup_complete = True
    rig.camera_health, clock = healthy_guard()
    rig.publish_operator_log = Mock()
    rig.supervision_stop_thread = None
    rig._stop_unexpected_idle_motion = lambda reason: RobotController._stop_unexpected_idle_motion(
        rig, reason)
    outputs, source = dict(rig.expected_outputs), rig.managed.session
    clock[0] += 5.01  # Watchdog disappearance also trips the interlock.
    for _ in range(5):
        RobotController._supervise_cameras(rig)
    rig.supervision_stop_thread.join(1.)
    assert not rig.supervision_stop_thread.is_alive()
    assert rig.machine.state == 'RECOVERY_REQUIRED'
    assert rig.cancel_event.is_set()
    assert sum(row[0] == 'stop' for row in rig.log) == 1
    assert rig.expected_outputs == outputs
    assert rig.managed.session is source
    assert not any(row[0] in ('move', 'output', 'home', 'pulse') for row in rig.log)
    rig.camera_health.update(message(2), OWNERS, 2_000_000_000)
    assert rig.camera_health.message(True).startswith(RESTORED)
    with pytest.raises(CommandRejected, match='Explicit preparation or Recover'):
        rig._begin_operation('pick')
    assert rig.cancel_event.is_set()
    rig._begin_operation('recover')
    assert not rig.camera_health.latched and not rig.cancel_event.is_set()


def test_unconfirmed_stop_stays_blocked_after_camera_reconnect():
    rig = RecoveryRig(held=True)
    rig.machine = ControllerStateMachine('HOLDING')
    rig.camera_health, _ = healthy_guard()
    rig.publish_operator_log = Mock()
    rig.supervision_stop_thread = None
    rig._stop_unexpected_idle_motion = lambda reason: RobotController._stop_unexpected_idle_motion(
        rig, reason)
    rig.hardware.confirm_stop = Mock(side_effect=StopUnconfirmed('no advancing robot feedback'))
    rig.camera_health.update(message(2, healthy=False), OWNERS, 2_000_000_000)
    RobotController._supervise_cameras(rig)
    rig.supervision_stop_thread.join(1.)
    assert rig.machine.state == 'FAULT'
    assert 'unconfirmed' in rig.machine.message
    rig.camera_health.update(message(3), OWNERS, 3_000_000_000)
    RobotController._supervise_cameras(rig)
    assert rig.machine.state == 'FAULT' and rig.cancel_event.is_set()


@pytest.mark.parametrize('invalid', ['old', 'repeated', 'future', 'owner', 'duplicate', 'serial', 'partial'])
def test_stale_replayed_or_wrong_camera_health_cannot_authorize_motion(invalid):
    guard, _ = healthy_guard()
    value, owners, now = message(2), OWNERS, 2_000_000_000
    if invalid == 'old':
        now = 8_000_000_001
    elif invalid == 'repeated':
        value = message(1)
    elif invalid == 'future':
        now = 1_000_000_000
    elif invalid == 'owner':
        owners = []
    elif invalid == 'duplicate':
        owners = OWNERS * 2
    elif invalid == 'serial':
        value.status[0].hardware_id = 'OTHER'
    else:
        value.status.pop()
    guard.update(value, owners, now)
    assert guard.unavailable().startswith(ALARM)
    with pytest.raises(CommandRejected):
        guard.admit('recover', acknowledge=True)


def test_external_controller_monitors_watchdog_once_seen():
    guard = CameraHealth(EXPECTED)
    assert not guard.unavailable()
    guard.update(message(1, healthy=False), OWNERS, 1_000_000_000)
    assert guard.unavailable()


def test_camera_loss_during_admission_does_not_clear_cancellation_or_leak_owner_lock():
    rig = RecoveryRig(held=False)
    rig.machine = ControllerStateMachine('RECOVERY_REQUIRED')
    rig.cancel_event.set()
    rig.camera_health, _ = healthy_guard()
    original = rig.camera_health.admit

    def admit(operation, *, acknowledge=False):
        if acknowledge:
            rig.camera_health.update(message(2, healthy=False), OWNERS, 2_000_000_000)
        original(operation, acknowledge=acknowledge)
    rig.camera_health.admit = admit
    with pytest.raises(CommandRejected, match='Check camera connection'):
        rig._begin_operation('recover')
    assert rig.cancel_event.is_set()
    assert not rig.operation_lock.locked()


def test_alarm_prompt_and_button_guards_include_feedback_loss_and_active_auto_run():
    sample = status(state='STOPPING', message=ALARM + 'Check robot_camera USB connection',
                    auto_run_active=True, feedback_fresh=False)
    assert presentation(sample)[0] == 'CAMERA ALARM'
    assert all(button_policy(sample).values())
    sample = status(state='RECOVERY_REQUIRED', message=RESTORED + 'Click Recover')
    assert not button_policy(sample)['recover']
    assert button_policy(sample)['pick']


def test_late_ready_callback_does_not_erase_latched_camera_alarm():
    guard, _ = healthy_guard()
    guard.update(message(2, healthy=False), OWNERS, 2_000_000_000)
    assert guard.trip()
    guard.update(message(3), OWNERS, 3_000_000_000)
    assert guard.latched
    assert not guard.trip()
    guard.admit('configure', acknowledge=True)
    assert not guard.latched


@pytest.mark.parametrize('mode', ['AUTO', 'MANUAL', 'STARTING_AUTO'])
def test_hmi_keeps_camera_prompt_visible_during_mode_changes_and_pending_requests(window, mode):
    window.node.camera_health = CameraHealth(EXPECTED, required=True)
    window.modes.managed = True
    window.modes.state = mode
    window.node.status = status(state='STOPPING', message=ALARM + 'Check camera connection')
    window.pending['configure'] = Mock()
    window._refresh_controls()
    assert window.status.text() == 'CAMERA ALARM'
    assert 'Check camera connection' in window.status_detail.text()
    if mode == 'AUTO':
        assert window.stop.isEnabled()
