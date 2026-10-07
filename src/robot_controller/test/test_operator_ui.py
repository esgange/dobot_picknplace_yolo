"""Operator decisions and independent Stop with synthetic status and transports."""

from types import SimpleNamespace
from unittest.mock import Mock
import math
import threading

from builtin_interfaces.msg import Time
import pytest
from rclpy.task import Future

from robot_controller.controller import RobotController
from robot_controller.errors import CommandRejected
from robot_controller.operator_ui import presentation
from robot_controller.state_machine import ControllerStateMachine
from test_joint_position import PositionRig
from test_status_ui import status, window  # noqa: F401
from test_tray_acquisition_pause import acquisition_rig, exhaust


@pytest.mark.parametrize('internal,label', [
    ('UNCONFIGURED', 'NOT READY'), ('INACTIVE', 'NOT READY'), ('READY', 'READY'),
    ('HOLDING', 'HOLDING ITEM'), ('PAUSED', 'PAUSED'),
    *[(state, 'BUSY') for state in (
        'STARTING', 'HOMING', 'PICKING', 'TRAY_POSITIONING', 'PLACING',
        'RETURNING_ITEM', 'RECOVERING', 'PAUSING', 'STOPPING')],
    *[(state, 'ATTENTION REQUIRED') for state in ('FAULT', 'HELD_UNKNOWN', 'RECOVERY_REQUIRED')],
])
def test_internal_states_have_clear_operator_labels(internal, label):
    assert presentation(status(state=internal))[0] == label


@pytest.mark.parametrize('internal,operation,enabled', [
    ('UNCONFIGURED', '', {'configure'}),
    ('INACTIVE', '', {'configure', 'preview_toggle'}),
    ('READY', '', {'configure', 'home', 'pick', 'place', 'preview_toggle', 'speed', 'cp'}),
    ('HOLDING', '', {'home', 'place', 'pause', 'preview_toggle', 'speed', 'cp'}),
    ('HOMING', 'home', {'pause'}), ('PICKING', 'pick', {'pause'}),
    ('PLACING', 'place', {'pause'}), ('TRAY_POSITIONING', 'tray_position', {'pause'}),
    ('STARTING', 'startup', set()), ('RECOVERING', 'recover', set()),
    ('RETURNING_ITEM', 'place', set()), ('PAUSING', 'pick', set()),
    ('STOPPING', 'stop', set()), ('PAUSED', 'pick', {'continue', 'return_item'}),
    ('PAUSED', 'place', {'place', 'continue', 'return_item'}),
    ('FAULT', '', {'recover', 'preview_toggle'}),
    ('HELD_UNKNOWN', '', {'recover', 'preview_toggle'}),
    ('RECOVERY_REQUIRED', '', {'recover', 'preview_toggle'}),
])
def test_button_matrix_and_stop_do_not_depend_on_the_action_result(
        window, internal, operation, enabled):  # noqa: F811
    window.item_path.setText('item.yaml')
    window.node.status = status(
        state=internal, operation=operation, holding_item=internal in ('HOLDING', 'PAUSED'),
        configured=internal != 'UNCONFIGURED', startup_complete=internal != 'UNCONFIGURED',
        tray_position_recorded=True, manual_placement_enabled=True, can_return_item=True,
        operation_active=bool(operation),
        phase='TRAY_ACQUISITION_PAUSED' if internal == 'PAUSED' and operation == 'place' else '')
    if operation:
        window.result_future = Future()  # Place/Pick's original goal can still be pending.
    window._refresh()
    reasons = window._availability()
    assert {name for name, reason in reasons.items() if not reason} == enabled
    assert window.stop.isEnabled() and window.stop.text() == 'STOP'


@pytest.mark.parametrize('field,value', [
    ('feedback_fresh', False), ('motion_ready', False),
    ('tray_detector_ready', False), ('tray_position_recorded', False),
])
def test_lost_prerequisite_disables_place_and_rechecks_before_send(
        window, field, value):  # noqa: F811
    window.node.status = status(manual_placement_enabled=True, tray_position_recorded=True)
    window.node.action_clients['place'].send_goal_async = Mock()
    window._refresh()
    assert window.place_item.isEnabled()
    setattr(window.node.status, field, value)
    window._action('place')  # Status changed between paint and click.
    window.node.action_clients['place'].send_goal_async.assert_not_called()
    window._refresh()
    assert not window.place_item.isEnabled() and window.stop.isEnabled()
    assert window.place_item.toolTip()


@pytest.mark.parametrize('value', ['', '0', '-1', 'nan', 'inf', 'text'])
def test_invalid_place_inputs_are_disabled_before_click(window, value):  # noqa: F811
    window.node.status = status(manual_placement_enabled=True, tray_position_recorded=True)
    window.place_x.setText(value)
    window._refresh()
    assert not window.place_item.isEnabled()
    assert 'positive X/Y' in window.availability_details.text()


@pytest.mark.parametrize('internal', (
    'UNCONFIGURED', 'INACTIVE', 'READY', 'HOLDING', 'HOMING', 'PICKING', 'PLACING',
    'TRAY_POSITIONING', 'STARTING', 'RECOVERING', 'RETURNING_ITEM', 'PAUSING',
    'STOPPING', 'PAUSED', 'FAULT', 'HELD_UNKNOWN', 'RECOVERY_REQUIRED', 'OFFLINE'))
def test_permanent_stop_never_parks_or_returns_in_any_state(window, internal):  # noqa: F811
    window.node.status = None if internal == 'OFFLINE' else status(state=internal)
    stop = Mock(return_value=Future())
    window.node.service_clients['stop'] = SimpleNamespace(
        service_is_ready=lambda: True, call_async=stop)
    window.goal_handle = SimpleNamespace(cancel_goal_async=Mock())
    window.pending['return_item'] = Future()
    window._refresh()
    window.stop.click()
    stop.assert_called_once()
    window.goal_handle.cancel_goal_async.assert_called_once()
    assert window.stop.text() == 'STOP'


def test_pending_request_immediately_disables_conflicting_buttons(window):  # noqa: F811
    window.node.status = status(state='HOLDING', holding_item=True, tray_position_recorded=True)
    pause = Mock(return_value=Future())
    window.node.service_clients['pause'] = SimpleNamespace(
        service_is_ready=lambda: True, call_async=pause)
    window._refresh()
    window.pause.click()
    assert pause.call_count == 1
    assert not window.pause.isEnabled() and not window.home_button.isEnabled()
    assert not window.place_item.isEnabled() and window.stop.isEnabled()


def status_node(monitor):
    messages = []
    node = SimpleNamespace(
        status_publisher=SimpleNamespace(publish=messages.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=1))),
        machine=SimpleNamespace(state='READY', message='Idle'),
        configuration=SimpleNamespace(configuration_id='test', selection=object(),
                                      tray=SimpleNamespace(detect_joints=(0.,) * 6)),
        headless=False, holding_item=False, operation_lock=threading.Lock(),
        active_action='', phase='', waypoint='', candidate_index=0, candidate_total=0,
        managed=SimpleNamespace(session=None, can_return_item=lambda: False,
                                continue_block_reason=lambda _sample: 'Not paused'),
        global_speed_percent=50, global_cp_percent=100, startup_complete=True,
        _perception_ready=lambda _action: True,
        monitor=monitor)
    return node, messages


@pytest.mark.parametrize('joint', range(6))
def test_tray_status_uses_every_joint_and_idle_without_waits_or_commands(joint):
    rig = PositionRig()
    node, messages = status_node(rig.monitor)
    RobotController.publish_status(node)
    assert messages[-1].at_tray_detect and messages[-1].motion_ready
    joints = [0.] * 6
    joints[joint] = math.radians(1.01)
    rig.publish(joints=tuple(joints), idle=True)
    RobotController.publish_status(node)
    assert not messages[-1].at_tray_detect
    rig.publish(joints=(0.,) * 6, idle=False)
    RobotController.publish_status(node)
    assert not messages[-1].at_tray_detect and not messages[-1].motion_ready
    assert rig.waits == 0


def test_uncertain_release_disables_continue_without_mutating_placement(monkeypatch):
    node, _tray = acquisition_rig(monkeypatch, [None] * 10)
    exhaust(node)
    node.machine = ControllerStateMachine(initial='PAUSED')
    node.managed.parked_pose = node.hardware.current_pose()
    node.placement.release_issued = True
    node.placement.phase = 'APPROACH'
    before = list(node.log)
    reason = node.managed.continue_block_reason(node.monitor.snapshot())
    assert 'release unconfirmed' in reason
    assert node.log == before and not node.managed.resume.is_set()
    with pytest.raises(CommandRejected, match='release unconfirmed'):
        node.managed.continue_operation()
    assert not node.managed.resume.is_set()
