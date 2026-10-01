"""Read-only status telemetry, including stale feedback and raw gripper inputs."""

from types import SimpleNamespace
from unittest.mock import Mock
import threading

from builtin_interfaces.msg import Time
from PyQt5 import QtWidgets
import pytest

from robot_controller.controller import RobotController
from robot_controller.errors import FeedbackFailure
from robot_controller.errors import EMERGENCY_STOP_GUIDANCE
from robot_controller.gui import ControllerWindow, GuiNode
import robot_controller.gui as gui_module
from robot_controller_interfaces.msg import ControllerStatus


def status(**fields):
    message = ControllerStatus(
        state="READY", message="Ready to pick", configured=True,
        configuration_id="test-configuration", startup_complete=True,
        feedback_fresh=True, robot_enabled=True, global_speed_percent=60,
        item_detector_ready=True, tray_detector_ready=True, manual_placement_enabled=False,
        configuration_editable=True, pick_configured=True, motion_ready=True,
        preview_ready=True, at_tray_detect=True, can_continue=True)
    for name, value in fields.items():
        setattr(message, name, value)
    return message


def test_pick_requires_recorded_tray_position_even_with_item_detector_ready(window, monkeypatch):
    warnings = []
    monkeypatch.setattr(QtWidgets.QMessageBox, 'warning', lambda *_args: warnings.append(_args))
    window.node.status = status(tray_position_recorded=False)
    window.node.action_clients = {}
    window._refresh()
    assert not window.pick_item.isEnabled()
    assert 'recorded Tray Detect Pose' in window.pick_item.toolTip()
    window._action('pick')
    assert not warnings  # Disabled actions do not send a request or open a refusal dialog.


@pytest.mark.parametrize('headless', [False, True])
def test_status_publishes_observed_feed_bits_and_clears_unavailable_telemetry(headless):
    feed = {
        "EnableStatus": 1, "RunningStatus": 1, "isRunQueuedCmd": 1,
        "ErrorStatus": 1, "CollisionStates": 1,
        "robot_mode": 9, "userCoordinate": 0, "toolCoordinate": 0,
        "digital_input_bits": 1 << 11, "digital_outputs": (1 << 12) | (1 << 13)}
    sample = SimpleNamespace(feed=feed, robot_enabled=False, suction_present=True)
    messages = []
    node = SimpleNamespace(
        status_publisher=SimpleNamespace(publish=messages.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: Time(sec=100))),
        machine=SimpleNamespace(state="HOLDING", message="Holding"),
        configuration=None, headless=headless, holding_item=True, operation_lock=threading.Lock(),
        active_action="", phase="", waypoint="", candidate_index=1, candidate_total=2,
        managed=SimpleNamespace(session=None, can_return_item=lambda: False,
                                continue_block_reason=lambda _sample: "Not paused"),
        global_speed_percent=60,
        startup_complete=True, expected_outputs={13: False, 14: False},
        _perception_ready=lambda action: action == "pick",
        monitor=SimpleNamespace(snapshot=lambda **_kwargs: sample))
    RobotController.publish_status(node)
    message = messages[-1]
    assert message.feedback_fresh and message.robot_enabled
    assert message.robot_running and message.robot_queue_active
    assert message.robot_error and message.robot_collision
    assert message.digital_outputs == feed["digital_outputs"]
    assert message.digital_input_bits == 1 << 11  # Raw DI1 LOW despite debounced holding.
    assert message.holding_item
    assert message.item_detector_ready and not message.tray_detector_ready
    assert message.manual_placement_enabled

    def stale(**_kwargs):
        raise FeedbackFailure("stale")
    node.monitor.snapshot = stale
    RobotController.publish_status(node)
    stale_message = messages[-1]
    assert not stale_message.feedback_fresh
    assert stale_message.digital_input_bits == stale_message.digital_outputs == 0
    assert not stale_message.robot_enabled and not stale_message.robot_running
    assert stale_message.holding_item  # Logical holding is separate from live I/O.


@pytest.mark.parametrize("source,ros_now,received,now,available", [
    (100, 100.2, 10., 10.2, True),
    (100, 100.2, 10., 11.01, False),  # No updates, even with frozen ROS time.
    (100, 101.01, 11., 11.01, False),  # An old transient-local sample just arrived.
    (101, 100., 10., 10.1, False),
    (0, 0., 10., 10.1, False),
])
def test_gui_rejects_expired_or_invalid_status(
        source, ros_now, received, now, available, monkeypatch):
    message = status()
    message.header.stamp.sec = source
    node = SimpleNamespace(
        _status_sample=(message, received),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int(ros_now * 1e9))))
    monkeypatch.setattr(gui_module.time, "monotonic", lambda: now)
    assert (GuiNode.status.fget(node) is message) is available


@pytest.fixture
def window(tmp_path):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda _request: pytest.fail("Status rendering sent a command"))
    node = SimpleNamespace(
        root=tmp_path, prefill=None, status=status(), take_operator_logs=lambda: (),
        action_clients={name: SimpleNamespace(server_is_ready=lambda: True)
                        for name in ("home", "pick", "place")},
        service_clients={name: client for name in (
            "configure", "startup", "recover", "pause", "continue", "stop", "return_item",
            "speed", "preview")})
    view = ControllerWindow(node)
    view.timer.stop()
    view.place_x.setText("30")
    view.place_y.setText("40")
    yield view
    view.close()
    view.deleteLater()
    app.processEvents()


def test_placement_controls_send_typed_offsets_and_allow_empty_or_held(window):
    sent = []
    window.node.action_clients = {"place": SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=lambda goal, **kwargs: sent.append(goal) or object())}
    window.node.status = status(state="HOLDING", holding_item=True, tray_position_recorded=True)
    window._refresh()
    assert window.place_item.isEnabled()
    assert window.place_rotation.minimum() == -180 and window.place_rotation.maximum() == 180
    window.place_x.setText("30")
    window.place_y.setText("40")
    window.place_rotation.setValue(-90)
    window._action("place")
    assert len(sent) == 1
    assert (sent[0].x_mm, sent[0].y_mm, sent[0].rotation_deg) == (30., 40., -90.)
    assert sent[0].configuration_id == "test-configuration"
    window.pending_goal = None
    window.node.status = status(state="READY", holding_item=False, tray_position_recorded=True)
    window._refresh()
    assert window.place_item.isEnabled()
    window.node.status = status(state="PLACING", holding_item=True, tray_position_recorded=True,
                                operation_active=True, operation="place")
    window._refresh()
    assert not window.place_x.isEnabled() and window.pause.text() == "PAUSE"


@pytest.mark.parametrize("inputs", [0, 1, 1 << 11, (1 << 11) | 1])
def test_gripper_leds_follow_two_raw_inputs_not_outputs_or_holding(window, inputs):
    window.node.status = status(digital_outputs=(1 << 12) | (1 << 13), digital_input_bits=inputs,
                                holding_item=True, robot_running=True, robot_queue_active=True)
    window._refresh()
    assert set(window.gripper_leds) == {1, 12}
    for channel in (1, 12):
        expected = "Detected" if inputs & (1 << (channel - 1)) else "Not detected"
        assert window.gripper_values[channel].text() == expected
        assert window.gripper_leds[channel].accessibleDescription() == expected
    assert window.status.text() == "READY"


def test_unavailable_feedback_never_displays_old_io_as_live_or_off(window):
    window.node.status = status(digital_outputs=1 << 12, digital_input_bits=1)
    window._refresh()
    assert window.gripper_values[1].text() == "Detected"
    window.node.status.feedback_fresh = False
    window._refresh()
    assert all(value.text() == "Unknown" for value in window.gripper_values.values())
    assert all(led.accessibleDescription() == "Unknown" for led in window.gripper_leds.values())
    assert "Feedback: unavailable" in window.status.toolTip()
    window.node.status = None
    window._refresh()
    assert window.status.text() == "OFFLINE"
    assert all(value.text() == "Unknown" for value in window.gripper_values.values())
    assert not window.home_button.isEnabled() and not window.pick_item.isEnabled()
    assert window.stop.isEnabled() and window.stop.text() == "STOP"


def test_confirmed_estop_has_visible_feedback_without_latching_recover_disabled(window):
    window.node.status = status(
        state="FAULT", message="Recovery failed: " + EMERGENCY_STOP_GUIDANCE)
    window._refresh()
    assert window.status.text() == "EMERGENCY STOP PRESSED"
    assert EMERGENCY_STOP_GUIDANCE in window.status.toolTip()
    assert not hasattr(window, "startup") and not window.home_button.isEnabled()
    # The alarm may latch until clear: keep explicit Recover available after the
    # physical button is released. Only the controller can establish readiness.
    assert window.recover.isEnabled()
    window.node.status = status(state="FAULT", message="Unrelated alarm")
    window._refresh()
    assert window.status.text() == "ATTENTION REQUIRED"
    window.node.status = status()
    window._refresh()
    assert window.status.text() == "READY"


def test_permanent_stop_remains_available_until_pause_status_and_reply_arrive(window):
    commands = []
    pending = SimpleNamespace(done=lambda: False)

    def command(name):
        commands.append(name)
        window.pending[name] = pending
        return True
    window._command = command
    window.node.status = status(state="HOLDING", holding_item=True, can_return_item=True)
    window._refresh()
    assert window.pause.text() == "PAUSE"
    window.pause.click()
    assert commands == ["pause"] and not window.pause.isEnabled() and window.stop.text() == "STOP"
    window._refresh()
    assert not window.pause.isEnabled() and window.stop.text() == "STOP"
    window.node.status.state = "PAUSED"  # Topic can beat the Pause service reply.
    window._refresh()
    assert not window.pause.isEnabled() and window.stop.text() == "STOP"
    window.stop.click()
    assert commands == ["pause", "stop"]


def test_confirmed_paused_return_preserves_the_separate_immediate_stop(window):
    commands = []
    window._command = lambda name: commands.append(name) or True
    window.node.status = status(state="PAUSED", holding_item=True, can_return_item=True)
    window._refresh()
    assert window.pause.text() == "RETURN ITEM"
    window.pause.click()
    assert commands == ["return_item"] and not window.pause.isEnabled()
    assert window.stop.text() == "STOP"
    window.stop.click()
    assert commands == ["return_item", "stop"]


def acquisition_paused_status(**fields):
    values = dict(state="PAUSED", operation="place", phase="TRAY_ACQUISITION_PAUSED",
                  operation_active=True, holding_item=True, can_return_item=True,
                  tray_position_recorded=True)
    values.update(fields)
    return status(**values)


def test_failed_acquisition_place_button_retries_while_original_action_is_pending(window):
    commands = []
    pending = SimpleNamespace(done=lambda: False)
    window.result_future = pending
    window.node.status = acquisition_paused_status()

    def command(name):
        commands.append(name)
        window.pending[name] = pending
        return True
    window._call = lambda name, _request: command(name)
    window._refresh()
    assert window.place_item.isEnabled()
    assert window.place_item.text() == "Place Item (Retry)"
    assert not window.pick_item.isEnabled()
    assert not window.home_button.isEnabled() and not window.preview_toggle.isEnabled()
    assert not hasattr(window, "startup")
    assert window.pause.text() == "RETURN ITEM"
    window.place_item.click()
    assert commands == ["continue"] and window.result_future is pending
    window._refresh()
    assert not window.place_item.isEnabled()
    window._action("place")
    assert commands == ["continue"]


@pytest.mark.parametrize("blocker", ["provider", "stop", "return", "ordinary_pause"])
def test_acquisition_retry_button_cannot_bypass_its_guards(window, blocker):
    commands = []
    window._command = lambda name: commands.append(name) or True
    window.result_future = SimpleNamespace(done=lambda: False)
    window.node.status = acquisition_paused_status()
    if blocker == "provider":
        window.node.status.tray_detector_ready = False
    elif blocker == "stop":
        window.pending["stop"] = SimpleNamespace(done=lambda: False)
    elif blocker == "return":
        window.return_requested_locally = True
    else:
        window.node.status.phase = "MOTION"
    window._refresh()
    assert not window.place_item.isEnabled()
    window._action("place")
    assert commands == []


def test_acquisition_return_uses_same_paused_control_and_then_direct_stop(window):
    commands = []
    pending = SimpleNamespace(done=lambda: False)
    window.result_future = pending
    window.goal_handle = SimpleNamespace(cancel_goal_async=Mock())
    window._command = lambda name: commands.append(name) or True
    window.node.status = acquisition_paused_status()
    window._refresh()
    window._managed_command("continue")
    assert commands == []
    window.pause.click()
    assert commands == ["return_item"]
    assert not window.pause.isEnabled() and window.stop.text() == "STOP"
    window.goal_handle.cancel_goal_async.assert_not_called()
    assert window.result_future is pending
    window._refresh()
    assert not window.place_item.isEnabled() and not window.pick_item.isEnabled()
    assert not window.pause.isEnabled() and window.stop.text() == "STOP"
    window.stop.click()
    assert commands == ["return_item", "stop"]
    window.goal_handle.cancel_goal_async.assert_called_once()


def test_empty_acquisition_pause_has_retry_and_direct_stop_without_invented_return(window):
    commands = []
    window._command = lambda name: commands.append(name) or True
    window.node.status = acquisition_paused_status(holding_item=False, can_return_item=False)
    window._refresh()
    assert window.place_item.isEnabled()
    assert not window.pick_item.isEnabled()
    assert window.stop.text() == "STOP"
    window.stop.click()
    assert commands == ["stop"]


def test_acquisition_return_stays_available_without_tray_detector(window):
    window.node.status = acquisition_paused_status(tray_detector_ready=False)
    window._refresh()
    assert not window.place_item.isEnabled() and not window.pick_item.isEnabled()
    assert window.pause.text() == "RETURN ITEM" and window.stop.isEnabled()


def test_acquisition_retry_label_resets_after_return_to_ready(window):
    window.node.status = acquisition_paused_status()
    window._refresh()
    assert window.place_item.text() == "Place Item (Retry)"
    window.node.status = status(state="READY", tray_position_recorded=True)
    window._refresh()
    assert window.place_item.text() == "Place Item"


@pytest.mark.parametrize("inputs", [0, 1])
def test_unknown_held_state_allows_explicit_recovery_recheck(window, inputs):
    window.node.status = status(state="HELD_UNKNOWN", digital_input_bits=inputs)
    window._refresh()
    assert window.recover.isEnabled()
    assert not window.home_button.isEnabled() and not window.pick_item.isEnabled()
    window.pending["recover"] = SimpleNamespace(done=lambda: False)
    window._refresh()
    assert not window.recover.isEnabled()
    assert window.stop.isEnabled()


def test_recover_response_shows_relaxed_gripper_once_without_issuing_commands(
        window, monkeypatch):
    prompts = []
    monkeypatch.setattr(QtWidgets.QMessageBox, "information",
                        lambda _parent, title, text: prompts.append((title, text)))
    window.pending["recover"] = SimpleNamespace(
        done=lambda: True,
        result=lambda: SimpleNamespace(success=True, state="READY"))
    window._refresh()
    window._refresh()
    assert len(prompts) == 1
    assert "Fingers are relaxed" in prompts[0][1]
    assert "suction and exhaust are OFF" in prompts[0][1]
    assert "DI1 is LOW" in prompts[0][1] and "action was cancelled" in prompts[0][1]


def test_recovery_unknown_suction_refusal_displays_server_clearing_instructions(
        window, monkeypatch):
    from robot_controller.errors import UNKNOWN_ITEM_GUIDANCE

    prompts = []
    monkeypatch.setattr(QtWidgets.QMessageBox, "warning",
                        lambda _parent, _title, text: prompts.append(text))
    window.pending["recover"] = SimpleNamespace(
        done=lambda: True,
        result=lambda: SimpleNamespace(success=False, state="HELD_UNKNOWN",
                                       message=UNKNOWN_ITEM_GUIDANCE))
    window._refresh()
    assert prompts == [UNKNOWN_ITEM_GUIDANCE]


@pytest.mark.parametrize('ready', [False, True])
def test_pick_button_tracks_item_arming_independently_of_tray(window, ready):
    window.node.status = status(item_detector_ready=ready, tray_detector_ready=False,
                                tray_position_recorded=True)
    window._refresh()
    assert window.pick_item.isEnabled() is ready
    assert window.home_button.isEnabled()
    if not ready:
        assert 'Arm Item Teach' in window.pick_item.toolTip()
    window.node.status.item_detector_ready = not ready
    window._refresh()
    assert window.pick_item.isEnabled() is not ready


@pytest.mark.parametrize('ready', [False, True])
def test_place_button_requires_tray_arming_for_empty_or_held_robot(window, ready):
    window.node.status = status(state='HOLDING', holding_item=True,
                                tray_position_recorded=True, tray_detector_ready=ready,
                                item_detector_ready=False)
    window._refresh()
    assert window.place_item.isEnabled() is ready
    assert not hasattr(window, "tray_position")
    window.node.status.holding_item = False
    window.node.status.state = 'READY'
    window._refresh()
    assert window.place_item.isEnabled() is ready


@pytest.mark.parametrize('ready', [False, True])
@pytest.mark.parametrize('holding', [False, True])
def test_gui_debug_place_allows_empty_but_still_requires_tray_readiness(window, ready, holding):
    window.node.status = status(
        state='HOLDING' if holding else 'READY', holding_item=holding,
        manual_placement_enabled=True, tray_position_recorded=True, tray_detector_ready=ready)
    window._refresh()
    assert window.place_item.isEnabled() is ready
    if ready:
        assert 'place and retract' in window.place_item.toolTip()
    window.node.status.operation_active = True
    window._refresh()
    assert not window.place_item.isEnabled()
    window.node.status.operation_active = False
    window.node.status.tray_position_recorded = False
    window._refresh()
    assert not window.place_item.isEnabled()


@pytest.mark.parametrize('action', ['pick', 'place'])
def test_gui_rechecks_provider_status_before_sending_goal(window, action, monkeypatch):
    warnings = []
    monkeypatch.setattr(QtWidgets.QMessageBox, 'warning', lambda *_args: warnings.append(_args))
    window.node.status = status(item_detector_ready=False, tray_detector_ready=False,
                                tray_position_recorded=True)
    window.node.action_clients = {}  # Must return before accessing/sending any action.
    window._action(action)
    assert not warnings
    assert window.pending_goal is None
