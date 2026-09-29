"""Read-only status telemetry, including stale feedback and raw gripper inputs."""

from types import SimpleNamespace
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
        item_detector_ready=True, tray_detector_ready=True, manual_placement_enabled=False)
    for name, value in fields.items():
        setattr(message, name, value)
    return message


@pytest.mark.parametrize('headless', [False, True])
def test_status_publishes_observed_feed_bits_and_clears_unavailable_telemetry(headless):
    feed = {
        "EnableStatus": 1, "RunningStatus": 1, "isRunQueuedCmd": 1,
        "ErrorStatus": 1, "CollisionStates": 1,
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
        managed=SimpleNamespace(session=None), global_speed_percent=60,
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
    assert message.manual_placement_enabled is not headless

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
        service_clients={name: client for name in (
            "configure", "startup", "recover", "pause", "continue", "stop", "return_item",
            "speed", "preview")})
    view = ControllerWindow(node)
    view.timer.stop()
    yield view
    view.close()
    view.deleteLater()
    app.processEvents()


def test_placement_controls_send_typed_offsets_and_rotation_only_when_held(window):
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
    assert not window.place_item.isEnabled()
    window.node.status = status(state="PLACING", holding_item=True, tray_position_recorded=True,
                                operation_active=True, operation="place")
    window._refresh()
    assert not window.place_x.isEnabled() and window.stop.text() == "PAUSE"


@pytest.mark.parametrize("inputs", [0, 1, 1 << 11, (1 << 11) | 1])
def test_gripper_leds_follow_two_raw_inputs_not_outputs_or_holding(window, inputs):
    window.node.status = status(digital_outputs=(1 << 12) | (1 << 13), digital_input_bits=inputs,
                                holding_item=True, robot_running=True, robot_queue_active=True)
    window._refresh()
    assert set(window.gripper_leds) == {1, 12}
    for channel in (1, 12):
        expected = "HIGH" if inputs & (1 << (channel - 1)) else "LOW"
        assert window.gripper_values[channel].text() == expected
        assert window.gripper_leds[channel].accessibleDescription() == expected
    assert window.status.text() == "READY"


def test_unavailable_feedback_never_displays_old_io_as_live_or_off(window):
    window.node.status = status(digital_outputs=1 << 12, digital_input_bits=1)
    window._refresh()
    assert window.gripper_values[1].text() == "HIGH"
    window.node.status.feedback_fresh = False
    window._refresh()
    assert all(value.text() == "UNKNOWN" for value in window.gripper_values.values())
    assert all(led.accessibleDescription() == "UNKNOWN" for led in window.gripper_leds.values())
    assert "Feedback: unavailable" in window.status.toolTip()
    window.node.status = None
    window._refresh()
    assert window.status.text() == "UNAVAILABLE"
    assert all(value.text() == "UNKNOWN" for value in window.gripper_values.values())
    assert not window.hardware_home.isEnabled() and not window.hardware_pick.isEnabled()
    assert window.stop.isEnabled() and window.stop.text() == "STOP"


def test_confirmed_estop_has_visible_feedback_without_latching_recover_disabled(window):
    window.node.status = status(state="FAULT", message="Recovery failed: " + EMERGENCY_STOP_GUIDANCE)
    window._refresh()
    assert window.status.text() == "EMERGENCY STOP\nPRESSED\nCannot start / recover"
    assert EMERGENCY_STOP_GUIDANCE in window.status.toolTip()
    assert not window.startup.isEnabled() and not window.hardware_home.isEnabled()
    # The alarm may latch until clear: keep explicit Recover available after the
    # physical button is released. Only the controller can establish readiness.
    assert window.recover.isEnabled()
    window.node.status = status(state="FAULT", message="Unrelated alarm")
    window._refresh()
    assert window.status.text() == "FAULT"
    window.node.status = status()
    window._refresh()
    assert window.status.text() == "READY"


def test_pause_button_matches_direct_stop_until_both_status_and_reply_arrive(window):
    commands = []
    pending = SimpleNamespace(done=lambda: False)

    def command(name):
        commands.append(name)
        window.pending[name] = pending
        return True
    window._command = command
    window.node.status = status(state="HOLDING", holding_item=True, can_return_item=True)
    window._refresh()
    assert window.stop.text() == "PAUSE"
    window.stop.click()
    assert commands == ["pause"] and window.stop.text() == "STOP NOW"
    window._refresh()
    assert window.stop.text() == "STOP NOW"
    window.node.status.state = "PAUSED"  # Topic can beat the Pause service reply.
    window._refresh()
    assert window.stop.text() == "STOP NOW"
    window.stop.click()
    assert commands == ["pause", "stop"]


def test_confirmed_paused_button_returns_item_then_offers_immediate_stop(window):
    commands = []
    window._command = lambda name: commands.append(name) or True
    window.node.status = status(state="PAUSED", holding_item=True, can_return_item=True)
    window._refresh()
    assert window.stop.text() == "RETURN ITEM & STOP"
    window.stop.click()
    assert commands == ["return_item"] and window.stop.text() == "STOP NOW"
    window.stop.click()
    assert commands == ["return_item", "stop"]


@pytest.mark.parametrize("inputs", [0, 1])
def test_unknown_held_state_allows_explicit_recovery_recheck(window, inputs):
    window.node.status = status(state="HELD_UNKNOWN", digital_input_bits=inputs)
    window._refresh()
    assert window.recover.isEnabled()
    assert not window.hardware_home.isEnabled() and not window.hardware_pick.isEnabled()
    window.pending["recover"] = SimpleNamespace(done=lambda: False)
    window._refresh()
    assert not window.recover.isEnabled()
    assert window.stop.isEnabled()


def test_recover_response_shows_held_instructions_once_without_issuing_commands(
        window, monkeypatch):
    prompts = []
    monkeypatch.setattr(QtWidgets.QMessageBox, "information",
                        lambda _parent, title, text: prompts.append((title, text)))
    window.pending["recover"] = SimpleNamespace(
        done=lambda: True,
        result=lambda: SimpleNamespace(success=True, state="HOLDING"))
    window._refresh()
    window._refresh()
    assert len(prompts) == 1
    assert "RETURN ITEM & STOP" in prompts[0][1]
    assert "STOP NOW" in prompts[0][1] and "obstruction" in prompts[0][1]


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
    window.node.status = status(item_detector_ready=ready, tray_detector_ready=False)
    window._refresh()
    assert window.hardware_pick.isEnabled() is ready
    assert window.hardware_home.isEnabled()
    if not ready:
        assert 'Arm Item Teach' in window.hardware_pick.toolTip()
    window.node.status.item_detector_ready = not ready
    window._refresh()
    assert window.hardware_pick.isEnabled() is not ready


@pytest.mark.parametrize('ready', [False, True])
def test_place_button_requires_tray_arming_and_a_held_item(window, ready):
    window.node.status = status(state='HOLDING', holding_item=True,
                                tray_position_recorded=True, tray_detector_ready=ready,
                                item_detector_ready=False)
    window._refresh()
    assert window.place_item.isEnabled() is ready
    assert window.tray_position.isEnabled()
    window.node.status.holding_item = False
    window.node.status.state = 'READY'
    window._refresh()
    assert not window.place_item.isEnabled()
    assert 'Pick an item successfully first' in window.place_item.toolTip()


@pytest.mark.parametrize('ready', [False, True])
@pytest.mark.parametrize('holding', [False, True])
def test_gui_debug_place_allows_empty_but_still_requires_tray_readiness(window, ready, holding):
    window.node.status = status(
        state='HOLDING' if holding else 'READY', holding_item=holding,
        manual_placement_enabled=True, tray_position_recorded=True, tray_detector_ready=ready)
    window._refresh()
    assert window.place_item.isEnabled() is ready
    if ready:
        assert 'with or without an item' in window.place_item.toolTip()
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
    window.node.status = status(item_detector_ready=False, tray_detector_ready=False)
    window.node.action_clients = {}  # Must return before accessing/sending any action.
    window._action(action)
    assert len(warnings) == 1 and warnings[0][1] == 'Detection unavailable'
    assert window.pending_goal is None
