"""Read-only status telemetry, including stale feedback and raw gripper inputs."""

from types import SimpleNamespace
import threading

from builtin_interfaces.msg import Time
from PyQt5 import QtWidgets
import pytest

from robot_controller.controller import RobotController
from robot_controller.errors import FeedbackFailure
from robot_controller.gui import ControllerWindow, GuiNode
import robot_controller.gui as gui_module
from robot_controller_interfaces.msg import ControllerStatus


def status(**fields):
    message = ControllerStatus(
        state="READY", message="Ready to pick", configured=True,
        configuration_id="test-configuration", startup_complete=True,
        feedback_fresh=True, robot_enabled=True, global_speed_percent=60)
    for name, value in fields.items():
        setattr(message, name, value)
    return message


def test_status_publishes_observed_feed_bits_and_clears_unavailable_telemetry():
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
        configuration=None, holding_item=True, operation_lock=threading.Lock(),
        active_action="", phase="", waypoint="", candidate_index=1, candidate_total=2,
        managed=SimpleNamespace(session=None), global_speed_percent=60,
        startup_complete=True, expected_outputs={13: False, 14: False},
        monitor=SimpleNamespace(snapshot=lambda **_kwargs: sample))
    RobotController.publish_status(node)
    message = messages[-1]
    assert message.feedback_fresh and message.robot_enabled
    assert message.robot_running and message.robot_queue_active
    assert message.robot_error and message.robot_collision
    assert message.digital_outputs == feed["digital_outputs"]
    assert message.digital_input_bits == 1 << 11  # Raw DI1 LOW despite debounced holding.
    assert message.holding_item

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


@pytest.mark.parametrize("outputs,inputs,vacuum,fingers", [
    (0, 0, "NEUTRAL", "NEUTRAL"),
    ((1 << 12) | (1 << 13), (1 << 11) | 1, "SUCK", "OPEN"),
    (1 | (1 << 1), 0, "EXHAUST", "CLOSE"),
    (1 | (1 << 1) | (1 << 12) | (1 << 13), 0, "CONFLICT", "CONFLICT"),
])
def test_live_io_reports_canonical_outputs_and_raw_sensors(
        window, outputs, inputs, vacuum, fingers):
    window.node.status = status(digital_outputs=outputs, digital_input_bits=inputs,
                                holding_item=True, robot_running=True, robot_queue_active=True)
    window._refresh()
    text = window.gripper_status.text()
    assert f"Vacuum: {vacuum}" in text and f"Finger outputs: {fingers}" in text
    assert "Held context: YES" in text
    assert f"DI1 suction {'HIGH' if inputs & 1 else 'LOW'}" in text
    assert f"DI12 fully open {'HIGH' if inputs & (1 << 11) else 'LOW'}" in text
    for channel, label in ((1, "exhaust"), (2, "close"), (13, "suction"), (14, "open")):
        assert f"DO{channel} {label} {'ON' if outputs & (1 << (channel - 1)) else 'OFF'}" in text
    assert "ENABLED · Moving · Queue active" in window.status.text()


def test_unavailable_feedback_never_displays_old_io_as_live_or_off(window):
    window.node.status = status(digital_outputs=1 << 12, digital_input_bits=1)
    window._refresh()
    assert "DO13 suction ON" in window.gripper_status.text()
    window.node.status.feedback_fresh = False
    window._refresh()
    assert "UNKNOWN" in window.gripper_status.text()
    assert "DO13" not in window.gripper_status.text()
    assert "ENABLED" not in window.status.text()
    window.node.status = None
    window._refresh()
    assert "UNAVAILABLE" in window.status.text()
    assert "UNKNOWN" in window.gripper_status.text()
    assert not window.hardware_home.isEnabled() and not window.hardware_pick.isEnabled()
    assert window.stop.isEnabled() and window.stop.text() == "STOP"
