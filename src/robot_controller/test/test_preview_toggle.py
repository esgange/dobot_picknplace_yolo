"""Qt preview routing cannot reach robot motion/settings clients."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rclpy.task import Future
from robot_controller_interfaces.srv import Command, Preview

import robot_controller.gui as gui_module
from test_status_ui import window, status  # noqa: F401


def resolved(result):
    future = Future()
    future.set_result(result)
    return future


@pytest.fixture
def controls(window, monkeypatch):  # noqa: F811 - imported pytest fixture
    monkeypatch.setattr(gui_module.QtWidgets.QMessageBox, "warning", Mock())
    window.item_path.setText("item.yaml")
    window.bin_path.setText("bin.yaml")
    window.tray_path.setText("tray.yaml")
    window.place_x.setText("30")
    window.place_y.setText("40")
    calls = []

    def send(request):
        calls.append(request)
        return resolved(Preview.Response(success=True, message="preview accepted"))
    window.node.service_clients["preview"] = SimpleNamespace(
        service_is_ready=lambda: True, call_async=Mock(side_effect=send), srv_name="preview_v2")
    window.node.action_clients = {name: SimpleNamespace(
        server_is_ready=lambda: True, send_goal_async=Mock(return_value=Future()))
        for name in ("home", "pick", "place")}
    window.preview_calls = calls
    window._refresh()
    return window


def test_operation_grid_has_exactly_the_four_requested_buttons(controls):
    grid = controls.centralWidget().layout().itemAt(2).layout()
    assert grid.itemAtPosition(0, 0).widget() is controls.home_button
    assert grid.itemAtPosition(0, 1).widget() is controls.preview_toggle
    assert grid.itemAtPosition(1, 0).widget() is controls.pick_item
    assert grid.itemAtPosition(1, 1).widget() is controls.place_item
    assert not any(hasattr(controls, old) for old in (
        "preview_home", "preview_pick", "tray_position"))
    assert not controls.preview_mode and not controls.preview_toggle.isChecked()


@pytest.mark.parametrize("name,operation", [
    ("home", Preview.Request.HOME), ("pick", Preview.Request.PICK),
    ("place", Preview.Request.PLACE)])
def test_preview_routes_each_motion_button_without_startup_or_hardware_goal(
        controls, name, operation):
    controls.node.status = status(state="INACTIVE", startup_complete=False, configured=False)
    controls.preview_toggle.setChecked(True)
    controls._action(name)
    request = controls.preview_calls[-1]
    assert request.operation == operation and request.item_teach_file == "item.yaml"
    assert request.bin_teach_file == "bin.yaml" and request.tray_teach_file == "tray.yaml"
    if name == "place":
        assert (request.x_mm, request.y_mm, request.rotation_deg) == (30., 40., 0.)
    for client in controls.node.action_clients.values():
        client.send_goal_async.assert_not_called()
    assert not controls.startup.isEnabled() and not controls.recover.isEnabled()
    assert not controls.speed_slider.isEnabled() and controls.stop.text() == "STOP"


@pytest.mark.parametrize("name", [
    "startup", "continue", "recover", "pause", "return_item", "speed"])
def test_preview_blocks_motion_and_settings_even_when_called_directly(controls, name):
    controls.preview_toggle.setChecked(True)
    assert not controls._call(name, Command.Request())
    # Fixture raises if any non-preview service client receives a command.


def test_preview_keeps_direct_stop_and_never_turns_stop_into_pause_or_return(controls):
    controls.preview_toggle.setChecked(True)
    stop = Mock(return_value=resolved(Command.Response(success=True)))
    controls.node.service_clients["stop"] = SimpleNamespace(
        service_is_ready=lambda: True, call_async=stop)
    controls._pause_or_stop()
    stop.assert_called_once()
    assert controls.preview_calls[-1].operation == Preview.Request.CLEAR


@pytest.mark.parametrize("state", ["PICKING", "PLACING", "PAUSED", "STOPPING"])
def test_cannot_switch_active_or_paused_hardware_operation_into_preview(controls, state):
    controls.node.status = status(state=state, operation_active=True)
    controls.preview_toggle.setChecked(True)
    assert not controls.preview_mode and not controls.preview_toggle.isChecked()
    assert not controls.preview_calls


def test_unavailable_preview_never_falls_back_to_hardware(controls):
    controls.preview_toggle.setChecked(True)
    controls.node.service_clients["preview"].service_is_ready = lambda: False
    controls._action("home")
    assert controls.preview_mode
    controls.node.action_clients["home"].send_goal_async.assert_not_called()
    gui_module.QtWidgets.QMessageBox.warning.assert_called_once()


def test_toggle_off_cancels_pending_preview_and_restores_hardware_dispatch(controls):
    controls.preview_toggle.setChecked(True)
    pending = Future()
    send = controls.node.service_clients["preview"].call_async
    send.side_effect = lambda request: (
        resolved(Preview.Response(success=True)) if request.operation == Preview.Request.CLEAR
        else pending)
    controls._action("home")
    assert "preview" in controls.pending
    controls.preview_toggle.setChecked(False)
    assert pending.cancelled() and "preview" not in controls.pending
    pending.set_result(Preview.Response(success=True, message="obsolete"))
    controls._refresh()
    assert "obsolete" not in controls.feedback_message
    assert controls.home_button.toolTip() == "Move the robot to taught Home"
    controls._action("home")
    controls.node.action_clients["home"].send_goal_async.assert_called_once()


def test_editing_placement_target_clears_old_preview(controls):
    controls.preview_toggle.setChecked(True)
    controls._action("place")
    controls.place_rotation.setValue(90.)
    assert controls.preview_calls[-1].operation == Preview.Request.CLEAR
    assert "preview" not in controls.pending


def test_preview_clear_timeout_is_bounded_and_never_dispatches_motion(controls, monkeypatch):
    controls.preview_clear_future = Future()
    future = controls.preview_clear_future
    controls.preview_clear_deadline = 1.
    monkeypatch.setattr(gui_module.time, "monotonic", lambda: 2.)
    controls._collect()
    assert future.cancelled() and controls.preview_clear_future is None
    for client in controls.node.action_clients.values():
        client.send_goal_async.assert_not_called()
