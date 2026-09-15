"""Global speed uses synthetic responses only; never connect to hardware."""

import threading
from types import MethodType, SimpleNamespace as NS
from unittest.mock import MagicMock

import pytest
from dobot_msgs_v4.srv import SpeedFactor
from rclpy.task import Future

from robot_controller import controller, hardware
from test_controller import pair as _pair_fixture
from test_execution import synthetic_transport, settings, plan


pair = _pair_fixture


def speed_node(monkeypatch):
    transport, feed, _, clock = synthetic_transport(monkeypatch)
    node = transport.node
    node.hardware = transport
    node.state_lock, node.action_lock = threading.RLock(), threading.Lock()
    node.action_thread = node.stop_thread = node.stop_future = None
    node.live, node.debug, node.headless = True, False, True
    node.startup_settings_applied, node.holding_item = True, False
    node.global_speed_percent, node.global_speed_message = 100, "Startup confirmed"
    node.execution_state, node.execution_message = "READY", "Ready"
    node.fatal_error, node.shutdown_requested = None, threading.Event()
    node._publish = MagicMock()
    node.set_execution_state = MethodType(controller.RobotController.set_execution_state, node)

    def cancelled():
        if node.cancel.is_set() or node.shutdown_requested.is_set():
            raise ValueError("Controller action cancelled")

    node.check_cancelled = cancelled
    return node, feed, clock


def invoke(node, ratio):
    return controller.RobotController._set_global_speed_service(
        node, NS(ratio=ratio), SpeedFactor.Response())


@pytest.mark.parametrize("ratio", [1, 37, 100])
@pytest.mark.parametrize("state", ["READY", "NO_PICK", "HOLDING"])
def test_valid_global_speed_changes_only_speedfactor_and_preserves_idle_state(
        monkeypatch, ratio, state):
    node, feed, _ = speed_node(monkeypatch)
    node.execution_state = state
    node.holding_item = state == "HOLDING"
    feed["digital_input_bits"] = int(node.holding_item)
    cfg = settings()
    before = [(target.speed_percent, target.acceleration_percent) for target in plan(cfg=cfg)]
    response = invoke(node, ratio)
    assert response.res == 0
    assert node.global_speed_percent == ratio and node.execution_state == state
    assert not node.action_lock.locked() and not node.cancel.is_set()
    node.hardware.clients["SpeedFactor"].call_async.assert_called_once()
    assert node.hardware.clients["SpeedFactor"].call_async.call_args.args[0].ratio == ratio
    assert all(not client.call_async.called for name, client in node.hardware.clients.items()
               if name != "SpeedFactor")
    assert before == [(target.speed_percent, target.acceleration_percent)
                      for target in plan(cfg=cfg)]
    assert node.live and node.headless  # Same service works headlessly.


@pytest.mark.parametrize("ratio", [0, 101, -1, True, 1.0, "50", None])
def test_invalid_global_speed_never_dispatches(monkeypatch, ratio):
    node, _, _ = speed_node(monkeypatch)
    assert invoke(node, ratio).res == -1
    assert node.global_speed_percent == 100 and node.execution_state == "READY"
    assert not node.action_lock.locked()
    node.hardware.clients["SpeedFactor"].call_async.assert_not_called()


@pytest.mark.parametrize("blocker", [
    "live_off", "startup", "fatal", "shutdown", "initializing", "failed", "moving",
    "action", "recovery", "action_lock", "stop_pending", "cancel", "owner",
    "stale", "queued", "running", "mode", "lost_suction", "response_pending",
])
def test_unsafe_speed_change_is_rejected_without_robot_commands(monkeypatch, blocker):
    node, feed, _ = speed_node(monkeypatch)
    transport = node.hardware
    locked = blocker == "action_lock"
    if blocker == "live_off":
        node.live, node.debug, node.hardware = False, True, None
    elif blocker == "startup":
        node.startup_settings_applied = False
    elif blocker == "fatal":
        node.fatal_error = "Initialization failed"
    elif blocker == "shutdown":
        node.shutdown_requested.set()
    elif blocker in ("initializing", "failed"):
        node.execution_state = blocker.upper()
    elif blocker == "moving":
        transport.moving = True
    elif blocker in ("action", "recovery"):
        setattr(node, "action_thread" if blocker == "action" else "stop_thread",
                NS(is_alive=lambda: True))
    elif locked:
        node.action_lock.acquire()
    elif blocker == "stop_pending":
        node.stop_future = Future()
    elif blocker == "cancel":
        node.cancel.set()
    elif blocker == "owner":
        node.check_command_owner.side_effect = ValueError("competing command owner")
    elif blocker == "stale":
        node.feedback_snapshot = MagicMock(side_effect=ValueError("stale feedback"))
    elif blocker in ("queued", "running", "mode"):
        key = {"queued": "isRunQueuedCmd", "running": "RunningStatus", "mode": "robot_mode"}
        feed[key[blocker]] = 7 if blocker == "mode" else 1
    elif blocker == "lost_suction":
        node.holding_item, node.execution_state = True, "HOLDING"
    elif blocker == "response_pending":
        transport.pending_response = "MovLIO", Future()
    assert invoke(node, 50).res == -1
    transport.clients["SpeedFactor"].call_async.assert_not_called()
    assert node.action_lock.locked() is locked
    if locked:
        node.action_lock.release()
    assert "rejected" in node.global_speed_message or "request failed" in node.global_speed_message


def test_global_speed_waits_for_actual_response_and_reports_unknown_while_pending(monkeypatch):
    node, _, clock = speed_node(monkeypatch)
    future = Future()
    node.hardware.clients["SpeedFactor"].call_async.return_value = future

    def advance(dt):
        assert node.action_lock.locked() and node.execution_state == "SPEED_SETTING"
        assert node.global_speed_percent is None
        clock[0] += dt
        if clock[0] >= .12:
            future.set_result(NS(res=0))

    monkeypatch.setattr(hardware.time, "sleep", advance)
    assert invoke(node, 22).res == 0 and clock[0] >= .12
    assert node.global_speed_percent == 22 and node.execution_state == "READY"


@pytest.mark.parametrize("failure", ["returned_failure", "timeout", "cancel", "running", "stop"])
def test_failed_or_ambiguous_speed_response_never_retries_or_restores_ready(monkeypatch, failure):
    node, feed, clock = speed_node(monkeypatch)
    future = Future()
    node.hardware.clients["SpeedFactor"].call_async.return_value = future
    if failure == "returned_failure":
        future.set_result(NS(res=-3))
    elif failure in ("cancel", "running", "stop"):
        def advance(dt):
            clock[0] += dt
            if failure in ("cancel", "stop"):
                node.cancel.set()
                if failure == "stop":
                    assert node.hardware.stop().result().res == 0
            else:
                feed["RunningStatus"] = 1
        monkeypatch.setattr(hardware.time, "sleep", advance)
    assert invoke(node, 25).res == -1
    assert node.execution_state == "FAILED" and node.cancel.is_set()
    assert node.global_speed_percent is None and not node.action_lock.locked()
    assert invoke(node, 30).res == -1
    node.hardware.clients["SpeedFactor"].call_async.assert_called_once()
    if failure == "stop":
        node.hardware.clients["Stop"].call_async.assert_called_once()
    if failure == "timeout":
        assert clock[0] >= hardware.SERVICE_TIMEOUT_SEC
        future.set_result(NS(res=0))  # Late response cannot silently restore READY/factor.
        assert node.global_speed_percent is None and node.execution_state == "FAILED"


def test_live_initialization_resets_previous_global_factor_to_100(monkeypatch):
    node, _, _ = speed_node(monkeypatch)
    assert invoke(node, 40).res == 0
    node.hardware.initialize()
    ratios = [call.args[0].ratio for call in
              node.hardware.clients["SpeedFactor"].call_async.call_args_list]
    assert ratios == [40, 100] and node.global_speed_percent == 100


def test_global_slider_range_debounce_gating_and_confirmed_value(pair, monkeypatch):
    import rclpy
    from PyQt5 import QtWidgets
    from rclpy.executors import MultiThreadedExecutor
    from robot_controller.gui import ControllerWindow
    root, _, _ = pair
    monkeypatch.setattr(controller, "workspace_root", lambda: root)
    monkeypatch.setattr(controller, "load_robot_lan1_ip", lambda _: "192.168.20.204")
    monkeypatch.setattr(controller, "_parse_env_file", lambda _: {
        "DOBOT_ROBOT_NODE_NAME": "dobot_bringup_ros2"})
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    assert application is not None
    rclpy.init(args=[])
    node, window, executor = None, None, None
    try:
        node = controller.RobotController()
        window = ControllerWindow(node)
        assert window.global_speed.minimum() == 1 and window.global_speed.maximum() == 100
        assert not window.global_speed.isEnabled() and node.global_speed_percent is None
        assert (window.service_clients["global_speed"].srv_name
                == "/robot_controller/set_global_speed")
        node.live, node.debug, node.startup_settings_applied = True, False, True
        node.execution_state, node.global_speed_percent = "READY", 100
        node.hardware = NS(moving=False)
        window._call_service = MagicMock()
        window.refresh()
        assert window.global_speed.isEnabled()
        window.global_speed.setValue(33)
        assert window.speed_timer.isActive()
        window.refresh()
        assert window.global_speed.value() == 33  # Refresh cannot overwrite a pending edit.
        window.speed_timer.stop()
        window.set_global_speed()
        name, request = window._call_service.call_args.args
        assert name == "global_speed" and request.ratio == 33
        assert isinstance(request, SpeedFactor.Request)
        node.global_speed_percent = 33
        window.refresh()
        assert "33%" in window.mode.text() and window.global_speed.value() == 33
        future = Future()
        window.pending_calls["global_speed"] = future
        window.refresh()
        assert not window.global_speed.isEnabled() and not window.home.isEnabled()
        future.set_result(SpeedFactor.Response(res=0))
        window.refresh()
        assert window.global_speed.isEnabled()
        window.global_speed.setValue(44)
        node.execution_state = "BUSY"
        window.refresh()
        assert not window.global_speed.isEnabled() and not window.speed_timer.isActive()
        node.hardware = None
        node.live, node.debug = False, True
        window.refresh()
        assert "Live OFF" in window.global_speed_label.text()
        assert node.global_speed_percent == 33  # UI never issues hardware calls privately.
        # This is a transient command, not saved profile/operator setup.
        assert not (root / "logs/robot_controller/last_session.json").exists()
        # Verify the public request/response wire type with a real isolated ROS
        # executor, but keep the vendor transport entirely synthetic.
        node.live, node.debug = True, False
        node.execution_state = "READY"
        node.cancel.clear()
        node.check_command_owner = MagicMock()
        node.feedback_snapshot = MagicMock(return_value={"feed": {"digital_input_bits": 0}})

        def set_factor(ratio):
            assert node.action_lock.locked() and node.execution_state == "SPEED_SETTING"
            node.global_speed_percent = ratio

        node.hardware = NS(moving=False, set_global_speed=set_factor)
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        future = window.service_clients["global_speed"].call_async(SpeedFactor.Request(ratio=18))
        executor.spin_until_future_complete(future, timeout_sec=2)
        assert future.done() and future.result().res == 0
        assert node.global_speed_percent == 18
        window.refresh()
        assert window.global_speed.value() == 18 and "18%" in window.mode.text()
        assert len(list(node.clients)) == 7  # Pose + GUI clients; no vendor command clients.
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2)
        if window is not None:
            window.close()
        if node is not None:
            node.hardware = None
            node.close_runtime()
            node.destroy_node()
        rclpy.shutdown()
