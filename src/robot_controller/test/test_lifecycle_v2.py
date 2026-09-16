from types import SimpleNamespace
import threading

from PyQt5 import QtWidgets
from rclpy.action import GoalResponse

from robot_controller.controller import RobotController
from robot_controller.gui import ControllerWindow, GuiNode
from robot_controller.state_machine import ControllerStateMachine


class Config:
    configuration_id = "active"
    selection = object()


def reservation(state, *, startup=True, holding=False, selection=True):
    config = Config()
    if not selection:
        config.selection = None
    calls = []
    node = SimpleNamespace(
        configuration=config, startup_complete=startup,
        machine=ControllerStateMachine(initial=state), holding_item=holding,
        _begin_operation=lambda action: calls.append(action))
    return node, calls


def test_goal_requires_exact_configuration_id_and_ready_lifecycle():
    node, calls = reservation("READY")
    assert RobotController._reserve_goal(node, "home", "stale") == GoalResponse.REJECT
    assert not calls
    assert RobotController._reserve_goal(node, "home", "active") == GoalResponse.ACCEPT
    assert calls == ["home"]
    node, _calls = reservation("INACTIVE", startup=False)
    assert RobotController._reserve_goal(node, "home", "active") == GoalResponse.REJECT


def test_home_allows_trusted_holding_but_pick_does_not():
    node, calls = reservation("HOLDING", holding=True)
    assert RobotController._reserve_goal(node, "home", "active") == GoalResponse.ACCEPT
    assert calls == ["home"]
    node, calls = reservation("HOLDING", holding=True)
    assert RobotController._reserve_goal(node, "pick", "active") == GoalResponse.REJECT
    assert not calls


def test_pick_requires_bin_station_binding():
    node, calls = reservation("READY", selection=False)
    assert RobotController._reserve_goal(node, "pick", "active") == GoalResponse.REJECT
    assert not calls


class Monitor:
    def __init__(self, di1):
        self.di1 = di1

    def snapshot(self, **_kwargs):
        return SimpleNamespace(feed={"digital_input_bits": int(self.di1)})


def finish_stop(previous, *, di1=False):
    machine = ControllerStateMachine(initial="STOPPING")
    node = SimpleNamespace(
        state_before_stop=previous, monitor=Monitor(di1), startup_complete=True,
        machine=machine,
        _transition=lambda target, message: machine.transition(target, message))
    RobotController._finish_stop_state(node)
    return machine.snapshot(), node.startup_complete


def test_confirmed_action_stop_always_requires_explicit_recovery():
    snapshot, startup = finish_stop("PICKING")
    assert snapshot.state == "RECOVERY_REQUIRED"
    assert "explicit recovery" in snapshot.message
    assert not startup


def test_stop_does_not_activate_an_unconfigured_or_inactive_controller():
    assert finish_stop("UNCONFIGURED")[0].state == "UNCONFIGURED"
    assert finish_stop("INACTIVE")[0].state == "INACTIVE"


def test_manual_cold_di1_resolution_is_observed_only_by_explicit_stop():
    assert finish_stop("HELD_UNKNOWN", di1=True)[0].state == "HELD_UNKNOWN"
    assert finish_stop("HELD_UNKNOWN", di1=False)[0].state == "RECOVERY_REQUIRED"


class EventLog:
    def record(self, *_args, **_kwargs):
        pass


class PauseHardware:
    def __init__(self, sample):
        self.sample = sample
        self.calls = []

    def pause_queue(self):
        self.calls.append("Pause")
        return self.sample

    def continue_queue(self):
        self.calls.append("Continue")
        return self.sample


def pause_node(state="PICKING"):
    machine = ControllerStateMachine(initial=state)
    sample = SimpleNamespace(robot_enabled=True, feed={
        "robot_mode": 10, "EnableStatus": 1, "ErrorStatus": 0,
        "CollisionStates": 0, "isPauseCmdFlag": 1,
        "digital_input_bits": 0, "digital_outputs": 0,
        "userCoordinate": 0, "toolCoordinate": 0,
    })
    node = SimpleNamespace(
        startup_complete=True, machine=machine, operation_lock=threading.Lock(),
        active_action="pick" if state == "PICKING" else "",
        phase="MOTION", waypoint="p1_pick", pause_guard=threading.Lock(),
        pause_event=threading.Event(), continue_event=threading.Event(),
        paused_context=None,
        hardware=PauseHardware(sample), monitor=SimpleNamespace(
            snapshot=lambda **_kwargs: sample), holding_item=False,
        expected_outputs={}, events=EventLog())
    node._transition = lambda target, message: machine.transition(target, message)
    node.raise_if_cancelled = lambda: None
    node._validate_pause_integrity = lambda value, **_kwargs: value
    node._contain_queue_control_failure = lambda *_args: None
    return node


def test_pause_preserves_action_context_and_continue_restores_it():
    node = pause_node()
    response = SimpleNamespace(success=False, message="", state="")
    RobotController._pause(node, None, response)
    assert response.success and response.state == "PAUSED"
    assert node.pause_event.is_set()
    assert node.paused_context["state"] == "PICKING"
    RobotController._continue(node, None, response)
    assert response.success and response.state == "PICKING"
    assert not node.pause_event.is_set()
    assert node.hardware.calls == ["Pause", "Continue"]


def test_stop_after_pause_is_direct_and_enters_recovery():
    snapshot, startup = finish_stop("PAUSED")
    assert snapshot.state == "RECOVERY_REQUIRED"
    assert not startup
    assert finish_stop("PAUSED", di1=True)[0].state == "HELD_UNKNOWN"


def test_idle_supervision_ignores_enable_pause_latch():
    sample = SimpleNamespace(robot_enabled=True, feed={
        "robot_mode": 5, "EnableStatus": 1, "isRunQueuedCmd": 0,
        "RunningStatus": 0, "ErrorStatus": 0, "CollisionStates": 0,
        "isPauseCmdFlag": 1, "userCoordinate": 0, "toolCoordinate": 0,
        "digital_input_bits": 0, "digital_outputs": 0,
    })
    stops = []
    node = SimpleNamespace(
        startup_complete=True,
        machine=SimpleNamespace(state="READY", message="Ready"),
        operation_lock=threading.Lock(),
        monitor=SimpleNamespace(
            snapshot=lambda **_kwargs: sample,
            consistent_flags=lambda _count: (1, 0, 0)),
        holding_item=False, expected_outputs={},
        _stop_unexpected_idle_motion=lambda reason="": stops.append(reason),
        _transition=lambda *_args: None, publish_status=lambda: None,
    )
    RobotController._supervise(node)
    assert stops == []


class Button:
    def __init__(self):
        self.text = ""

    def setText(self, value):
        self.text = value


class SpeedSlider:
    def __init__(self, position, *, down=False, focus=True):
        self.position = position
        self.down = down
        self.focus = focus

    def sliderPosition(self):
        return self.position

    def isSliderDown(self):
        return self.down

    def hasFocus(self):
        return self.focus


class Timer:
    def __init__(self):
        self.started = False

    def start(self):
        self.started = True

    def stop(self):
        self.started = False


def test_gui_speed_uses_live_slider_position_and_suppresses_noop():
    calls = []
    window = SimpleNamespace(
        pending={}, speed_slider=SpeedSlider(35), speed_debounce=Timer(),
        speed_pending_percent=None, speed_label=Button(),
        node=SimpleNamespace(status=SimpleNamespace(global_speed_percent=100)),
        _call=lambda name, request: calls.append((name, request.percent)) or True)
    ControllerWindow._speed(window)
    assert calls == [("speed", 35)]
    assert window.speed_pending_percent == 35
    assert "requesting 35%" in window.speed_label.text

    calls.clear()
    window.speed_slider.position = 100
    window.speed_pending_percent = None
    ControllerWindow._speed(window)
    assert calls == []


def test_gui_keyboard_speed_change_is_debounced():
    window = SimpleNamespace(
        speed_syncing=False, speed_slider=SpeedSlider(42),
        speed_debounce=Timer(), speed_label=Button())
    window._speed_preview = lambda value: ControllerWindow._speed_preview(window, value)
    ControllerWindow._speed_value_changed(window, 42)
    assert window.speed_debounce.started
    assert "42% selected" in window.speed_label.text


def test_gui_operator_log_is_copyable_and_bounded_for_display():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    view = QtWidgets.QPlainTextEdit()
    view.document().setMaximumBlockCount(1000)
    view.setReadOnly(True)
    view.setPlainText("first service\nsecond service")
    window = SimpleNamespace(log_view=view)
    ControllerWindow._copy_log(window)
    assert app.clipboard().text() == "first service\nsecond service"
    assert view.isReadOnly()
    assert view.document().maximumBlockCount() == 1000


def test_gui_drains_operator_log_queue_without_losing_order():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    assert app is not None
    values = ["one", "two"]
    view = QtWidgets.QPlainTextEdit()
    view.setReadOnly(True)
    window = SimpleNamespace(
        log_view=view,
        node=SimpleNamespace(take_operator_logs=lambda: tuple(values)))
    ControllerWindow._append_operator_logs(window)
    assert view.toPlainText() == "one\ntwo"


def test_ros_operator_log_queue_is_locked_bounded_and_ordered():
    from collections import deque

    node = SimpleNamespace(
        operator_logs=deque(maxlen=2), operator_log_lock=threading.Lock())
    for value in ("discarded", "one", "two"):
        GuiNode._operator_log(node, SimpleNamespace(data=value))
    assert GuiNode.take_operator_logs(node) == ("one", "two")
    assert GuiNode.take_operator_logs(node) == ()


def test_shared_initial_home_skips_all_motion_when_joint_gate_is_already_met():
    calls = []
    hardware = SimpleNamespace(
        home_already_reached=lambda joints: calls.append(("check", joints)) or True,
        move_batch=lambda *_args, **_kwargs: calls.append(("move",)))
    node = SimpleNamespace(
        hardware=hardware, holding_item=False,
        configuration=SimpleNamespace(home_joints=(0.1,) * 6),
        raise_if_cancelled=lambda: None, wait_for_resume=lambda: None,
        _preflight_item_state=lambda holding: calls.append(("preflight", holding)),
        _home_plan=lambda _preceding: calls.append(("plan",)) or (),
        operation_progress=lambda phase, message, **fields: calls.append(
            ("progress", phase, message, fields)))

    assert RobotController._execute_home(node) == ()
    assert calls[0] == ("preflight", False)
    assert calls[1] == ("check", (0.1,) * 6)
    assert not any(entry[0] in ("plan", "move") for entry in calls)
    assert "motion skipped" in calls[-1][2]


def test_queued_pick_return_never_uses_current_home_skip():
    calls = []
    hardware = SimpleNamespace(
        home_already_reached=lambda _joints: calls.append(("unexpected_check",)),
        move_batch=lambda targets, **kwargs: calls.append(("move", targets, kwargs)))
    node = SimpleNamespace(
        hardware=hardware, holding_item=False,
        configuration=SimpleNamespace(home_joints=(0.1,) * 6),
        raise_if_cancelled=lambda: None, wait_for_resume=lambda: None,
        _preflight_item_state=lambda holding: calls.append(("preflight", holding)),
        _home_plan=lambda preceding: calls.append(("plan", preceding)) or ("home",),
        operation_progress=lambda phase, message, **fields: calls.append(
            ("progress", phase, message, fields)))

    preceding = ("retract",)
    assert RobotController._execute_home(
        node, preceding=preceding, require_suction=False,
        forbid_suction=True) == ("home",)
    assert not any(entry[0] == "unexpected_check" for entry in calls)
    assert ("plan", preceding) in calls
    assert any(entry[0] == "move" for entry in calls)


def test_gui_second_pause_click_queues_stop_without_overlapping_pause():
    commands = []
    window = SimpleNamespace(
        node=SimpleNamespace(status=SimpleNamespace(state="PICKING")), pending={},
        pause_requested_locally=False, stop_after_pause=False, stop=Button(),
        _command=lambda name: commands.append(name) or True,
        _immediate_stop=lambda: commands.append("stop"))
    ControllerWindow._pause_or_stop(window)
    ControllerWindow._pause_or_stop(window)
    assert commands == ["pause"]
    assert window.stop_after_pause
    assert window.stop.text == "STOP QUEUED"


def test_gui_nonpausable_state_calls_stop_directly_and_paused_start_continues():
    commands = []
    window = SimpleNamespace(
        node=SimpleNamespace(status=SimpleNamespace(state="FAULT")), pending={},
        pause_requested_locally=False, stop_after_pause=False, stop=Button(),
        _command=lambda name: commands.append(name) or True,
        _immediate_stop=lambda: commands.append("stop"))
    ControllerWindow._pause_or_stop(window)
    window.node.status.state = "PAUSED"
    ControllerWindow._start_or_continue(window)
    assert commands == ["stop", "continue"]


def test_gui_queued_stop_dispatches_only_after_pause_response():
    class Future:
        @staticmethod
        def done():
            return True

        @staticmethod
        def result():
            return SimpleNamespace(success=True, message="paused")

    calls = []
    window = SimpleNamespace(
        pending={"pause": Future()}, pending_goal=None, result_future=None,
        pause_requested_locally=True, stop_after_pause=True,
        _immediate_stop=lambda: calls.append("stop"))
    ControllerWindow._collect(window)
    assert calls == ["stop"]
    assert not window.pause_requested_locally
    assert not window.stop_after_pause
