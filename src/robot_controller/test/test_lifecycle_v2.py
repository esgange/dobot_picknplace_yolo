from types import SimpleNamespace
import threading

import numpy as np
import pytest
from PyQt5 import QtWidgets
from rclpy.action import GoalResponse
from robot_controller_interfaces.action import GoHome

import robot_controller.controller as controller_module
import robot_controller.gui as gui_module
from robot_controller.controller import RobotController
from robot_controller.errors import FeedbackFailure, HeldSuctionLost
from robot_controller.gui import ControllerWindow, GuiNode
from robot_controller.state_machine import ControllerStateMachine


class Config:
    configuration_id = "active"
    selection = object()


@pytest.mark.parametrize("provider", [
    ("item_detect", "/"),
    ("item_teach", "/"),
])
def test_pick_accepts_one_canonical_headless_or_armed_teach_pose_provider(provider):
    node = SimpleNamespace(_service_providers=lambda _service: [provider])
    RobotController.check_detector_owner(node)


@pytest.mark.parametrize("providers", [
    [],
    [("unexpected_detector", "/")],
    [("item_teach", "/station")],
    [("item_detect", "/"), ("item_teach", "/")],
])
def test_pick_rejects_missing_unknown_namespaced_or_duplicate_pose_providers(providers):
    node = SimpleNamespace(_service_providers=lambda _service: providers)
    with pytest.raises(FeedbackFailure, match="exactly one canonical root provider"):
        RobotController.check_detector_owner(node)


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
    RobotController._publish_stopped_state(node, previous)
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


def configuration_node(state="READY", *, headless=False):
    machine = ControllerStateMachine(initial=state)
    old = SimpleNamespace(configuration_id="old")
    calls = []
    node = SimpleNamespace(
        headless=headless, machine=machine, configuration=old,
        startup_complete=True, global_speed_percent=35, holding_item=False,
        expected_outputs={13: 1}, root=object(), kinematics=object(),
        events=EventLog())
    node._begin_operation = lambda name: calls.append(("begin", name))
    node._end_operation = lambda: calls.append(("end",))
    node._transition = lambda target, message: machine.transition(target, message)
    node._log_configuration = lambda event: calls.append(("log", event))
    return node, old, calls


def test_ready_configuration_reload_replaces_snapshot_and_requires_startup(monkeypatch):
    node, old, calls = configuration_node()
    new = SimpleNamespace(configuration_id="new")
    monkeypatch.setattr(controller_module, "load_configuration", lambda *_args, **_kwargs: new)
    request = SimpleNamespace(item_teach_file="item.yaml", bin_teach_file="bin.yaml")
    response = SimpleNamespace(success=False, message="", configuration_id="")

    RobotController._configure(node, request, response)

    assert node.configuration is new and node.configuration is not old
    assert node.machine.state == "INACTIVE"
    assert not node.startup_complete
    assert node.global_speed_percent is None
    assert node.expected_outputs == {}
    assert response.success and response.configuration_id == "new"
    assert calls == [
        ("begin", "configure"), ("log", "configuration_loaded"), ("end",)]


def test_failed_ready_reload_preserves_active_configuration(monkeypatch):
    node, old, calls = configuration_node()

    def reject(*_args, **_kwargs):
        raise ValueError("invalid replacement")

    monkeypatch.setattr(controller_module, "load_configuration", reject)
    request = SimpleNamespace(item_teach_file="bad.yaml", bin_teach_file="bin.yaml")
    response = SimpleNamespace(success=True, message="", configuration_id="stale")

    RobotController._configure(node, request, response)

    assert node.configuration is old
    assert node.machine.state == "READY"
    assert node.startup_complete
    assert node.global_speed_percent == 35
    assert node.expected_outputs == {13: 1}
    assert not response.success and response.configuration_id == ""
    assert "invalid replacement" in response.message
    assert calls == [("begin", "configure"), ("end",)]


@pytest.mark.parametrize("state", [
    "STARTING", "HOMING", "PICKING", "HOLDING", "PAUSED", "STOPPING",
    "RECOVERY_REQUIRED", "RECOVERING", "HELD_UNKNOWN", "FAULT",
])
def test_configuration_reload_rejects_nonidle_or_held_states(state):
    node, old, calls = configuration_node(state)
    request = SimpleNamespace(item_teach_file="item.yaml", bin_teach_file="bin.yaml")
    response = SimpleNamespace(success=True, message="", configuration_id="stale")

    RobotController._configure(node, request, response)

    assert node.configuration is old
    assert node.machine.state == state
    assert not response.success
    assert "UNCONFIGURED, INACTIVE, or READY" in response.message
    assert calls == []


def test_headless_configuration_remains_immutable():
    node, old, calls = configuration_node("INACTIVE", headless=True)
    request = SimpleNamespace(item_teach_file="item.yaml", bin_teach_file="bin.yaml")
    response = SimpleNamespace(success=True, message="", configuration_id="stale")

    RobotController._configure(node, request, response)

    assert node.configuration is old
    assert node.machine.state == "INACTIVE"
    assert not response.success
    assert "immutable" in response.message
    assert calls == []


def test_ready_configuration_reload_rejects_inconsistent_holding_context():
    node, old, calls = configuration_node("READY")
    node.holding_item = True
    request = SimpleNamespace(item_teach_file="item.yaml", bin_teach_file="bin.yaml")
    response = SimpleNamespace(success=True, message="", configuration_id="stale")

    RobotController._configure(node, request, response)

    assert node.configuration is old
    assert node.machine.state == "READY"
    assert not response.success
    assert "holding an item" in response.message
    assert calls == []


class EventLog:
    def record(self, *_args, **_kwargs):
        pass


def test_pause_and_continue_delegate_to_managed_operation_without_vendor_queue_resume():
    calls = []
    node = SimpleNamespace(
        machine=SimpleNamespace(state="PAUSED"), events=EventLog(),
        managed=SimpleNamespace(continue_operation=lambda: calls.append("continue")),
        _managed_request=lambda kind, response: calls.append(kind) or response)
    response = SimpleNamespace(success=False, message="", state="")
    RobotController._pause(node, None, response)
    RobotController._continue(node, None, response)
    assert response.success
    assert calls == ["pause", "continue"]


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


def test_home_preflight_and_idle_holding_share_the_fifty_ms_suction_loss_gate():
    from test_feedback_v2 import timed_monitor

    monitor, _clock, emit = timed_monitor()
    machine = ControllerStateMachine(initial="HOLDING")
    node = SimpleNamespace(
        monitor=monitor, holding_item=True, startup_complete=True,
        operation_lock=threading.Lock(), machine=machine, expected_outputs={13: True},
        managed=SimpleNamespace(note_suction_loss=lambda _sample: None),
        _transition=lambda target, message: machine.transition(target, message))
    emit(0.0, True)
    emit(0.010, False)
    emit(0.010 + 0.049999, False)
    RobotController._preflight_item_state(node)
    RobotController._supervise(node)
    assert machine.state == "HOLDING"
    emit(0.010 + 0.050, False)
    with pytest.raises(HeldSuctionLost, match="lost DI1"):
        RobotController._preflight_item_state(node)
    RobotController._supervise(node)
    assert machine.state == "FAULT"
    assert "DI1 lost" in machine.message


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


@pytest.mark.parametrize("holding", [False, True])
def test_hardware_home_queues_two_cartesian_waypoints_in_one_group(holding):
    calls = []
    current = np.eye(4)
    current[:3, 3] = [0.1, 0.2, 0.1]
    home = np.eye(4)
    home[:3, 3] = [0.3, -0.4, 0.5]
    config = SimpleNamespace(
        home_matrix=home,
        profile={"speed": {"travel_percent": 75},
                 "acceleration": {"travel_percent": 60}},
        validate_sources=lambda _root: calls.append(("validate",)))
    hardware = SimpleNamespace(
        current_pose=lambda: calls.append(("current_pose",)) or current,
        move_batch=lambda targets, **kwargs: calls.append(("move", targets, kwargs)))
    node = SimpleNamespace(
        root="/unused", hardware=hardware, configuration=config,
        holding_item=holding, raise_if_cancelled=lambda: None,
        wait_for_resume=lambda: None,
        _preflight_item_state=lambda expected: calls.append(("preflight", expected)),
        operation_progress=lambda phase, message, **fields: calls.append(
            ("progress", phase, message, fields)))

    targets = RobotController._execute_cartesian_home(node)
    moves = [call for call in calls if call[0] == "move"]
    assert len(moves) == 1
    assert [target.name for target in moves[0][1]] == ["home_align", "home"]
    assert all(target.joints_rad is None and not target.relative_z
               for target in moves[0][1])
    assert moves[0][2] == {"batch_name": "home", "require_suction": holding,
                           "forbid_suction": not holding,
                           "confirmed_start_pose": current}
    assert targets == moves[0][1]
    assert calls.index(("preflight", holding)) < calls.index(moves[0])
    assert calls.count(("preflight", holding)) == 1
    assert calls.count(("current_pose",)) == 1


def test_hardware_home_skips_when_cartesian_feedback_is_already_within_tolerance():
    home = np.eye(4)
    current = home.copy()
    current[0, 3] = 0.004
    moves = []
    node = SimpleNamespace(
        root="/unused", holding_item=False,
        configuration=SimpleNamespace(
            home_matrix=home, validate_sources=lambda _root: None),
        hardware=SimpleNamespace(
            current_pose=lambda: current,
            move_batch=lambda *_args, **_kwargs: moves.append("move")),
        raise_if_cancelled=lambda: None, wait_for_resume=lambda: None,
        _preflight_item_state=lambda _holding: None,
        operation_progress=lambda *_args, **_kwargs: None)

    assert RobotController._execute_cartesian_home(node) == ()
    assert moves == []


def test_hardware_home_group_failure_is_propagated():
    home = np.eye(4)
    home[:3, 3] = [0.3, -0.4, 0.5]
    calls = []

    def move(targets, **_kwargs):
        calls.append(tuple(target.name for target in targets))
        raise FeedbackFailure("Home group failed")

    node = SimpleNamespace(
        root="/unused", holding_item=False,
        configuration=SimpleNamespace(
            home_matrix=home,
            profile={"speed": {"travel_percent": 100},
                     "acceleration": {"travel_percent": 100}},
            validate_sources=lambda _root: None),
        hardware=SimpleNamespace(current_pose=lambda: np.eye(4), move_batch=move),
        raise_if_cancelled=lambda: None, wait_for_resume=lambda: None,
        _preflight_item_state=lambda _holding: None,
        operation_progress=lambda *_args, **_kwargs: None)

    with pytest.raises(FeedbackFailure, match="Home group failed"):
        RobotController._execute_cartesian_home(node)
    assert calls == [("home_align", "home")]


def test_home_action_uses_cartesian_route_without_changing_pick_home():
    calls = []
    machine = SimpleNamespace(message="")

    def transition(state, message):
        machine.message = message
        calls.append(("state", state))

    node = SimpleNamespace(
        root="/unused", active_goal=None, holding_item=False, machine=machine,
        configuration=SimpleNamespace(validate_sources=lambda _root: None),
        _transition=transition,
        _execute_cartesian_home=lambda: calls.append(("cartesian_home",)),
        _execute_home=lambda: pytest.fail("Pick's shared Home was called"),
        managed=SimpleNamespace(lock=threading.RLock()),
        wait_for_resume=lambda: None,
        events=SimpleNamespace(record=lambda *_args, **_kwargs: None),
        _end_operation=lambda: calls.append(("end",)))
    goal = SimpleNamespace(succeed=lambda: calls.append(("succeed",)))

    result = RobotController._execute_home_action(node, goal)
    assert result.outcome == GoHome.Result.SUCCESS
    assert result.final_state == "READY"
    assert calls == [("state", "HOMING"), ("cartesian_home",),
                     ("state", "READY"), ("succeed",), ("end",)]


@pytest.mark.parametrize("holding", [False, True])
def test_separate_home_mode_confirms_preceding_targets_and_height(holding):
    calls = []
    clearance_pose = np.eye(4)
    clearance = SimpleNamespace(name="p1_final")
    height = SimpleNamespace(name="home_height")
    home = SimpleNamespace(name="home")
    hardware = SimpleNamespace(
        home_already_reached=lambda _joints: calls.append(("check",)) or False,
        current_pose=lambda: calls.append(("pose",)) or clearance_pose,
        move_batch=lambda targets, **kwargs: calls.append(("move", targets, kwargs)))
    node = SimpleNamespace(
        hardware=hardware, holding_item=holding, root=None,
        configuration=SimpleNamespace(
            home_joints=(0.1,) * 6, validate_sources=lambda _root: None),
        raise_if_cancelled=lambda: None, wait_for_resume=lambda: None,
        _preflight_item_state=lambda holding: calls.append(("preflight", holding)),
        _home_plan=lambda current: calls.append(("plan", current)) or (height, home),
        operation_progress=lambda phase, message, **fields: calls.append(
            ("progress", phase, message, fields)))

    preceding = (clearance,)
    confirmed_start = np.eye(4)
    assert RobotController._execute_home(
        node, preceding=preceding, require_suction=holding,
        forbid_suction=not holding,
        confirmed_start_pose=confirmed_start,
        batch_name="candidate_1_pick_to_home") == (height, home)
    moves = [entry for entry in calls if entry[0] == "move"]
    assert [entry[1] for entry in moves] == [preceding, (height,), (home,)]
    assert [entry[2]["batch_name"] for entry in moves] == [
        "candidate_1_pick_to_home_clearance",
        "candidate_1_pick_to_home_height",
        "candidate_1_pick_to_home"]
    assert all(entry[2]["require_suction"] is holding
               and entry[2]["forbid_suction"] is not holding for entry in moves)
    assert moves[0][2]["confirmed_start_pose"] is confirmed_start
    assert moves[1][2]["confirmed_start_pose"] is clearance_pose
    assert moves[2][2]["confirmed_start_pose"] is None
    assert next(entry for entry in calls if entry[0] == "plan")[1] is clearance_pose
    assert calls.count(("pose",)) == 1
    assert calls.index(("check",)) > calls.index(moves[0])


@pytest.mark.parametrize("holding", [False, True])
def test_pick_return_queues_retract_clearance_and_home_as_one_group(holding):
    calls = []
    start = np.eye(4)
    retract = SimpleNamespace(name="p2_retract", matrix=np.eye(4))
    clearance = SimpleNamespace(name="p2_final", matrix=np.eye(4))
    height = SimpleNamespace(name="home_height")
    home = SimpleNamespace(name="home")
    hardware = SimpleNamespace(
        home_already_reached=lambda _joints: pytest.fail(
            "Queued return must not wait for a Home-skip sample"),
        move_batch=lambda targets, **kwargs: calls.append((targets, kwargs)))
    node = SimpleNamespace(
        hardware=hardware, holding_item=holding, root=None,
        configuration=SimpleNamespace(
            home_joints=(0.1,) * 6, validate_sources=lambda _root: None),
        raise_if_cancelled=lambda: None, wait_for_resume=lambda: None,
        _preflight_item_state=lambda required: calls.append(("preflight", required)),
        _home_plan=lambda origin: (
            calls.append(("plan_origin", origin)) or (height, home)),
        operation_progress=lambda *_args, **_kwargs: None)

    assert RobotController._execute_home(
        node, preceding=(retract, clearance), require_suction=holding,
        forbid_suction=False, ignore_suction=not holding,
        confirmed_start_pose=start, queue_through_home=True,
        batch_name="pick_to_home") == (height, home)
    assert calls[:-2] == ([("preflight", True)] if holding else [])
    assert calls[-2] == ("plan_origin", clearance.matrix)
    assert calls[-1] == ((retract, clearance, height, home), {
        "batch_name": "pick_to_home", "require_suction": holding,
        "forbid_suction": False, "confirmed_start_pose": start})


def test_home_height_failure_prevents_joint_home_dispatch():
    calls = []
    height = SimpleNamespace(name="home_height")
    home = SimpleNamespace(name="home")

    def move(targets, **_kwargs):
        calls.append(tuple(target.name for target in targets))
        if targets == (height,):
            raise FeedbackFailure("Home Z not reached")

    node = SimpleNamespace(
        hardware=SimpleNamespace(
            home_already_reached=lambda _joints: False, move_batch=move,
            current_pose=lambda: np.eye(4)),
        holding_item=False, root=None,
        configuration=SimpleNamespace(
            home_joints=(0.1,) * 6, validate_sources=lambda _root: None),
        raise_if_cancelled=lambda: None, wait_for_resume=lambda: None,
        _preflight_item_state=lambda _holding: None,
        _home_plan=lambda _current: (height, home),
        operation_progress=lambda *_args, **_kwargs: None)

    with pytest.raises(FeedbackFailure, match="Home Z not reached"):
        RobotController._execute_home(node)
    assert calls == [("home_height",)]


@pytest.mark.parametrize("below_home_m", [-0.1, 0.0, 0.000013, 0.005])
@pytest.mark.parametrize("holding", [False, True])
def test_home_near_safety_z_uses_one_pose_and_direct_joint_home(below_home_m, holding):
    source_checks = []
    poses = []
    moves = []
    preflights = []
    home = np.eye(4)
    home[2, 3] = 0.35
    current = home.copy()
    current[:3, 3] = [0.6, -0.1, home[2, 3] - below_home_m]

    def current_pose():
        assert source_checks, "Validate source files before acquiring the fresh origin"
        assert not poses, "Planning and dispatch must share one confirmed origin"
        poses.append(current)
        return current

    node = SimpleNamespace(
        root=None,
        hardware=SimpleNamespace(
            home_already_reached=lambda _joints: False,
            current_pose=current_pose,
            move_batch=lambda targets, **kwargs: moves.append((targets, kwargs))),
        holding_item=holding,
        configuration=SimpleNamespace(
            home_joints=(0.1,) * 6, home_matrix=home,
            validate_sources=lambda _root: source_checks.append(True),
            profile={"speed": {"travel_percent": 75},
                     "acceleration": {"travel_percent": 60}}),
        raise_if_cancelled=lambda: None, wait_for_resume=lambda: None,
        _preflight_item_state=preflights.append,
        operation_progress=lambda *_args, **_kwargs: None)
    node._home_plan = lambda origin: RobotController._home_plan(node, origin)

    targets = RobotController._execute_home(node)
    assert [target.name for target in targets] == ["home"]
    assert targets[0].joints_rad == node.configuration.home_joints
    assert np.array_equal(targets[0].matrix, home)
    assert len(poses) == len(moves) == 1
    assert len(moves[0][0]) == 1 and moves[0][0][0] is targets[0]
    assert moves[0][1]["confirmed_start_pose"] is current
    assert moves[0][1]["require_suction"] is holding
    assert moves[0][1]["forbid_suction"] is not holding
    assert preflights == [holding]


def test_gui_second_pause_click_stops_immediately_during_parking():
    commands = []
    window = SimpleNamespace(
        node=SimpleNamespace(status=SimpleNamespace(state="PICKING")), pending={},
        pause_requested_locally=False, return_requested_locally=False, stop=Button(),
        _command=lambda name: commands.append(name) or True,
        _immediate_stop=lambda: commands.append("stop"))
    ControllerWindow._pause_or_stop(window)
    ControllerWindow._pause_or_stop(window)
    assert commands == ["pause", "stop"]


def test_gui_nonpausable_state_calls_stop_directly_and_paused_start_continues():
    commands = []
    window = SimpleNamespace(
        node=SimpleNamespace(status=SimpleNamespace(state="FAULT")), pending={},
        pause_requested_locally=False, return_requested_locally=False, stop=Button(),
        _command=lambda name: commands.append(name) or True,
        _immediate_stop=lambda: commands.append("stop"))
    ControllerWindow._pause_or_stop(window)
    window.node.status.state = "PAUSED"
    ControllerWindow._start_or_continue(window)
    assert commands == ["stop", "continue"]


def test_gui_pause_acceptance_does_not_dispatch_another_stop():
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
        pause_requested_locally=True, return_requested_locally=False,
        _immediate_stop=lambda: calls.append("stop"))
    ControllerWindow._collect(window)
    assert calls == []
    assert window.pause_requested_locally


def test_successful_gui_reload_saves_selection_and_clears_preview(monkeypatch, tmp_path):
    class Future:
        @staticmethod
        def done():
            return True

        @staticmethod
        def result():
            return SimpleNamespace(success=True, message="loaded")

    saved = []
    cleared = []
    monkeypatch.setattr(
        gui_module, "save_state",
        lambda path, item, bin_path: saved.append((path, item, bin_path)))
    window = SimpleNamespace(
        pending={"configure": Future()}, pending_goal=None, result_future=None,
        saved_selection=("item.yaml", "bin.yaml"),
        node=SimpleNamespace(root=tmp_path),
        _clear_preview=lambda: cleared.append(True))

    ControllerWindow._collect(window)

    assert saved == [(
        tmp_path / "logs/robot_controller/last_session.json",
        "item.yaml", "bin.yaml")]
    assert cleared == [True]


def test_gui_held_pause_stop_requests_return_without_canceling_active_pick():
    calls = []
    window = SimpleNamespace(
        node=SimpleNamespace(status=SimpleNamespace(state="PAUSED", can_return_item=True)),
        pending={}, pause_requested_locally=False, return_requested_locally=False, stop=Button(),
        _command=lambda name: calls.append(name) or True,
        _immediate_stop=lambda: calls.append("stop"))
    ControllerWindow._pause_or_stop(window)
    assert calls == ["return_item"]
    ControllerWindow._pause_or_stop(window)
    assert calls == ["return_item", "stop"]


@pytest.mark.parametrize("state", ["PAUSING", "RETURNING_ITEM"])
def test_gui_stop_preempts_managed_motion_immediately(state):
    calls = []
    window = SimpleNamespace(
        node=SimpleNamespace(status=SimpleNamespace(state=state)),
        pending={}, pause_requested_locally=False, return_requested_locally=False,
        _command=lambda _name: pytest.fail("Cannot Pause a managed movement"),
        _immediate_stop=lambda: calls.append("stop"))
    ControllerWindow._pause_or_stop(window)
    assert calls == ["stop"]
