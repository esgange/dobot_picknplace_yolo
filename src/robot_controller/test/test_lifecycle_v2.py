from types import SimpleNamespace

from rclpy.action import GoalResponse

from robot_controller.controller import RobotController
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
