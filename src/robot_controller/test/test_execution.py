"""Synthetic transport/streams only: never instantiate a real Dobot transport."""

import json
from dataclasses import replace
from datetime import datetime
import math
from pathlib import Path
import shutil
import threading
from types import MethodType, SimpleNamespace as NS
from unittest.mock import MagicMock, call

import numpy as np
import pytest
from rclpy.task import Future

from test_controller import pair as _pair_fixture
from robot_controller import controller, hardware, profiles, ui_state
from robot_controller.kinematics import Cr10Kinematics, pose_matrix, pose_values
from robot_controller.motion import PickExecutor, Target, home_targets, pick_targets


pair = _pair_fixture  # Reuse the strict synthetic item writer.


def model():
    path = Path(__file__).resolve().parents[2] / "DOBOT_6Axis_ROS2_V4/cra_description"
    return Cr10Kinematics(path / "urdf/cr10_robot.xacro")


def settings():
    return {"motion": {"standoff_height": 90.,
                       "prepick_height": 50., "retract_height": 100.},
            "speed": {"travel_percent": 100, "approach_percent": 6, "retract_percent": 6},
            "acceleration": {"travel_percent": 100, "approach_percent": 100,
                             "retract_percent": 100},
            "gripper": {"use_grip": True, "grip_onpick": True},
            "timing": {"pick_settling": .2}}


def plan(index=1, cfg=None):
    return pick_targets(pose_matrix([100, 200, 800, 180, 0, 0]), [.3, .4, .1],
                        cfg or settings(), index)


def home_plan(current, home, joints):
    return home_targets(current, home, joints, speed_percent=100, acceleration_percent=100)


class FakeHardware:
    def __init__(self, acquisitions=(True,), *, failed=None, stopped_z=None):
        self.acquisitions = iter(acquisitions)
        self.failed = failed
        self.trace = []
        self.pose = np.eye(4)
        self.stopped_z = stopped_z

    def output(self, channel, active, **_):
        self.trace.append(("DO", channel, active))

    def sensor(self, active, timeout, *, settling_sec):
        self.trace.append(("DI", active, timeout))
        return True if not active else next(self.acquisitions)

    def move(self, target, **kw):
        self.trace.append(("move", target.name, kw, float(target.matrix[2, 3]),
                           target.speed_percent, target.acceleration_percent))
        if self.failed == target.name:
            raise ValueError("Injected hardware failure")
        self.pose = target.matrix.copy()
        if kw.get("stop_on_suction"):
            acquired = next(self.acquisitions)
            if acquired and self.stopped_z is not None:
                self.pose[2, 3] = self.stopped_z
            return acquired
        return False

    def current_pose(self):
        return self.pose.copy()

    def move_batch(self, targets, **kw):
        acquired = False
        for target in targets:
            if target.name.endswith("_pick") and kw.get("before_suction"):
                kw["before_suction"]()
            for event in target.motion_io:
                if event.percent < 100:
                    self.output(event.channel, event.active)
            acquired = self.move(target, require_suction=kw.get("require_suction", False),
                                 stop_on_suction=bool(kw.get("stop_on_suction") and
                                                      target.name.endswith("_pick")))
            for event in target.motion_io:
                if event.percent == 100:
                    self.output(event.channel, event.active)
        return acquired


def execute(fake, cfg=None, plans=None, check=None, *, finish_home=False,
            remember_prepick=None):
    def return_home(preceding=(), require_suction=False, forbid_suction=False):
        fake.move_batch(preceding, require_suction=require_suction, forbid_suction=forbid_suction)
        fake.trace.append(("home", {"require_suction": require_suction}))

    return PickExecutor(fake, finish_home=finish_home).run(
        plans or [plan(cfg=cfg)], cfg or settings(), check=check or (lambda _: None),
        return_home=return_home,
        remember_prepick=remember_prepick)


def test_pose_units_and_canonical_fk_chain():
    robot = model()
    joints = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    expected = np.eye(4)
    for joint, angle in zip(robot.joints, joints):
        rotation = pose_matrix([0, 0, 0, 0, 0, math.degrees(angle)])
        expected = expected @ joint.origin @ rotation
    np.testing.assert_allclose(robot.forward(joints), expected, atol=1e-12)
    original = pose_matrix([100, 200, 300, 20, -30, 40])
    np.testing.assert_allclose(pose_matrix(pose_values(original)), original, atol=1e-12)
    assert len(robot.sha256) == 64
    for bad in ([0.] * 5, [math.nan] * 6, [100.] * 6):
        with pytest.raises(ValueError):
            robot.forward(bad)


def test_home_height_keeps_current_xy_and_orientation_then_exact_joints():
    current, home = pose_matrix([10, 20, 30, 5, 6, 7]), model().forward([.1] * 6)
    height, final = home_plan(current, home, [.1] * 6)
    assert height.relative_z and final.joints_rad == (.1,) * 6
    assert height.matrix[2, 3] == home[2, 3]
    np.testing.assert_allclose(height.matrix[:2, 3], current[:2, 3])
    np.testing.assert_allclose(height.matrix[:3, :3], current[:3, :3])
    np.testing.assert_allclose(final.matrix, home)


@pytest.mark.parametrize("offset", [0., .001, .5])
def test_at_or_above_home_height_goes_directly_to_home(offset):
    home = model().forward([.1] * 6)
    current = home.copy()
    current[2, 3] += offset
    targets = home_plan(current, home, [.1] * 6)
    assert len(targets) == 1
    assert targets[0].name == "home" and not targets[0].relative_z
    assert targets[0].joints_rad == (.1,) * 6


def test_pick_height_equations_and_home_attitude():
    targets = plan()
    np.testing.assert_allclose([t.matrix[2, 3] for t in targets], [.8, .34, .24, .19, .24, .34])
    for target in targets:
        np.testing.assert_allclose(target.matrix[:3, :3], targets[0].matrix[:3, :3])
        np.testing.assert_allclose(target.matrix[:2, 3], [.3, .4])
    with pytest.raises(ValueError, match="Home Z"):
        pick_targets(np.eye(4), [.3, .4, .1], settings(), 1)


def test_motion_rates_are_assigned_to_each_pick_stage_and_survive_early_retract():
    cfg = settings()
    cfg["speed"] = {"travel_percent": 90, "approach_percent": 7, "retract_percent": 8}
    cfg["acceleration"] = {"travel_percent": 80, "approach_percent": 40, "retract_percent": 30}
    targets = pick_targets(pose_matrix([100, 200, 800, 180, 0, 0]), [.3, .4, .1], cfg, 1)
    assert [t.speed_percent for t in targets] == [90, 90, 90, 7, 8, 90]
    assert [t.acceleration_percent for t in targets] == [80, 80, 80, 40, 30, 80]
    fake = FakeHardware(stopped_z=.35)
    execute(fake, cfg=cfg, plans=[targets])
    moves = [v for v in fake.trace if v[0] == "move"]
    assert [v[4:] for v in moves] == [(90, 80)] * 3 + [(7, 40), (8, 30), (90, 80)]
    assert all(v[3] == .35 for v in moves[-2:])


@pytest.mark.parametrize("field,bad", [
    ("speed_percent", 0), ("speed_percent", 101), ("speed_percent", 6.), ("speed_percent", True),
    ("acceleration_percent", 0), ("acceleration_percent", 101),
    ("acceleration_percent", 100.), ("acceleration_percent", False),
])
def test_target_rates_reject_invalid_values(field, bad):
    values = {"speed_percent": 6, "acceleration_percent": 100, field: bad}
    with pytest.raises(ValueError, match="explicit integer"):
        Target("test", np.eye(4), **values)


@pytest.mark.parametrize("use_grip,close", [(False, False), (False, True),
                                            (True, False), (True, True)])
def test_finger_rules_and_success_holds_final_retract(use_grip, close):
    cfg, fake = settings(), FakeHardware()
    cfg["gripper"] = {"use_grip": use_grip, "grip_onpick": close}
    outcome = execute(fake, cfg)
    assert outcome["picked"] and outcome["holding_item"]
    finger = [v for v in fake.trace if v[0] == "DO" and v[1] in (2, 14)]
    expected = [("DO", 2, False), ("DO", 14, True)] if use_grip else []
    if use_grip:
        expected += [("DO", 14, False), ("DO", 2, True)]
    assert finger == expected
    assert fake.trace[-1][1] == "p1_final" and fake.trace[-1][2]["require_suction"]
    assert not any(v[0] == "home" for v in fake.trace)
    vacuum = [v for v in fake.trace if v[0] == "DO" and v[1] == 13]
    assert vacuum == [("DO", 13, False), ("DO", 13, True)]


def test_early_suction_never_descends_to_a_nominal_retract():
    fake = FakeHardware(stopped_z=.35)
    execute(fake)
    assert [v[3] for v in fake.trace if v[0] == "move"][-2:] == [.35, .35]


def test_missed_suction_settles_and_final_completion_precedes_next_candidate():
    fake = FakeHardware((False, False, True))
    checked = []
    outcome = execute(fake, plans=[plan(), plan(2)], check=checked.append)
    assert checked == [1, 1, 2, 2] and outcome["candidate"] == 2
    assert ("DI", True, .2) in fake.trace
    final = next(i for i, v in enumerate(fake.trace) if v[0:2] == ("move", "p1_final"))
    home = next(i for i, v in enumerate(fake.trace) if v[0] == "home")
    second = next(i for i, v in enumerate(fake.trace) if v[0:2] == ("move", "p2_transit"))
    assert final < home < second


def test_live_pick_returns_home_on_success_and_after_exhausting_all_candidates():
    success = FakeHardware()
    outcome = execute(success, finish_home=True)
    assert outcome["picked"] and success.trace[-1] == ("home", {"require_suction": True})
    missed = FakeHardware((False, False, False, False))
    outcome = execute(missed, plans=[plan(), plan(2)], finish_home=True)
    homes = [entry for entry in missed.trace if entry[0] == "home"]
    assert not outcome["picked"] and homes == [
        ("home", {"require_suction": False}), ("home", {"require_suction": False})]


def test_pick_records_each_candidate_last_prepick_for_stop_recovery():
    remembered = []
    execute(FakeHardware((False, False, True)), plans=[plan(), plan(2)],
            remember_prepick=lambda target, grip: remembered.append((target, dict(grip))))
    assert [entry[0].name for entry in remembered] == ["p1_prepick", "p2_prepick"]
    assert remembered[-1][1] == settings()["gripper"]


@pytest.mark.parametrize("failed", ["p1_transit", "p1_pick", "p1_retract", "p1_final"])
def test_hardware_fault_is_not_retryable(failed):
    fake = FakeHardware(failed=failed)
    with pytest.raises(ValueError, match="Injected hardware"):
        execute(fake, plans=[plan(), plan(2)])
    assert not any(v[0:2] == ("move", "p2_transit") for v in fake.trace)


def test_exhausted_candidates_and_expiry_do_not_reacquire():
    fake = FakeHardware((False, False))
    outcome = execute(fake)
    assert not outcome["picked"] and not outcome["holding_item"]
    fake = FakeHardware((False, False))

    def check(index):
        if index == 2:
            raise ValueError("expired")
    with pytest.raises(ValueError, match="expired"):
        execute(fake, plans=[plan(), plan(2)], check=check)
    assert any(v[0:2] == ("move", "p1_final") for v in fake.trace)
    assert not any(v[0:2] == ("move", "p2_transit") for v in fake.trace)


def synthetic_transport(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(hardware.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(hardware.time, "sleep", lambda dt: clock.__setitem__(0, clock[0]+dt))
    robot = model()
    pose = robot.forward([.1] * 6)
    feed = {"digital_input_bits": 0, "digital_outputs": 0, "EnableStatus": 1, "isRunQueuedCmd": 0,
            "RunningStatus": 0, "robot_mode": 5, "tool_vector_actual": pose_values(pose)}
    sequence = [0]

    def snapshot(**_):
        sequence[0] += 1
        return {"feed": feed, "enabled": feed["robot_mode"] == 5, "sequence": sequence[0]}
    node = NS(check_cancelled=MagicMock(), check_command_owner=MagicMock(),
              cancel=threading.Event(),
              feedback_snapshot=snapshot, current_joints=lambda: [.1] * 6, kinematics=robot,
              events=MagicMock(), set_execution_state=MagicMock())
    transport = object.__new__(hardware.DobotHardware)
    transport.node, transport.moving = node, False
    transport.response_lock = threading.Lock()
    transport.pending_response = None
    transport.suction_stop, transport.suction_interrupted = None, False
    names = ("Stop", "StopMoveJog", "DisableRobot", "EnableRobot", "SpeedFactor", "Tool",
             "SetTool", "CP", "DO", "GetPose", "InverseKin", "MovLIO", "RelMovLUser")
    transport.types = {name: NS(Request=lambda **fields: NS(**fields)) for name in names}
    transport.clients = {name: MagicMock() for name in names}
    for name, client in transport.clients.items():
        future = Future()
        raw = "0,{" + ",".join(map(str, pose_values(pose))) + "},GetPose(user=0,tool=0);"
        if name == "InverseKin":
            raw = "0,{" + ",".join(map(str, np.rad2deg([.1]*6))) + "},InverseKin();"
        future.set_result(NS(res=0, robot_return=raw))
        client.call_async.return_value = future
    return transport, feed, pose, clock


def test_initialization_order_and_only_first_two_best_effort(monkeypatch):
    transport, feed, _, _ = synthetic_transport(monkeypatch)
    calls = []

    def call(name, **fields):
        calls.append((name, fields))
        if name in ("StopMoveJog", "DisableRobot"):
            raise ValueError("optional unavailable")
    transport.call = call
    transport.initialize()
    assert [n for n, _ in calls] == [
        "StopMoveJog", "DisableRobot", "EnableRobot", "SpeedFactor", "Tool", "SetTool", "CP"]
    assert calls[3][1] == {"ratio": 100} and calls[5][1]["value"] == "{0,0,0,0,0,0}"
    assert transport.node.events.record.call_count == 2
    calls.clear()

    def strict_failure(name, **fields):
        calls.append(name)
        if name == "EnableRobot":
            raise ValueError("strict failed")
    transport.call = strict_failure
    with pytest.raises(ValueError, match="strict failed"):
        transport.initialize()
    assert calls == ["StopMoveJog", "DisableRobot", "EnableRobot"]


def test_startup_waits_for_every_delayed_response_before_dispatching_next(monkeypatch):
    transport, feed, _, clock = synthetic_transport(monkeypatch)
    sent, pending = [], []

    def send(name):
        assert not pending or pending[-1][0].done(), "Startup commands overlapped"
        future = Future()
        pending.append((future, clock[0] + .06, name))
        sent.append(name)
        return future

    def sleep(dt):
        clock[0] += dt
        if pending:
            future, ready_at, name = pending[-1]
            if not future.done() and clock[0] >= ready_at:
                if name == "DisableRobot":
                    feed["robot_mode"] = 4
                elif name == "EnableRobot":
                    feed["robot_mode"] = 5
                future.set_result(NS(res=0))

    monkeypatch.setattr(hardware.time, "sleep", sleep)
    for name, client in transport.clients.items():
        client.call_async.side_effect = lambda _, n=name: send(n)
    transport.initialize()
    assert sent == ["StopMoveJog", "DisableRobot", "EnableRobot", "SpeedFactor", "Tool",
                    "SetTool", "CP"]
    assert clock[0] >= .42
    assert transport.node.set_execution_state.call_args.args[0] == "READY"


@pytest.mark.parametrize("name", ["StopMoveJog", "DisableRobot"])
def test_unanswered_optional_startup_call_never_advances_or_overlaps(name, monkeypatch):
    transport, _, _, _ = synthetic_transport(monkeypatch)
    pending = Future()
    transport.clients[name].call_async.return_value = pending
    with pytest.raises(hardware.ResponsePending, match=f"Startup {name}.*response timeout"):
        transport.initialize()
    transport.clients["EnableRobot"].call_async.assert_not_called()
    if name == "StopMoveJog":
        transport.clients["DisableRobot"].call_async.assert_not_called()
    with pytest.raises(hardware.ResponsePending, match=f"still awaiting {name}"):
        transport.call("SpeedFactor", ratio=100)
    transport.clients["SpeedFactor"].call_async.assert_not_called()
    pending.set_result(NS(res=0))
    transport.clients["EnableRobot"].call_async.assert_not_called()  # No late auto-advance.


@pytest.mark.parametrize("failure", ["missing", "rejected"])
def test_optional_startup_failures_continue_only_when_no_response_is_pending(failure, monkeypatch):
    transport, _, _, _ = synthetic_transport(monkeypatch)
    for name in ("StopMoveJog", "DisableRobot"):
        if failure == "missing":
            transport.clients[name].service_is_ready.return_value = False
        else:
            reply = Future()
            reply.set_result(NS(res=-1))
            transport.clients[name].call_async.return_value = reply
    transport.initialize()
    transport.clients["EnableRobot"].call_async.assert_called_once()
    transport.clients["CP"].call_async.assert_called_once()
    warnings = [c for c in transport.node.events.record.call_args_list
                if c.args[:2] == ("WARNING", "startup_best_effort")]
    assert [c.kwargs["service"] for c in warnings] == ["StopMoveJog", "DisableRobot"]


@pytest.mark.parametrize("name", ["EnableRobot", "SpeedFactor", "Tool", "SetTool", "CP"])
def test_strict_startup_failure_identifies_call_and_sends_no_later_setting(name, monkeypatch):
    transport, _, _, _ = synthetic_transport(monkeypatch)
    reply = Future()
    reply.set_result(NS(res=-2))
    transport.clients[name].call_async.return_value = reply
    order = ["StopMoveJog", "DisableRobot", "EnableRobot", "SpeedFactor", "Tool", "SetTool", "CP"]
    with pytest.raises(ValueError, match=f"Startup {name}: {name} failed: -2"):
        transport.initialize()
    for later in order[order.index(name) + 1:]:
        transport.clients[later].call_async.assert_not_called()


def test_missing_strict_startup_service_is_bounded_and_named_before_preconditioning(monkeypatch):
    transport, _, _, clock = synthetic_transport(monkeypatch)
    transport.clients["Tool"].service_is_ready.return_value = False
    with pytest.raises(ValueError, match="Startup required services unavailable: Tool"):
        transport.initialize()
    assert clock[0] >= hardware.SERVICE_TIMEOUT_SEC
    assert all(client.call_async.call_count == 0 for client in transport.clients.values())


def test_startup_calls_complete_but_paused_feedback_does_not_claim_ready(monkeypatch):
    transport, _, _, _ = synthetic_transport(monkeypatch)
    original = transport.node.feedback_snapshot

    def snapshot(*, enabled):
        if enabled:
            raise ValueError("Robot readiness blocked: isPauseCmdFlag=1")
        return original(enabled=enabled)

    transport.node.feedback_snapshot = snapshot
    transport.initialize()
    transport.clients["CP"].call_async.assert_called_once()
    assert transport.node.set_execution_state.call_args.args == (
        "FAILED", "Startup calls completed; Robot readiness blocked: isPauseCmdFlag=1")
    assert transport.node.startup_settings_applied


def test_final_readiness_waits_for_transient_disabled_feedback_without_another_command(monkeypatch):
    transport, _, _, clock = synthetic_transport(monkeypatch)
    original = transport.node.feedback_snapshot
    first_read = []

    def snapshot(*, enabled):
        if enabled:
            if not first_read:
                first_read.append(clock[0])
            if clock[0] - first_read[0] < .12:
                raise ValueError("Robot readiness blocked: RobotStatus.is_enable=False")
        return original(enabled=enabled)

    transport.node.feedback_snapshot = snapshot
    transport.initialize()
    assert transport.node.set_execution_state.call_args.args[0] == "READY"
    transport.clients["EnableRobot"].call_async.assert_called_once()
    assert clock[0] - first_read[0] >= .12


def test_explicit_enable_sends_only_enable_and_confirms_readiness(monkeypatch):
    transport, _, _, _ = synthetic_transport(monkeypatch)
    transport.enable_robot()
    assert transport.clients["EnableRobot"].call_async.call_count == 1
    assert all(client.call_async.call_count == 0 for name, client in transport.clients.items()
               if name != "EnableRobot")
    assert transport.node.set_execution_state.call_args.args == (
        "READY", "EnableRobot confirmed; no Home or Pick sent")


@pytest.mark.parametrize("blocker", ["disabled", "paused", "running"])
def test_explicit_enable_never_bypasses_blocked_feedback_or_retries(blocker, monkeypatch):
    transport, feed, _, clock = synthetic_transport(monkeypatch)
    original = transport.node.feedback_snapshot

    def snapshot(*, enabled):
        if enabled:
            if blocker != "running":
                reason = ("RobotStatus.is_enable=False" if blocker == "disabled"
                          else "isPauseCmdFlag=1")
                raise ValueError(f"Robot readiness blocked: {reason}")
            feed["RunningStatus"] = 1
        return original(enabled=enabled)

    transport.node.feedback_snapshot = snapshot
    with pytest.raises(ValueError, match="Robot readiness blocked"):
        transport.enable_robot()
    assert clock[0] >= hardware.SERVICE_TIMEOUT_SEC
    transport.clients["EnableRobot"].call_async.assert_called_once()
    assert transport.node.set_execution_state.call_args.args[0] != "READY"


def enable_service_node():
    feed = {"robot_mode": 4, "ErrorStatus": 0, "CollisionStates": 0,
            "isRunQueuedCmd": 0, "RunningStatus": 0, "digital_input_bits": 0}
    node = NS(live=True, debug=False, startup_settings_applied=True, holding_item=False,
              fatal_error=None, shutdown_requested=threading.Event(),
              state_lock=threading.RLock(), action_lock=threading.Lock(),
              action_thread=None, stop_thread=None, stop_future=None, cancel=threading.Event(),
              hardware=NS(moving=False, enable_robot=MagicMock()),
              check_command_owner=MagicMock(),
              feedback_snapshot=MagicMock(return_value={"feed": feed}),
              clear_preview=MagicMock(), set_execution_state=MagicMock(), events=MagicMock())
    node._run_enable_robot = MethodType(controller.RobotController._run_enable_robot, node)
    return node, feed


def test_enable_service_can_recover_disabled_after_completed_startup_without_teach_or_auto_pick():
    node, _ = enable_service_node()
    node.cancel.set()
    response = controller.RobotController._enable_robot_service(node, None, NS())
    assert response.success
    node.action_thread.join(timeout=1)
    assert not node.action_thread.is_alive() and not node.action_lock.locked()
    node.hardware.enable_robot.assert_called_once()
    node.check_command_owner.assert_called_once_with("EnableRobot")
    assert not node.cancel.is_set()


@pytest.mark.parametrize("blocker", [
    "live_off", "startup", "holding", "moving", "action", "recovery", "fault", "collision",
    "queued", "running", "suction", "mode", "stale", "owner", "stop_pending", "shutdown",
])
def test_enable_service_rejects_unsafe_or_incomplete_states_before_dispatch(blocker):
    node, feed = enable_service_node()
    if blocker == "live_off":
        node.live, node.debug = False, True
    elif blocker == "startup":
        node.startup_settings_applied = False
    elif blocker == "holding":
        node.holding_item = True
    elif blocker == "moving":
        node.hardware.moving = True
    elif blocker in ("action", "recovery"):
        setattr(node, "action_thread" if blocker == "action" else "stop_thread",
                NS(is_alive=lambda: True))
    elif blocker in ("fault", "collision", "queued", "running", "suction", "mode"):
        fields = {"fault": "ErrorStatus", "collision": "CollisionStates",
                  "queued": "isRunQueuedCmd", "running": "RunningStatus",
                  "suction": "digital_input_bits", "mode": "robot_mode"}
        key = fields[blocker]
        feed[key] = 7 if blocker == "mode" else 1
    elif blocker == "stale":
        node.feedback_snapshot.side_effect = ValueError("stale feedback")
    elif blocker == "owner":
        node.check_command_owner.side_effect = ValueError("competing owner")
    elif blocker == "stop_pending":
        node.stop_future = Future()
    elif blocker == "shutdown":
        node.shutdown_requested.set()
    response = controller.RobotController._enable_robot_service(node, None, NS())
    assert not response.success and not node.action_lock.locked()
    node.hardware.enable_robot.assert_not_called()


def test_failed_enable_worker_is_cancelled_and_does_not_restart_startup():
    node, _ = enable_service_node()
    node.hardware.enable_robot.side_effect = ValueError("EnableRobot rejected")
    node.action_lock.acquire()
    controller.RobotController._run_enable_robot(node)
    assert node.cancel.is_set() and not node.action_lock.locked()
    assert node.set_execution_state.call_args.args == (
        "FAILED", "EnableRobot: EnableRobot rejected")
    node.hardware.enable_robot.assert_called_once()


def test_home_transport_uses_relative_z_then_joint_movlio_and_confirms_completion(monkeypatch):
    transport, _, pose, clock = synthetic_transport(monkeypatch)
    current = pose.copy()
    current[2, 3] -= .1
    transport.current_pose = lambda: current
    for target in home_plan(current, pose, [.1] * 6):
        transport.move(target)
    relative = transport.clients["RelMovLUser"].call_async.call_args.args[0]
    assert relative.a == relative.b == 0 and relative.c == pytest.approx(100)
    final = transport.clients["MovLIO"].call_async.call_args.args[0]
    assert final.mode and final.mdis == []
    assert relative.param_value == final.param_value == ["user=0", "tool=0", "v=100", "a=100"]
    assert final.a == pytest.approx(math.degrees(.1))
    assert clock[0] >= .6 and not transport.moving


def test_editable_home_rates_reach_both_relative_and_movlio_services(monkeypatch):
    transport, _, pose, _ = synthetic_transport(monkeypatch)
    current = pose.copy()
    current[2, 3] -= .1
    transport.current_pose = lambda: current
    targets = home_targets(current, pose, [.1]*6, speed_percent=42, acceleration_percent=73)
    for target in targets:
        transport.move(target)
    for name in ("RelMovLUser", "MovLIO"):
        request = transport.clients[name].call_async.call_args.args[0]
        assert request.param_value == ["user=0", "tool=0", "v=42", "a=73"]


def test_direct_home_dispatches_only_joint_movlio(monkeypatch):
    transport, _, pose, _ = synthetic_transport(monkeypatch)
    current = pose.copy()
    current[2, 3] += .1
    for target in home_plan(current, pose, [.1] * 6):
        transport.move(target)
    transport.clients["RelMovLUser"].call_async.assert_not_called()
    transport.clients["MovLIO"].call_async.assert_called_once()


def test_default_and_edited_pick_rates_reach_each_movlio_command(monkeypatch):
    transport, _, pose, _ = synthetic_transport(monkeypatch)
    for target in plan():
        transport.move(replace(target, matrix=pose))
    requests = transport.clients["MovLIO"].call_async.call_args_list
    assert [c.args[0].param_value for c in requests] == [
        ["user=0", "tool=0", f"v={v}", "a=100"] for v in (100, 100, 100, 6, 6, 100)]
    transport.move(replace(plan()[3], matrix=pose, speed_percent=11, acceleration_percent=25))
    request = transport.clients["MovLIO"].call_async.call_args.args[0]
    assert request.param_value == ["user=0", "tool=0", "v=11", "a=25"]
    transport.clients["SpeedFactor"].call_async.assert_not_called()


def test_controller_home_uses_loaded_profile_motion_rates(pair):
    _, _, profile = pair
    profile["speed"]["travel_percent"] = 42
    profile["acceleration"]["travel_percent"] = 73
    fake = FakeHardware()
    fake.pose = model().forward(profile["home"]["positions_rad"])
    node = NS(check_cancelled=lambda: None, kinematics=model(), debug=False, hardware=fake)
    targets = controller.RobotController.home(node, profile)
    assert [(t.speed_percent, t.acceleration_percent) for t in targets] == [(42, 73)]
    assert [v[4:] for v in fake.trace if v[0] == "move"] == [(42, 73)]


def test_di1_before_descent_stops_without_dispatching_descent(monkeypatch):
    transport, feed, pose, _ = synthetic_transport(monkeypatch)
    feed["digital_input_bits"] = 1
    target = home_plan(pose, pose, [.1] * 6)[-1]
    assert transport.move(target, stop_on_suction=True)
    transport.clients["Stop"].call_async.assert_called_once()
    transport.clients["MovLIO"].call_async.assert_not_called()
    assert not transport.moving


def test_di1_during_descent_waits_for_stop_and_fresh_stationary_feedback(monkeypatch):
    transport, feed, pose, clock = synthetic_transport(monkeypatch)
    original = transport.clients["MovLIO"].call_async.return_value

    def dispatch(_):
        feed["digital_input_bits"] = 1
        return original
    transport.clients["MovLIO"].call_async.side_effect = dispatch
    assert transport.move(home_plan(pose, pose, [.1] * 6)[-1], stop_on_suction=True)
    assert clock[0] >= .3 and not transport.moving
    transport.clients["Stop"].call_async.assert_called_once()


def test_stop_rejection_and_lost_suction_block_retract(monkeypatch):
    transport, feed, pose, _ = synthetic_transport(monkeypatch)
    target = home_plan(pose, pose, [.1] * 6)[-1]
    with pytest.raises(ValueError, match="Suction lost"):
        transport.move(target, require_suction=True)
    feed["digital_input_bits"] = 1
    future = Future()
    future.set_result(NS(res=1))
    transport.clients["Stop"].call_async.return_value = future
    with pytest.raises(ValueError, match="Stop rejected"):
        transport.move(target, stop_on_suction=True)
    assert transport.moving


def test_operator_stop_confirmation_ignores_action_cancel_but_requires_stationary(monkeypatch):
    transport, _, _, clock = synthetic_transport(monkeypatch)
    transport.node.cancel.set()
    transport.node.check_cancelled.side_effect = RuntimeError("action cancelled")
    future = Future()
    future.set_result(NS(res=0))
    transport.moving = True
    transport.confirm_stop(future)
    assert clock[0] >= hardware.STATIONARY_SEC and not transport.moving
    assert transport.node.check_cancelled.call_count == 0

    rejected = Future()
    rejected.set_result(NS(res=1))
    with pytest.raises(ValueError, match="Stop rejected"):
        transport.confirm_stop(rejected)


@pytest.mark.parametrize("raw", ["", "0,{1,2},GetPose();", "0,{nan,2,3,4,5,6},GetPose();",
                                 "1,{1,2,3,4,5,6},GetPose();"])
def test_malformed_getpose_is_not_accepted(raw):
    with pytest.raises(ValueError):
        hardware.robot_values(raw, "GetPose")


def test_ui_prefill_is_strict_unapplied_and_atomic(tmp_path):
    path = tmp_path / "last_session.json"
    assert ui_state.load_state(path) is None
    ui_state.save_state(path, "item.yaml", None)
    assert ui_state.load_state(path) == {"schema_version": 1, "item": "item.yaml", "bin": None}
    ui_state.save_state(path, "new.yaml", "bin.yaml")
    assert ui_state.load_state(path)["item"] == "new.yaml"
    path.write_text('{"schema_version":0}')
    with pytest.raises(ValueError):
        ui_state.save_state(path, "item.yaml", "bin.yaml")
    assert path.read_text() == '{"schema_version":0}'


def test_runtime_catalog_uses_strict_copied_pairs_and_no_implicit_choice(pair, monkeypatch):
    root, path, _ = pair
    # Share existing synthetic Bin Teach fixture builders, not operator station artifacts.
    helpers = Path(__file__).resolve().parents[2] / "item_perception_yolo/test"
    monkeypatch.syspath_prepend(str(helpers))
    from test_bin_teach import _applied, _capture
    from item_perception_yolo import bin_teach_core as bin_core
    applied = _applied(root)
    capture = _capture(applied)
    destination = bin_core.bin_output_path("192.168.20.204", root=root,
                                           created_at=datetime.fromisoformat(
                                               capture.captured_at_utc.replace("Z", "+00:00")))
    bin_core.write_bin_teach(
        destination, applied, bin_core.BinArucoSettings("DICT_5X5_50", 60.), capture, root=root)
    monkeypatch.setattr(profiles, "latest_station_calibration", lambda **_: applied)
    runtime = root / "runtime_teach"
    runtime.mkdir()
    for source in (path, path.with_suffix(".pt"), destination):
        shutil.copy2(source, runtime / source.name)
    selected = profiles.runtime_selection(root)
    assert selected.deployment and selected.item_path.parent == runtime
    assert selected.bin.points == bin_core.load_bin_teach(destination, root=root).points
    with pytest.raises(ValueError, match="offline_teach"):
        controller.load_item_profile(selected.item_path, root=root)
    with pytest.raises(ValueError, match="runtime_teach"):
        controller.load_item_profile(path, root=root, deployment=True)
    extra = runtime / "second.yaml"
    shutil.copy2(path, extra)
    with pytest.raises(ValueError, match="exactly one"):
        profiles.runtime_selection(root)
    extra.unlink()
    (runtime / "unknown.pt").write_bytes(b"opaque")
    with pytest.raises(ValueError, match="paired .pt"):
        profiles.runtime_selection(root)


def test_supervisor_uses_signatures_not_weight_hashing_and_clears_changed_preview(tmp_path):
    path = tmp_path / "profile.yaml"
    path.write_text("first")
    node = NS(state_lock=threading.RLock(), cancel=threading.Event(), preview_targets=plan(),
              preview_sources=((path, controller.source_signature(path)),), hardware=None,
              tf_broadcaster=MagicMock(), events=MagicMock(),
              get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: __import__(
                  "builtin_interfaces.msg", fromlist=["Time"]).Time())))
    node.clear_preview = MethodType(controller.RobotController.clear_preview, node)
    controller.RobotController._supervise(node)
    assert len(node.tf_broadcaster.sendTransform.call_args.args[0]) == 6
    path.write_text("changed")
    controller.RobotController._supervise(node)
    assert not node.preview_targets


def test_debug_gui_is_unapplied_and_has_no_hardware_transport(pair, monkeypatch):
    import rclpy
    from PyQt5 import QtWidgets
    from robot_controller.gui import ControllerWindow
    root, path, _ = pair
    monkeypatch.setattr(controller, "workspace_root", lambda: root)
    monkeypatch.setattr(controller, "load_robot_lan1_ip", lambda _: "192.168.20.204")
    monkeypatch.setattr(controller, "_parse_env_file", lambda _: {
        "DOBOT_ROBOT_NODE_NAME": "dobot_bringup_ros2"})
    ui_state.save_state(root / "logs/robot_controller/last_session.json", path, None)
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    assert application is not None
    rclpy.init(args=[])
    node = controller.RobotController()
    window = None
    try:
        assert node.debug and node.hardware is None and len(list(node.clients)) == 1
        window = ControllerWindow(node)
        assert node.profile_path is None and not window.home.isEnabled()
        assert window.item_path.text() == str(path)
        window.load_selected()
        assert window.home.isEnabled() and not window.pick.isEnabled()
        assert node.profile_path == path and node.action_thread is None
        with pytest.raises(RuntimeError, match="Live is OFF"):
            node.check_command_owner("EnableRobot")
        assert set(window.service_clients) == {
            "home", "pick", "stop", "live", "debug_images", "enable"}
        assert not window.enable.isEnabled()  # Live OFF can never enable hardware.
        invoke = MagicMock()
        window._call_service = invoke
        window.home.click()
        assert invoke.call_args.args[0] == "home"
        window.stop.click()
        assert invoke.call_args.args[0] == "stop"
        window.live.click()
        assert invoke.call_args.args[0] == "live"
        assert "#b51f24" in window.live.styleSheet()
        window.debug_images.click()
        assert invoke.call_args.args[0] == "debug_images"
        assert "#d77b00" in window.debug_images.styleSheet()
        node.live, node.debug = True, False
        node.hardware = NS(moving=False)
        node.startup_settings_applied = True
        node.execution_state = "FAILED"
        window.refresh()
        assert window.enable.isEnabled()
        window.enable.click()
        assert invoke.call_args.args[0] == "enable"
        node.hardware = None
        node.live, node.debug = False, True
    finally:
        if window is not None:
            window.close()
        node.close_runtime()
        node.destroy_node()
        rclpy.shutdown()


def test_debug_pick_converts_full_platform_transform_and_publishes_all_candidates(pair):
    root, path, profile = pair
    home = pose_matrix([100, 200, 800, 180, 0, 0])
    platform = pose_matrix([200, 100, 50, 5, 10, 20])
    selected = NS(validate=MagicMock(), bin=NS(sha256="b"),
                  station=NS(camera=NS(sha256="c"), platform=NS(sha256="p",
                             base_from_platform=platform)))
    digest = controller.file_profile_digest(path, root)
    batch = {"evidence": {"camera_sha256": "c", "platform_sha256": "p",
                          "bin_sha256": "b", "model_sha256": profile["model"]["sha256"]},
             "observation_stamp_ns": 100_000_000_000, "depth_stamp_ns": 100_000_000_000,
             "targets": [{"position_m": [.1, .2, .05]}, {"position_m": [.2, .1, .05]}]}
    node = NS(profile_path=path, root=root, headless=False, debug=True, hardware=None,
              state_lock=threading.RLock(), last_prepick=None, last_gripper=None,
              summary={"profile_sha256": digest}, selection=selected, action_lock=threading.Lock(),
              pose_client=MagicMock(), check_detector_owner=MagicMock(),
              kinematics=NS(forward=lambda _: home), current_joints=lambda: [.1]*6,
              check_cancelled=MagicMock(), clear_preview=MagicMock(), install_preview=MagicMock(),
              set_execution_state=MagicMock(), events=MagicMock(),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_100_000_000)),
              _request_poses=lambda *_: NS(success=True, message=json.dumps(batch)))
    node.home = MethodType(controller.RobotController.home, node)
    node.pick = MethodType(controller.RobotController.pick, node)
    node.action_lock.acquire()
    controller.RobotController._run_action(node, "pick")
    assert not node.action_lock.locked()
    targets = node.install_preview.call_args.args[0]
    assert len(targets) == 17
    expected = platform @ [.1, .2, .05, 1.]
    np.testing.assert_allclose(targets[4].matrix[:3, 3],
                               [*expected[:2], expected[2] + .01])
    assert node.set_execution_state.call_args.args[0] == "DEBUG"
    # Hash disagreement must clear rather than install executable/preview targets.
    node.install_preview.reset_mock()
    batch["evidence"]["bin_sha256"] = "wrong"
    node.action_lock.acquire()
    controller.RobotController._run_action(node, "pick")
    node.install_preview.assert_not_called()
    assert "mismatch" in node.set_execution_state.call_args.args[1]
    node.home = MagicMock()
    node.pose_client.service_is_ready.return_value = False
    node.action_lock.acquire()
    controller.RobotController._run_action(node, "pick")
    node.home.assert_not_called()
    assert "independently armed" in node.set_execution_state.call_args.args[1]


def test_feedback_staleness_includes_controller_timer_not_only_republished_feed():
    node = NS(state_lock=threading.RLock(), _sole_publisher=MagicMock(), current_joints=MagicMock(),
              robot_feedback=(True, True, hardware.time.monotonic()), feed_sequence=4,
              controller_progress_at=hardware.time.monotonic()-2,
              feed_feedback=({}, hardware.time.monotonic()))
    with pytest.raises(ValueError, match="stale"):
        controller.RobotController.feedback_snapshot(node, enabled=True)


def test_feedback_fault_and_nonzero_tool_block_execution():
    now = hardware.time.monotonic()
    feed = {"robot_mode": 5, "EnableStatus": 1,
            "ErrorStatus": 0, "CollisionStates": 0, "isPauseCmdFlag": 0,
            "userCoordinate": 0, "toolCoordinate": 1}
    node = NS(state_lock=threading.RLock(), _sole_publisher=MagicMock(), current_joints=MagicMock(),
              robot_feedback=(True, True, now), controller_progress_at=now,
              feed_feedback=(feed, now), feed_sequence=3)
    with pytest.raises(ValueError, match="nonzero user/tool"):
        controller.RobotController.feedback_snapshot(node, enabled=True)


@pytest.mark.parametrize("field,value", [
    ("robot_mode", 4), ("EnableStatus", 0), ("ErrorStatus", 1),
    ("CollisionStates", 1), ("isPauseCmdFlag", 1),
    ("userCoordinate", 2), ("toolCoordinate", 1),
])
def test_readiness_error_names_exact_feedback_blocker(field, value):
    now = hardware.time.monotonic()
    feed = {"robot_mode": 5, "EnableStatus": 1,
            "ErrorStatus": 0, "CollisionStates": 0, "isPauseCmdFlag": 0,
            "userCoordinate": 0, "toolCoordinate": 0}
    feed[field] = value
    node = NS(state_lock=threading.RLock(), _sole_publisher=MagicMock(), current_joints=MagicMock(),
              robot_feedback=(True, True, now), controller_progress_at=now,
              feed_feedback=(feed, now), feed_sequence=3)
    with pytest.raises(ValueError, match=f"{field}={value}"):
        controller.RobotController.feedback_snapshot(node, enabled=True)
    assert controller.RobotController.feedback_snapshot(node, enabled=False)["feed"] == feed


def test_readiness_error_lists_all_blockers_instead_of_generic_fault():
    now = hardware.time.monotonic()
    feed = {"robot_mode": 4, "EnableStatus": 0,
            "ErrorStatus": 1, "CollisionStates": 0, "isPauseCmdFlag": 1,
            "userCoordinate": 0, "toolCoordinate": 2}
    node = NS(state_lock=threading.RLock(), _sole_publisher=MagicMock(), current_joints=MagicMock(),
              robot_feedback=(True, False, now), controller_progress_at=now,
              feed_feedback=(feed, now), feed_sequence=3)
    with pytest.raises(ValueError) as failure:
        controller.RobotController.feedback_snapshot(node, enabled=True)
    assert all(reason in str(failure.value) for reason in (
        "RobotStatus.is_enable=False", "robot_mode=4", "ErrorStatus=1", "isPauseCmdFlag=1",
        "toolCoordinate=2"))


def test_service_timeout_late_motion_ack_receives_safety_stop(monkeypatch):
    transport, _, _, _ = synthetic_transport(monkeypatch)
    pending = Future()
    transport.clients["MovLIO"].call_async.return_value = pending
    with pytest.raises(ValueError, match="timeout"):
        transport.call("MovLIO")
    transport.node.cancel.set()
    pending.set_result(NS(res=0))
    transport.clients["Stop"].call_async.assert_called_once()


def test_suction_monitor_interrupts_before_slow_motion_ack(monkeypatch):
    transport, feed, _, _ = synthetic_transport(monkeypatch)
    pending = Future()
    transport.clients["MovLIO"].call_async.return_value = pending
    feed["digital_input_bits"] = 1
    original_snapshot = transport.node.feedback_snapshot

    def snapshot(**kw):
        if transport.clients["Stop"].call_async.call_count and not pending.done():
            pending.set_result(NS(res=0))
        return original_snapshot(**kw)
    transport.node.feedback_snapshot = snapshot
    assert transport.call("MovLIO", monitor_suction=True).res == 0
    assert transport.suction_interrupted
    # Contain a motion acknowledged after the first acquisition Stop.
    assert transport.clients["Stop"].call_async.call_count == 2


def test_real_supervisor_fault_requests_stop_and_never_claims_completion():
    node = NS(state_lock=threading.RLock(), preview_targets=(), preview_sources=(),
              hardware=NS(moving=True), execution_state="BUSY", cancel=threading.Event(),
              feedback_snapshot=MagicMock(side_effect=ValueError("stale")), holding_item=False,
              clear_preview=MagicMock(), request_stop=MagicMock(), set_execution_state=MagicMock())
    controller.RobotController._supervise(node)
    assert node.cancel.is_set()
    node.request_stop.assert_called_once()
    node.set_execution_state.assert_called_once_with("FAILED", "stale")


def test_headless_node_loads_runtime_profile_and_starts_permanently_live(pair, monkeypatch):
    import rclpy
    root, path, _ = pair
    directory = root / "runtime_teach"
    directory.mkdir()
    for source in (path, path.with_suffix(".pt")):
        shutil.copy2(source, directory / source.name)
    selected = NS(item_path=directory / path.name)
    monkeypatch.setattr(controller, "workspace_root", lambda: root)
    monkeypatch.setattr(controller, "load_robot_lan1_ip", lambda _: "192.168.20.204")
    monkeypatch.setattr(controller, "_parse_env_file", lambda _: {
        "DOBOT_ROBOT_NODE_NAME": "dobot_bringup_ros2"})
    monkeypatch.setattr(controller, "runtime_selection", lambda _: selected)
    initialized = threading.Event()

    class FakeStartup:
        moving = False

        def __init__(self, node):
            self.node = node

        def initialize(self):
            self.node.set_execution_state("READY", "Synthetic initialization complete")
            initialized.set()

        def close(self):
            self.closed = True

    monkeypatch.setattr(hardware, "DobotHardware", FakeStartup)
    rclpy.init(args=["--ros-args", "-p", "headless:=true"])
    node = controller.RobotController()
    try:
        assert initialized.wait(1)
        node.action_thread.join(timeout=1)
        assert node.headless and node.live and not node.debug
        assert isinstance(node.hardware, FakeStartup)
        assert node.profile_path.parent == directory
        service_names = {service.srv_name for service in node.services}
        assert {"/robot_controller/go_home", "/robot_controller/pick_item",
                "/robot_controller/stop", "/robot_controller/set_live",
                "/robot_controller/set_debug_images",
                "/robot_controller/enable_robot"} <= service_names
        response = controller.RobotController._set_live_service(
            node, controller.SetBool.Request(data=False), NS())
        assert not response.success and node.live
        from rclpy.parameter import Parameter
        result = node.set_parameters([Parameter("item_teach_file", value=str(path))])[0]
        assert not result.successful
    finally:
        node.close_runtime()
        node.destroy_node()
        rclpy.shutdown()


def test_live_service_constructs_initializes_and_removes_transport(pair, monkeypatch):
    import rclpy
    root, _, _ = pair
    initialized = threading.Event()
    constructed = []

    class FakeStartup:
        moving = False

        def __init__(self, node):
            self.node = node
            constructed.append(self)

        def initialize(self):
            self.node.set_execution_state("READY", "Synthetic initialization complete")
            initialized.set()

        def close(self):
            self.closed = True

    monkeypatch.setattr(hardware, "DobotHardware", FakeStartup)
    monkeypatch.setattr(controller, "workspace_root", lambda: root)
    monkeypatch.setattr(controller, "load_robot_lan1_ip", lambda _: "192.168.20.204")
    monkeypatch.setattr(controller, "_parse_env_file", lambda _: {
        "DOBOT_ROBOT_NODE_NAME": "dobot_bringup_ros2"})
    rclpy.init(args=[])
    node = controller.RobotController()
    try:
        assert node.debug and not node.live and node.hardware is None
        response = controller.RobotController._set_live_service(
            node, controller.SetBool.Request(data=True), NS())
        assert response.success and initialized.wait(1)
        node.action_thread.join(timeout=1)
        started = node.hardware
        assert node.live and not node.debug and isinstance(started, FakeStartup)
        assert node.execution_state == "READY" and node.profile_path is None
        assert not node.preview_targets and not node.holding_item
        node.holding_item = True
        response = controller.RobotController._set_live_service(
            node, controller.SetBool.Request(data=False), NS())
        assert not response.success and node.live
        node.holding_item = False
        response = controller.RobotController._set_live_service(
            node, controller.SetBool.Request(data=False), NS())
        assert response.success and not node.live and node.debug and node.hardware is None
        assert started.closed
    finally:
        node.close_runtime()
        node.destroy_node()
        rclpy.shutdown()
    state = root / "logs/robot_controller/last_session.json"
    state.write_text('{"schema_version":0}')
    constructed.clear()
    rclpy.init(args=[])
    try:
        with pytest.raises(ValueError, match="exact schema 1"):
            controller.RobotController()
        assert not constructed
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize("suction", [False, True])
def test_operator_stop_confirms_stationary_and_conditionally_returns_item(suction, monkeypatch):
    monkeypatch.setattr(controller.rclpy, "ok", lambda: True)
    target = plan()[2]
    transport = NS(confirm_stop=MagicMock(), move=MagicMock(), output=MagicMock())
    node = NS(hardware=transport, action_lock=threading.Lock(), cancel=threading.Event(),
              stop_future=object(), last_prepick=target, last_gripper=settings()["gripper"],
              holding_item=False, state_lock=threading.RLock(), events=MagicMock(),
              stop_recovery_abort=threading.Event(), shutdown_requested=threading.Event(),
              feedback_snapshot=MagicMock(return_value={
                  "feed": {"digital_input_bits": int(suction)}}),
              set_execution_state=MagicMock())
    node.cancel.set()
    controller.RobotController._complete_operator_stop(node, object(), None)
    transport.confirm_stop.assert_called_once()
    if suction:
        transport.move.assert_called_once_with(target, require_suction=True)
        assert transport.output.call_args_list == [
            call(13, False), call(1, True), call(2, False), call(14, True),
        ]
        assert node.last_prepick is None and not node.holding_item
        assert node.set_execution_state.call_args.args == (
            "READY", "Stopped; item returned and released at pre-pick")
    else:
        transport.move.assert_not_called()
        transport.output.assert_not_called()
        assert node.set_execution_state.call_args.args == (
            "READY", "Stopped; DI1 is OFF, no return motion")
    assert not node.cancel.is_set() and not node.action_lock.locked()


def test_operator_stop_recovery_fault_never_releases_item(monkeypatch):
    monkeypatch.setattr(controller.rclpy, "ok", lambda: True)
    transport = NS(confirm_stop=MagicMock(), move=MagicMock(side_effect=ValueError("blocked")),
                   output=MagicMock())
    node = NS(hardware=transport, action_lock=threading.Lock(), cancel=threading.Event(),
              stop_future=object(), last_prepick=plan()[2],
              last_gripper=settings()["gripper"], holding_item=False,
              state_lock=threading.RLock(), events=MagicMock(),
              stop_recovery_abort=threading.Event(), shutdown_requested=threading.Event(),
              feedback_snapshot=MagicMock(return_value={"feed": {"digital_input_bits": 1}}),
              set_execution_state=MagicMock())
    controller.RobotController._complete_operator_stop(node, object(), None)
    assert node.holding_item and node.cancel.is_set()
    transport.output.assert_not_called()
    assert node.set_execution_state.call_args.args[0] == "FAILED"


@pytest.mark.parametrize("reason", ["second_stop", "shutdown"])
@pytest.mark.parametrize("when", ["before", "after_motion"])
def test_stop_recovery_never_resumes_after_another_stop_or_shutdown(reason, when, monkeypatch):
    monkeypatch.setattr(controller.rclpy, "ok", lambda: True)
    transport = NS(confirm_stop=MagicMock(), move=MagicMock(), output=MagicMock())
    node = NS(hardware=transport, action_lock=threading.Lock(), cancel=threading.Event(),
              stop_future=object(), last_prepick=plan()[2], last_gripper=settings()["gripper"],
              holding_item=True, state_lock=threading.RLock(), events=MagicMock(),
              stop_recovery_abort=threading.Event(), shutdown_requested=threading.Event(),
              feedback_snapshot=MagicMock(return_value={"feed": {"digital_input_bits": 1}}),
              set_execution_state=MagicMock())
    node.cancel.set()
    flag = node.stop_recovery_abort if reason == "second_stop" else node.shutdown_requested
    if when == "before":
        flag.set()
    else:
        transport.move.side_effect = lambda *_args, **_kwargs: flag.set()
    controller.RobotController._complete_operator_stop(node, object(), None)
    assert transport.move.call_count == int(when == "after_motion")
    transport.output.assert_not_called()
    assert node.cancel.is_set() and node.holding_item
    assert node.set_execution_state.call_args.args[0] == "FAILED"


def test_second_stop_sends_stop_and_aborts_recovery_without_starting_another():
    transport = NS(stop=MagicMock(return_value=object()))
    node = NS(live=True, hardware=transport, stop_thread=NS(is_alive=lambda: True),
              stop_recovery_abort=threading.Event(), cancel=threading.Event(),
              state_lock=threading.RLock(), execution_message="Stopped",
              set_execution_state=MagicMock())
    response = controller.RobotController._stop_service(node, None, NS())
    assert response.success and node.cancel.is_set() and node.stop_recovery_abort.is_set()
    transport.stop.assert_called_once()


@pytest.mark.parametrize("kind", ["partition", "unknown", "malformed", "symlink"])
def test_runtime_catalog_rejects_bad_entries_without_fallback(tmp_path, kind):
    directory = tmp_path / "runtime_teach"
    directory.mkdir()
    if kind == "partition":
        (directory / "items").mkdir()
    elif kind == "unknown":
        (directory / "unknown.yaml").write_text("artifact_type: tray_teach\n")
    elif kind == "malformed":
        (directory / "bad.yaml").write_text("[not yaml")
    else:
        outside = tmp_path / "outside.yaml"
        outside.write_text("artifact_type: item_teach\n")
        (directory / "item.yaml").symlink_to(outside)
    with pytest.raises(ValueError):
        profiles.runtime_selection(tmp_path)


def test_unknown_duplicate_command_provider_and_legacy_clients_are_blocked():
    nodes = [("robot_controller", "/"), ("dobot_bringup_ros2", "/")]
    services = {("dobot_bringup_ros2", "/"): [("/dobot_bringup_ros2/srv/MovLIO", [])]}
    node = NS(debug=False, live=True, publisher_node="/dobot_bringup_ros2",
              get_name=lambda: "robot_controller", get_namespace=lambda: "/",
              get_node_names_and_namespaces=lambda: nodes,
              get_service_names_and_types_by_node=lambda name, ns: services.get((name, ns), []))
    controller.RobotController.check_command_owner(node, "MovLIO")
    nodes.append(("unknown", "/"))
    services[("unknown", "/")] = services[("dobot_bringup_ros2", "/")]
    with pytest.raises(ValueError, match="Duplicate command-service"):
        controller.RobotController.check_command_owner(node, "MovLIO")
    nodes[:] = nodes[:2] + [("motion_debug_gui", "/")]
    with pytest.raises(ValueError, match="Competing legacy"):
        controller.RobotController.check_command_owner(node, "MovLIO")


def test_acknowledgement_with_nonempty_queue_is_not_motion_completion(monkeypatch):
    transport, feed, pose, _ = synthetic_transport(monkeypatch)
    feed["isRunQueuedCmd"] = 1
    target = home_plan(pose, pose, [.1]*6)[-1]
    with pytest.raises(ValueError, match="completion timeout"):
        transport.move(target)
    assert transport.moving  # Caller must contain the ambiguous in-progress command.


def test_bad_ik_blocks_motion_before_command_dispatch(monkeypatch):
    transport, _, pose, _ = synthetic_transport(monkeypatch)
    future = Future()
    future.set_result(NS(res=0, robot_return="0,{0,0,0,0,0,0},InverseKin();"))
    transport.clients["InverseKin"].call_async.return_value = future
    current = pose.copy()
    current[2, 3] -= .1
    with pytest.raises(ValueError, match="InverseKin does not match"):
        transport.move(home_plan(current, pose, [.1]*6)[0])
    transport.clients["RelMovLUser"].call_async.assert_not_called()
    assert not transport.moving


def test_settle_window_and_do_feedback_confirmation(monkeypatch):
    transport, feed, _, clock = synthetic_transport(monkeypatch)
    assert not transport.sensor(True, .2, settling_sec=0)
    assert clock[0] >= .2
    future = transport.clients["DO"].call_async.return_value

    def output(request):
        feed["digital_outputs"] |= 1 << (request.index-1)
        return future

    transport.clients["DO"].call_async.side_effect = output
    transport.output(13, True)
    assert feed["digital_outputs"] & (1 << 12)


def test_expired_during_transit_blocks_suction_and_final_descent():
    fake = FakeHardware()
    calls = [0]

    def check(_):
        calls[0] += 1
        if calls[0] == 2:
            raise ValueError("Expired during transit")

    with pytest.raises(ValueError, match="Expired during transit"):
        execute(fake, check=check)
    assert ("DO", 13, True) not in fake.trace
    assert not any(v[0:2] == ("move", "p1_pick") for v in fake.trace)


def test_stop_feedback_drift_blocks_retract_even_with_idle_flags(monkeypatch):
    transport, feed, pose, _ = synthetic_transport(monkeypatch)
    feed["digital_input_bits"] = 1
    original = transport.node.feedback_snapshot

    def snapshot(**kw):
        if transport.clients["Stop"].call_async.call_count:
            feed["tool_vector_actual"][2] += .8  # <1 mm per tick, but not stationary overall.
        return original(**kw)

    transport.node.feedback_snapshot = snapshot
    with pytest.raises(ValueError, match="did not confirm stationary"):
        transport.move(home_plan(pose, pose, [.1]*6)[-1], stop_on_suction=True)
    assert transport.moving


def test_pose_service_must_have_one_canonical_provider():
    providers = [("item_teach", "/")]
    node = NS(get_node_names_and_namespaces=lambda: providers,
              get_service_names_and_types_by_node=lambda *_: [("/item_detect/get_item_poses", [])])
    controller.RobotController.check_detector_owner(node)
    providers.append(("item_detect", "/"))
    with pytest.raises(ValueError, match="exactly one canonical"):
        controller.RobotController.check_detector_owner(node)


def test_signal_shutdown_keeps_ros_alive_until_stop_and_restores_handlers(monkeypatch):
    handlers = {}
    previous = {controller.signal.SIGINT: "previous-int",
                controller.signal.SIGTERM: "previous-term"}
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setattr(controller.signal, "getsignal", lambda number: previous[number])
    monkeypatch.setattr(controller.signal, "signal", lambda number, callback: handlers.__setitem__(
        number, callback))
    alive, order = [False], []

    def initialize(**kwargs):
        assert kwargs["signal_handler_options"] == controller.SignalHandlerOptions.NO
        alive[0] = True

    def close():
        assert alive[0]
        order.append("stop-before-context-shutdown")

    node = NS(headless=True, fatal_error=None, debug=True, live=False, close_runtime=close,
              events=MagicMock(), destroy_node=MagicMock())
    executor = MagicMock()
    executor.spin_once.side_effect = lambda **_: handlers[controller.signal.SIGINT](2, None)
    monkeypatch.setattr(controller.rclpy, "init", initialize)
    monkeypatch.setattr(controller.rclpy, "ok", lambda: alive[0])
    monkeypatch.setattr(controller.rclpy, "shutdown", lambda: alive.__setitem__(0, False))
    monkeypatch.setattr(controller, "RobotController", lambda: node)
    monkeypatch.setattr(controller, "MultiThreadedExecutor", lambda **_: executor)
    controller.main()
    assert order == ["stop-before-context-shutdown"] and not alive[0]
    assert handlers == previous
