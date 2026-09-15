"""Queued response/motion/I/O tests using synthetic clocks and transport only."""

import threading
from dataclasses import replace
from types import SimpleNamespace as NS
from unittest.mock import MagicMock

import numpy as np
import pytest
from rclpy.task import Future

from robot_controller import controller, hardware
from robot_controller.kinematics import pose_matrix, pose_values
from robot_controller.motion import MotionIO, pick_targets
from test_execution import FakeHardware, execute, plan, settings, synthetic_transport


def queue_transport(monkeypatch, count, *, acquisition=False, unanswered=None, rejected=None,
                    ignore_events=False, stop_rejected=False):
    transport, feed, _, clock = synthetic_transport(monkeypatch)
    # This deliberately fake analytic model maps six service values to a pose.
    # Canonical CR10 FK/model validation is independently tested in test_execution.
    home = pose_matrix([100, 200, 800, 180, 0, 0])
    feed["tool_vector_actual"] = pose_values(home)
    transport.node.kinematics = NS(forward=lambda joints: pose_matrix(np.rad2deg(joints)))
    transport.node.current_joints = lambda: tuple(np.deg2rad(feed["tool_vector_actual"]))
    state = NS(sent=[], pending=[], remaining=[], applied=[], stopped=False, executed=0)

    def reply(values):
        future = Future()
        future.set_result(NS(res=0, robot_return="{" + ",".join(map(str, values)) + "}"))
        return future

    transport.clients["GetPose"].call_async.side_effect = lambda _: reply(
        feed["tool_vector_actual"])

    def send(request, name):
        assert all(future.done() for future, _, _ in state.pending), "Responses overlapped"
        if name == "MovLIO":
            assert request.mdis, "Firmware rejects MovLIO without a timed I/O tuple"
        elif name == "MovL":
            assert not hasattr(request, "mdis")
        state.sent.append((name, request, clock[0]))
        index = len(state.sent)
        future = Future()
        state.pending.append((future, clock[0] + .06, index))
        state.remaining.append((name, request))
        feed.update(isRunQueuedCmd=1, RunningStatus=1, robot_mode=7)
        return future

    for name in ("MovL", "MovLIO", "RelMovLUser"):
        transport.clients[name].call_async.side_effect = lambda request, n=name: send(request, n)

    def stop(_):
        state.stopped = True
        state.remaining.clear()
        feed.update(isRunQueuedCmd=0, RunningStatus=0, robot_mode=5)
        future = Future()
        future.set_result(NS(res=int(stop_rejected)))
        return future

    transport.clients["Stop"].call_async.side_effect = stop

    def tick(dt):
        clock[0] += dt
        for future, deadline, index in state.pending:
            if not future.done() and index != unanswered and clock[0] >= deadline:
                future.set_result(NS(res=int(index == rejected)))
        # Acceptance is not completion: execute only AFTER the whole batch was submitted.
        if len(state.sent) < count or not all(f.done() for f, _, _ in state.pending):
            return
        if not state.remaining:
            return
        name, request = state.remaining.pop(0)
        state.executed += 1
        if name == "RelMovLUser":
            feed["tool_vector_actual"][2] += request.c
        else:
            goal = [getattr(request, key) for key in "abcdef"]
            if request.mode:
                goal = list(goal)  # Fake analytic FK maps joint degrees to pose values.
            events = request.mdis if name == "MovLIO" else ()
            for raw in events:
                mode, distance, channel, active = map(int, raw.strip("{}").split(","))
                if not ignore_events:
                    mask = 1 << (channel-1)
                    feed["digital_outputs"] = ((feed["digital_outputs"] | mask) if active
                                               else (feed["digital_outputs"] & ~mask))
                state.applied.append((state.executed, mode, distance, channel, active))
            if acquisition and events == ["{1,0,13,1}"]:
                feed["digital_input_bits"] = 1
                # Seal before nominal pick: keep a higher actual stopped Z.
                goal[2] += 25
            feed["tool_vector_actual"] = goal
        if not state.remaining:
            feed.update(isRunQueuedCmd=0, RunningStatus=0, robot_mode=5)

    monkeypatch.setattr(hardware.time, "sleep", tick)
    return transport, feed, home, clock, state


def test_forward_queue_ack_order_without_waypoint_arrival_waits(monkeypatch):
    transport, feed, _, clock, state = queue_transport(monkeypatch, 4)
    checks = MagicMock()
    acquired = transport.move_batch(plan()[:4], stop_on_suction=True, before_suction=checks)
    assert not acquired and not transport.moving
    assert len(state.sent) == state.executed == 4
    assert state.sent[-1][2] < .3  # Arrival/stationarity waits occur only after submission.
    assert clock[0] >= .3
    np.testing.assert_allclose(feed["tool_vector_actual"][:3], [300, 400, 190])
    assert [name for name, _, _ in state.sent] == ["MovL", "MovLIO", "MovL", "MovLIO"]
    requests = [request for _, request, _ in state.sent]
    assert [request.mdis for name, request, _ in state.sent if name == "MovLIO"] == [
        ["{0,50,2,0}", "{0,50,14,1}"], ["{1,0,13,1}"]]
    assert all(not hasattr(request, "mdis") for name, request, _ in state.sent
               if name == "MovL")
    assert [request.param_value for request in requests] == [
        ["user=0", "tool=0", f"v={v}", "a=100", "cp=0"] for v in (100, 100, 100, 6)]
    assert checks.call_count > 4


@pytest.mark.parametrize("failed", [1, 2, 3, 4])
@pytest.mark.parametrize("kind", ["timeout", "rejected"])
def test_failed_queue_ack_never_submits_next_waypoint(failed, kind, monkeypatch):
    options = {"unanswered" if kind == "timeout" else "rejected": failed}
    transport, _, _, _, state = queue_transport(monkeypatch, 4, **options)
    with pytest.raises(ValueError, match="MovL.*(timeout|failed)"):
        transport.move_batch(plan()[:4], stop_on_suction=True)
    assert len(state.sent) == failed
    assert transport.moving  # Owning action must Stop the ambiguous/remaining queue.


def test_all_targets_validated_before_any_motion_batch_is_submitted(monkeypatch):
    transport, _, _, _, state = queue_transport(monkeypatch, 4)
    targets = list(plan()[:4])
    targets[2] = replace(targets[2], matrix=np.full((4, 4), np.nan))
    with pytest.raises(ValueError, match="Invalid rigid target transform"):
        transport.move_batch(targets, stop_on_suction=True)
    assert "InverseKin" not in transport.clients
    assert not state.sent and not transport.moving


def test_early_acquisition_stops_queue_and_confirms_actual_stopped_pose(monkeypatch):
    transport, feed, _, clock, state = queue_transport(monkeypatch, 4, acquisition=True)
    assert transport.move_batch(plan()[:4], stop_on_suction=True)
    assert state.stopped and not transport.moving and clock[0] >= .3
    assert transport.current_pose()[2, 3] == pytest.approx(.215)
    assert feed["digital_input_bits"] == 1


def test_early_stop_rejection_blocks_retract(monkeypatch):
    transport, _, _, _, _ = queue_transport(
        monkeypatch, 4, acquisition=True, stop_rejected=True)
    with pytest.raises(ValueError, match="DI1 Stop rejected"):
        transport.move_batch(plan()[:4], stop_on_suction=True)
    assert transport.moving


def test_missing_hardware_event_feedback_does_not_report_batch_success(monkeypatch):
    transport, _, _, _, _ = queue_transport(monkeypatch, 4, ignore_events=True)
    with pytest.raises(ValueError, match="motion I/O feedback confirmation timeout"):
        transport.move_batch(plan()[:4], stop_on_suction=True)
    assert transport.moving


def test_return_is_one_queue_including_conditional_home_rise_and_joint_home(monkeypatch):
    transport, feed, home, _, state = queue_transport(monkeypatch, 4)
    feed["tool_vector_actual"] = pose_values(plan()[3].matrix)
    feed["digital_input_bits"] = 1
    cfg = settings()
    cfg["home"] = {"positions_rad": list(np.deg2rad(pose_values(home)))}
    upward = list(plan()[4:])
    upward[0] = replace(upward[0], motion_io=(MotionIO(100, 14, False), MotionIO(100, 2, True)))
    node = NS(check_cancelled=MagicMock(), kinematics=transport.node.kinematics,
              debug=False, hardware=transport, home_reference=home,
              home_reference_joints=tuple(cfg["home"]["positions_rad"]))
    node._loaded_home_reference = controller.RobotController._loaded_home_reference.__get__(node)
    targets = controller.RobotController.home(node, cfg, preceding=upward, require_suction=True)
    assert [name for name, _, _ in state.sent] == [
        "MovLIO", "MovL", "RelMovLUser", "MovL"]
    assert [request.param_value[2] for _, request, _ in state.sent] == [
        "v=6", "v=100", "v=100", "v=100"]
    assert state.sent[0][1].mdis == ["{0,100,14,0}", "{0,100,2,1}"]
    assert state.sent[2][1].c == pytest.approx(460.)
    assert state.sent[-1][1].mode and targets[-1].joints_rad is not None
    np.testing.assert_allclose(feed["tool_vector_actual"], pose_values(home))


@pytest.mark.parametrize("close_onpick", [False, True])
def test_finger_closing_requires_success_and_obeys_selected_timing(close_onpick):
    cfg = settings()
    cfg["gripper"]["grip_onpick"] = close_onpick
    success = FakeHardware()
    execute(success, cfg)
    close_at = next(i for i, entry in enumerate(success.trace)
                    if entry == ("DO", 2, True))
    pick_at = next(i for i, entry in enumerate(success.trace) if entry[:2] == ("move", "p1_pick"))
    retract_at = next(i for i, entry in enumerate(success.trace)
                      if entry[:2] == ("move", "p1_retract"))
    assert (pick_at < close_at < retract_at) if close_onpick else (retract_at < close_at)
    missed = FakeHardware((False, False))
    execute(missed, cfg)
    assert ("DO", 2, True) not in missed.trace
    assert ("DO", 14, False) not in missed.trace


@pytest.mark.parametrize("grip", [False, True])
def test_canonical_height_equations_and_event_assignment(grip):
    cfg = settings()
    cfg["gripper"]["use_grip"] = grip
    targets = pick_targets(pose_matrix([0, 0, 800, 180, 0, 0]), [.3, .4, .1], cfg, 1)
    assert "zheight_offset" not in cfg["motion"]
    np.testing.assert_allclose([target.matrix[2, 3] for target in targets],
                               [.8, .34, .24, .19, .24, .34])
    assert targets[1].motion_io == (
        (MotionIO(50, 2, False), MotionIO(50, 14, True)) if grip else ())
    assert targets[3].motion_io == (MotionIO(0, 13, True),)


@pytest.mark.parametrize("values", [(True, 13, True), (-1, 13, True), (101, 13, True),
                                    (50, 3, True), (50, 13, 1)])
def test_motion_events_require_strict_canonical_values(values):
    with pytest.raises(ValueError):
        MotionIO(*values)


def test_expiry_before_queued_suction_start_stops_advancement(monkeypatch):
    transport, feed, _, clock, state = queue_transport(monkeypatch, 4)

    def check():
        if clock[0] > .25:
            raise ValueError("Candidate expired in queued transit")

    with pytest.raises(ValueError, match="Candidate expired"):
        transport.move_batch(plan()[:4], stop_on_suction=True, before_suction=check)
    assert not feed["digital_outputs"] & (1 << 12)
    assert transport.moving and len(state.sent) == 4


@pytest.mark.parametrize("policy", ["require_suction", "forbid_suction"])
def test_suction_policy_violation_blocks_return_queue(policy, monkeypatch):
    transport, feed, _, _, state = queue_transport(monkeypatch, 2)
    feed["digital_input_bits"] = int(policy == "forbid_suction")
    with pytest.raises(ValueError, match="Suction lost|Late DI1"):
        transport.move_batch(plan()[4:], **{policy: True})
    assert not state.sent


def test_late_di1_during_missed_return_blocks_next_candidate(monkeypatch):
    transport, feed, _, _, state = queue_transport(monkeypatch, 2)
    tick = hardware.time.sleep

    def activate(dt):
        tick(dt)
        if len(state.sent) == 2:
            feed["digital_input_bits"] = 1

    monkeypatch.setattr(hardware.time, "sleep", activate)
    with pytest.raises(ValueError, match="Late DI1"):
        transport.move_batch(plan()[4:], forbid_suction=True)
    assert transport.moving


def test_unexpected_di1_without_suction_never_submits_forward_queue(monkeypatch):
    transport, feed, _, _, state = queue_transport(monkeypatch, 4)
    feed["digital_input_bits"] = 1
    with pytest.raises(ValueError, match="Unexpected DI1 before queued suction"):
        transport.move_batch(plan()[:4], stop_on_suction=True)
    assert not state.sent


@pytest.mark.parametrize("mode", [7, 8])
@pytest.mark.parametrize("enabled", [0, 1])
def test_moving_vendor_status_is_not_idle_but_requires_explicit_enable_flag(mode, enabled):
    now = hardware.time.monotonic()
    feed = {"robot_mode": mode, "EnableStatus": enabled, "ErrorStatus": 0,
            "CollisionStates": 0, "isPauseCmdFlag": 0, "userCoordinate": 0,
            "toolCoordinate": 0}
    node = NS(state_lock=threading.RLock(), _sole_publisher=MagicMock(),
              current_joints=MagicMock(),
              robot_feedback=(True, False, now), controller_progress_at=now,
              feed_feedback=(feed, now), feed_sequence=3)
    if not enabled:
        with pytest.raises(ValueError, match="EnableStatus=0"):
            controller.RobotController.feedback_snapshot(node, enabled=True)
    else:
        assert controller.RobotController.feedback_snapshot(node, enabled=True)["feed"] == feed


def test_idle_vendor_disabled_status_still_blocks_even_with_enable_flag():
    now = hardware.time.monotonic()
    feed = {"robot_mode": 5, "EnableStatus": 1, "ErrorStatus": 0, "CollisionStates": 0,
            "isPauseCmdFlag": 0, "userCoordinate": 0, "toolCoordinate": 0}
    node = NS(state_lock=threading.RLock(), _sole_publisher=MagicMock(),
              current_joints=MagicMock(),
              robot_feedback=(True, False, now), controller_progress_at=now,
              feed_feedback=(feed, now), feed_sequence=3)
    with pytest.raises(ValueError, match="RobotStatus.is_enable=False"):
        controller.RobotController.feedback_snapshot(node, enabled=True)


@pytest.mark.parametrize("stopped_z,expected", [
    (.19, [.24, .34]), (.25, [.25, .34]), (.35, [.35, .35]),
])
def test_executor_retracts_to_nominal_heights_clamped_upward_from_actual_stop(stopped_z, expected):
    fake = FakeHardware(stopped_z=stopped_z)
    recorded = []
    execute(fake, remember_prepick=lambda target, _: recorded.append(target))
    upward = [entry for entry in fake.trace if entry[0] == "move"][-2:]
    np.testing.assert_allclose([entry[3] for entry in upward], expected)
    if stopped_z > .24:
        assert recorded[-1].matrix[2, 3] == stopped_z
        assert recorded[-1].name == "p1_prepick"


def test_retract_preserves_actual_stopped_xy_and_attitude():
    fake = FakeHardware()
    stopped = pose_matrix([300.5, 399.5, 190, 179.9, 0, 0])
    fake.current_pose = lambda: stopped.copy()
    captured = []
    original = fake.move_batch

    def capture(targets, **kw):
        if not kw.get("stop_on_suction"):
            captured.extend(targets)
        return original(targets, **kw)

    fake.move_batch = capture
    execute(fake)
    assert len(captured) == 2
    for target in captured:
        np.testing.assert_allclose(target.matrix[:2, 3], stopped[:2, 3])
        np.testing.assert_allclose(target.matrix[:3, :3], stopped[:3, :3])


def test_di1_during_pending_motion_ack_re_stops_late_acceptance_before_retract(monkeypatch):
    transport, feed, _, _, state = queue_transport(monkeypatch, 4)
    original = hardware.time.sleep

    def seal_during_ack(dt):
        original(dt)
        if len(state.sent) == 4:
            feed["digital_outputs"] |= (1 << 12) | (1 << 13)
            feed["digital_input_bits"] = 1

    monkeypatch.setattr(hardware.time, "sleep", seal_during_ack)
    assert transport.move_batch(plan()[:4], stop_on_suction=True)
    assert transport.clients["Stop"].call_async.call_count == 2
    assert not transport.moving and not state.remaining


def test_di1_already_on_blocks_vacuum_off_and_new_pick():
    fake = FakeHardware()
    fake.sensor = lambda *_args, **_kw: False
    with pytest.raises(ValueError, match="DI1 failed to clear"):
        execute(fake)
    assert not fake.trace  # No exhaust, vacuum, finger or motion command.


def test_late_di1_after_home_before_vacuum_off_is_not_released_or_retried(monkeypatch):
    transport, feed, _, _, _ = queue_transport(monkeypatch, 2)
    feed["digital_input_bits"] = 1
    feed["digital_outputs"] = 1 << 12
    with pytest.raises(ValueError, match="late DI1 before vacuum OFF"):
        transport.output(13, False, require_clear=True)
    transport.clients["DO"].call_async.assert_not_called()
    assert feed["digital_outputs"] & (1 << 12)
