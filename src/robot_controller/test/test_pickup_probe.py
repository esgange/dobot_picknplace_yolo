"""Last-chance pickup lift retains acquisition through settling and queue admission."""

from concurrent.futures import Future
from dataclasses import replace
from types import SimpleNamespace
import threading

import numpy as np
import pytest

from robot_controller.controller import RobotController
from robot_controller.errors import FeedbackFailure, ManagedInterruption, OperationCanceled
from robot_controller.hardware import DobotTransport
from robot_controller.motion import MotionIO, PickExecutor, Target, pick_targets
from robot_controller.pick_session import PickSession
from test_motion_v2 import item_pose, matrix, settings, tray_target
from test_pick_acquisition import pickup_rig, VACUUM


def probe_rig(monkeypatch, *, retract_z=.14):
    rig, clock = pickup_rig(monkeypatch)
    rig.retract = np.eye(4)
    rig.retract[2, 3] = retract_z
    rig.settle = [dict(z=.1, outputs=VACUUM, running=1)] + [
        dict(z=.1, outputs=VACUUM)] * 8

    def dispatch(calls, **kwargs):
        replies = rig.dispatch(calls, **kwargs)
        for reply in replies:
            if hasattr(reply, "robot_return"):
                reply.robot_return = "{8}"
        return replies

    rig.transport.call_group = dispatch
    rig.during_admission = lambda: rig.emit(z=.1, outputs=VACUUM, running=1)
    return rig, clock


def pick_with_probe(rig):
    matrix = np.eye(4)
    matrix[2, 3] = .1
    return rig.run(targets=(Target("p1_pick", matrix, 10, 35,
                                  motion_io=(MotionIO(20, 13, True),)),),
                   stop_on_suction=True, pick_settling_sec=.3,
                   pickup_retract_pose=rig.retract, return_terminal_pose=True)


@pytest.mark.parametrize("at_endpoint", [False, True])
def test_di1_during_probe_takes_normal_acquisition_stop_without_waiting(monkeypatch, at_endpoint):
    rig, clock = probe_rig(monkeypatch)
    height = .120 if at_endpoint else .112
    rig.steps = iter([*rig.settle, dict(z=height, running=int(not at_endpoint),
                                      command_id=8, outputs=VACUUM, di1=True)])
    acquired, origin = pick_with_probe(rig)
    assert acquired and origin[2, 3] == pytest.approx(height)
    assert .4 <= clock[0] < .6
    assert len(rig.stops) == 1
    assert [name for name, _fields in rig.calls] == ["MovLIO", "MovL"]
    fields = rig.calls[-1][1]
    assert fields["c"] == pytest.approx(120.)  # 50% of 40 mm is 20 mm.
    assert fields["param_value"] == ["user=0", "tool=0", "v=10", "a=35"]
    assert "mdis" not in fields
    assert rig.transport.node.expected_outputs[13]
    assert not rig.transport.late_miss_suction
    assert rig.monitor._pickup_retract[0] == pytest.approx(.14)
    assert not any(a[1] == "motion_batch_completed" for a, _k in rig.events)


def test_no_di1_fails_only_after_probe_queue_id_idle_and_endpoint(monkeypatch):
    rig, clock = probe_rig(monkeypatch)
    rig.steps = iter([*rig.settle,
                      dict(z=.120, command_id=7, outputs=VACUUM),  # Old command.
                      dict(z=.1, command_id=8, outputs=VACUUM),  # Not at probe endpoint.
                      dict(z=.120, command_id=8, outputs=VACUUM, running=1),
                      dict(z=.120, command_id=8, outputs=VACUUM)])
    acquired, origin = pick_with_probe(rig)
    assert not acquired and origin[2, 3] == pytest.approx(.120)
    assert len(rig.calls) == 2 and not rig.stops
    assert clock[0] == pytest.approx(.65)  # No second settling interval.
    assert not rig.transport.acquisition_eligible and rig.transport.late_miss_suction
    assert rig.transport.node.expected_outputs[13]  # Release belongs to failed retract.
    completed = [k for a, k in rig.events if a[1] == "motion_batch_completed"]
    assert len(completed) == 1
    assert completed[0]["terminal_target"] == "p1_pick_pickup_probe"
    assert completed[0]["pickup_probe_attempted"]


def test_normal_pickup_does_not_add_probe(monkeypatch):
    rig, _clock = probe_rig(monkeypatch)
    rig.steps = iter([dict(z=.11, running=1, outputs=VACUUM, di1=True)])
    acquired, origin = pick_with_probe(rig)
    assert acquired and origin[2, 3] == .11
    assert len(rig.calls) == 1
    assert rig.monitor._pickup_retract[0] == pytest.approx(.14)
    assert not any(a[1] == "pickup_probe_started" for a, _k in rig.events)


def test_probe_preserves_actual_xy_and_attitude(monkeypatch):
    rig, _clock = probe_rig(monkeypatch)

    def actual(joints):
        pose = rig.forward(joints)
        angle = np.deg2rad(.2)
        pose[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
        pose[:2, 3] = [.002, -.001]
        return pose

    rig.transport.node.kinematics.forward = actual
    rig.steps = iter([*rig.settle, dict(z=.120, command_id=8, outputs=VACUUM)])
    acquired, origin = pick_with_probe(rig)
    fields = rig.calls[-1][1]
    assert not acquired
    assert [fields[axis] for axis in "abcf"] == pytest.approx([2., -1., 120., .2])
    assert np.allclose(origin[:2, 3], [.002, -.001])


@pytest.mark.parametrize("acquire", [False, True])
def test_executor_retains_active_ledger_until_probe_result_and_retracts_from_actual_pose(
        monkeypatch, acquire):
    rig, _clock = probe_rig(monkeypatch)
    taught = settings(use_grip=False, close_on_pick=False)
    taught["motion"]["prepick_height"] = 40.
    taught["timing"]["pick_settling"] = .3
    plan = list(pick_targets(matrix(.5), item_pose(x=0., y=0., z=.1), taught, 1,
                             rotation=np.eye(3)))
    # Omit entry finger I/O to isolate acquisition from presentation in this fixture.
    plan[0] = replace(plan[0], motion_io=())
    states, returns = [], []
    session = PickSession(["one"], [plan], changed=lambda _i, attempt: states.append(attempt.state))
    original_dispatch = rig.transport.call_group

    def dispatch(calls, **kwargs):
        replies = original_dispatch(calls, **kwargs)
        for index in range(len(replies)):
            kwargs["admitted"](index)
        return replies

    rig.transport.call_group = dispatch
    rig.transport.node.motion_admitted = session.admitted
    record = rig.transport.node.events.record

    def probe_event(*args, **kwargs):
        record(*args, **kwargs)
        if args[1] == "pickup_probe_started":
            assert states == ["ACTIVE"] and session.held_index is None

    rig.transport.node.events.record = probe_event
    move = rig.transport.move_batch

    def move_or_record(targets, **kwargs):
        if kwargs.get("stop_on_suction"):
            return move(targets, **kwargs)
        returns.append((targets, kwargs))

    rig.transport.move_batch = move_or_record
    rig.transport.sensor = lambda *_a: True
    relaxed = []

    def relax(channel, active, *, progress):
        assert states == ["ACTIVE"] and not active
        relaxed.append(channel)
        progress(rig.monitor.snapshot(require_enabled=True))

    rig.transport.output = relax
    height = .112 if acquire else .120
    rig.steps = iter([*rig.settle, dict(z=height, command_id=8, outputs=VACUUM,
                                      running=int(acquire), di1=acquire)])
    outcome = PickExecutor(rig.transport, finish_home=False).run(
        [plan], taught, tray_target=tray_target(), session=session, check=lambda _i: None,
        return_home=lambda **_k: pytest.fail("Unexpected Home during this pickup"))
    assert outcome["picked"] is acquire
    assert relaxed == [2, 14]
    assert states == ["ACTIVE", "HELD" if acquire else "FAILED"]
    assert len(returns) == 1
    targets, kwargs = returns[0]
    assert kwargs["confirmed_start_pose"][2, 3] == pytest.approx(height)
    assert targets[0].matrix[2, 3] == pytest.approx(.14)
    assert session.attempts[0].plan[2].matrix[2, 3] == pytest.approx(.14)
    assert targets[0].motion_io == (
        (MotionIO(50, 2, False), MotionIO(50, 14, False)) if acquire else
        (MotionIO(80, 13, False), MotionIO(80, 1, True)))


@pytest.mark.parametrize("outcome", ["pickup", "stop", "pause"])
def test_settling_to_probe_boundary_never_latches_miss_or_loses_acquisition(monkeypatch, outcome):
    rig, _clock = probe_rig(monkeypatch)
    record = rig.transport.node.events.record

    def boundary(*args, **kwargs):
        record(*args, **kwargs)
        if args[1] != "pickup_probe_started":
            return
        assert rig.transport.acquisition_eligible and not rig.transport.late_miss_suction
        if outcome == "pickup":
            rig.emit(z=.1, outputs=VACUUM, di1=True)
        else:
            def interrupt():
                raise (OperationCanceled("operator Stop") if outcome == "stop"
                       else ManagedInterruption("operator Pause"))
            rig.transport._wait_for_resume = interrupt

    rig.transport.node.events.record = boundary
    rig.steps = iter(rig.settle)
    if outcome == "pickup":
        acquired, origin = pick_with_probe(rig)
        assert acquired and origin[2, 3] == .1
    else:
        with pytest.raises(OperationCanceled if outcome == "stop" else ManagedInterruption):
            pick_with_probe(rig)
    assert len(rig.calls) == 1
    assert not any(a[1] == "motion_batch_completed" for a, _k in rig.events)
    assert not rig.transport.moving and rig.monitor._motion is None


@pytest.mark.parametrize("outputs", [0, VACUUM | (1 << 13), VACUUM | (1 << 1)])
def test_probe_rejects_suction_loss_or_uncommanded_finger_change(monkeypatch, outputs):
    rig, _clock = probe_rig(monkeypatch)
    rig.steps = iter([*rig.settle, dict(z=.112, running=1, command_id=8, outputs=outputs)])
    with pytest.raises(FeedbackFailure, match="Pickup probe output DO"):
        pick_with_probe(rig)
    assert len(rig.calls) == 2
    assert not rig.transport.late_miss_suction


@pytest.mark.parametrize("retract_z", [.1, .09])
def test_no_upward_travel_skips_probe_instead_of_inventing_height_or_descending(
        monkeypatch, retract_z):
    rig, _clock = probe_rig(monkeypatch, retract_z=retract_z)
    rig.steps = iter(rig.settle)
    acquired, origin = pick_with_probe(rig)
    assert not acquired and origin[2, 3] == .1
    assert len(rig.calls) == 1
    started = [k for a, k in rig.events if a[1] == "pickup_probe_started"]
    assert started[0]["lift_mm"] == 0.


def test_di1_during_probe_admission_resolves_reply_and_discards_queue_again(monkeypatch):
    rig, _clock = probe_rig(monkeypatch)
    transport = rig.transport
    pending = Future()
    transport.response_lock = threading.RLock()
    transport.pending_response = transport.pending_group = None
    transport.node.check_all_command_owners = lambda _names: None
    transport._begin_service_audit = lambda name, fields: {
        "name": name, "fields": fields, "started": 0.}
    transport._finish_service_audit = lambda audit, outcome, **_k: (
        audit.update(outcome=outcome) if audit is not None else None)

    def dispatch(name, fields):
        rig.calls.append((name, fields))
        if name == "MovL":
            rig.emit(z=.112, running=1, outputs=VACUUM, di1=True)
            return pending
        rig.emit(z=.1, running=1, outputs=VACUUM)
        future = Future()
        future.set_result(SimpleNamespace(res=0))
        return future

    transport.clients = {
        name: SimpleNamespace(service_is_ready=lambda: True,
                              call_async=lambda fields, name=name: dispatch(name, fields))
        for name in ("MovL", "MovLIO")}
    transport.types = {name: SimpleNamespace(Request=lambda **fields: fields)
                       for name in transport.clients}
    transport.call_group = lambda calls, **kwargs: DobotTransport.call_group(
        transport, calls, **kwargs)

    def respond(_seconds):
        assert rig.stops and not pending.done()
        pending.set_result(SimpleNamespace(res=0, robot_return="{8}"))

    transport.node.wait_control = respond
    rig.steps = iter(rig.settle)
    acquired, origin = pick_with_probe(rig)
    assert acquired and origin[2, 3] == .112
    assert pending.done() and len(rig.calls) == 2
    assert [fresh for _reason, fresh, _future in rig.stops] == [False, True]


@pytest.mark.parametrize("settling_ms", [300, 100, 250.5])
def test_failure_log_explains_both_attempts_but_keeps_typed_state(settling_ms):
    events, messages = [], []
    node = SimpleNamespace(
        configuration=SimpleNamespace(profile={"timing": {"pick_settling": settling_ms / 1000}}),
        events=SimpleNamespace(record=lambda *a, **k: events.append((a, k))),
        publish_operator_log=lambda _level, text: messages.append(text),
        publish_status=lambda: None)
    RobotController._attempt_changed(node, 1, SimpleNamespace(state="FAILED", identifier="item"))
    expected = (f"FAILED — no DI1 pickup detected after {settling_ms:g} ms settling "
                "and the 50% upward-lift check.")
    assert messages == [f"Candidate 1 (item): {expected}"]
    assert events[0][0][2] == expected
    assert events[0][1]["candidate_state"] == "FAILED"
