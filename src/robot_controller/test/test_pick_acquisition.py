"""Pickup switches queues on Stop acceptance, even with moving feedback."""

from concurrent.futures import Future
from types import SimpleNamespace
import threading

import numpy as np
import pytest

import robot_controller.hardware as hardware_module
from robot_controller.errors import OperationCanceled, StopUnconfirmed
from robot_controller.hardware import DobotTransport
from robot_controller.motion import MotionIO, Target
from test_motion_completion import MotionRig


VACUUM = 1 << 12


def pickup_rig(monkeypatch):
    rig = MotionRig()
    clock = [0.]
    monkeypatch.setattr(hardware_module, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    original_next = rig.next_sample

    def next_sample(*args, **kwargs):
        clock[0] += .05
        return original_next(*args, **kwargs)

    rig.monitor.wait_next = next_sample
    rig.transport.confirm_stop = lambda *_a, **_k: pytest.fail("Pickup waited for standstill")
    rig.monitor.wait = lambda *_a, **_k: pytest.fail("Pickup used a stationary/output wait")
    rig.transport.node.wait_control = lambda seconds: clock.__setitem__(0, clock[0] + seconds)
    rig.stops = []

    def stop(reason, *, fresh=False):
        future = Future()
        future.set_result(SimpleNamespace(res=0))
        rig.stops.append((reason, fresh, future))
        return future

    rig.transport.request_stop = stop
    return rig, clock


def pick(rig, settling=1.5):
    matrix = np.eye(4)
    matrix[2, 3] = .1
    target = Target("pick", matrix, 3, 100, motion_io=(MotionIO(20, 13, True),))
    return rig.run(targets=(target,),
                   stop_on_suction=True, pick_settling_sec=settling,
                   return_terminal_pose=True)


@pytest.mark.parametrize("during_settling", [False, True])
def test_di1_bypasses_remaining_descent_or_settling_and_never_waits_for_idle(
        monkeypatch, during_settling):
    rig, clock = pickup_rig(monkeypatch)
    z = .1 if during_settling else .15
    samples = [dict(z=z, running=1, outputs=VACUUM)]
    if during_settling:
        samples.append(dict(z=z, outputs=VACUUM))
    samples.append(dict(z=z, running=int(not during_settling), outputs=VACUUM, di1=True))
    rig.steps = iter(samples)
    acquired, origin = pick(rig)
    assert acquired and origin[2, 3] == z
    assert clock[0] < 1.5
    assert len(rig.stops) == 1
    assert rig.transport.node.expected_outputs[13]
    assert not rig.transport.pending_motion_outputs
    assert not any(a[1] == "motion_batch_completed" for a, _ in rig.events)

    # The return accepts the measured origin even if status still reports motion.
    rig.during_admission = lambda: rig.emit(z=z, running=1, outputs=VACUUM, di1=True)
    rig.steps = iter([dict(z=.2, command_id=7, outputs=VACUUM, di1=True)])
    rig.run(z=.2, require_suction=True, confirmed_start_pose=origin)
    assert len(rig.calls) == 2
    assert len(rig.stops) == 1


def test_pickup_waits_for_stop_reply_but_accepts_still_moving_feedback(monkeypatch):
    rig, _clock = pickup_rig(monkeypatch)
    pending = Future()
    rig.transport.request_stop = lambda _reason: pending
    rig.steps = iter([dict(z=.15, running=1, outputs=VACUUM, di1=True)])
    waits = []

    def respond(seconds):
        waits.append(seconds)
        rig.emit(z=.14, running=1, outputs=VACUUM, di1=True)
        pending.set_result(SimpleNamespace(res=0))

    rig.transport.node.wait_control = respond
    acquired, origin = pick(rig)
    assert acquired and origin[2, 3] == .14
    assert waits == [.02]
    assert rig.monitor.snapshot().feed["RunningStatus"] == 1


@pytest.mark.parametrize("failure", ["rejected", "exception", "timeout", "cancel"])
def test_failed_or_canceled_pickup_stop_cannot_start_return(monkeypatch, failure):
    rig, _clock = pickup_rig(monkeypatch)
    pending = Future()
    if failure == "rejected":
        pending.set_result(SimpleNamespace(res=-1))
    elif failure == "exception":
        pending.set_exception(RuntimeError("lost response"))
    rig.transport.request_stop = lambda _reason: pending
    rig.steps = iter([dict(z=.15, running=1, outputs=VACUUM, di1=True)])
    if failure == "cancel":
        def check():
            if rig.transport.suction_interrupted:
                raise OperationCanceled("operator Stop")
        rig.transport.node.raise_if_cancelled = check
    with pytest.raises(OperationCanceled if failure == "cancel" else StopUnconfirmed):
        pick(rig)
    assert len(rig.calls) == 1
    assert not any(a[1] == "pickup_stop_acknowledged" for a, _ in rig.events)


@pytest.mark.parametrize("response_delayed", [False, True])
def test_pickup_waits_for_motion_reply_then_stops_without_admitting_next_command(
        monkeypatch, response_delayed):
    rig, _clock = pickup_rig(monkeypatch)
    transport = rig.transport
    pending = Future()
    order = []
    transport.response_lock = threading.RLock()
    transport.pending_response = None
    transport.pending_group = None
    transport.node.check_all_command_owners = lambda _names: None
    transport._begin_service_audit = lambda name, fields: {
        "name": name, "fields": fields, "started": 0.}
    transport._finish_service_audit = lambda audit, outcome, **_k: (
        audit.update(outcome=outcome) if audit is not None else None)

    def dispatch(_request):
        order.append("motion sent")
        rig.emit(z=.15, running=1, outputs=VACUUM, di1=True)
        if not response_delayed:
            order.append("motion accepted")
            pending.set_result(SimpleNamespace(res=0))
        return pending

    transport.clients = {"MovLIO": SimpleNamespace(
        service_is_ready=lambda: True, call_async=dispatch)}
    transport.types = {"MovLIO": SimpleNamespace(Request=lambda **fields: fields)}
    transport.call_group = lambda calls, **kwargs: DobotTransport.call_group(
        transport, calls, **kwargs)

    def stop(_reason, *, fresh=False):
        assert pending.done() and not fresh
        order.append("pickup Stop")
        future = Future()
        future.set_result(SimpleNamespace(res=0))
        return future

    def respond(_seconds):
        assert order == ["motion sent"] and transport.suction_interrupted
        order.append("motion accepted")
        pending.set_result(SimpleNamespace(res=0))

    transport.request_stop = stop
    transport.node.wait_control = respond
    matrix = np.eye(4)
    matrix[2, 3] = .1
    target = Target("pick", matrix, 3, 100, motion_io=(MotionIO(20, 13, True),))
    acquired, _origin = rig.run(
        targets=(target, target), stop_on_suction=True, return_terminal_pose=True)
    assert acquired
    assert order == ["motion sent", "motion accepted", "pickup Stop"]
    # A late-scheduled normal callback must not Stop the newly queued return.
    transport._completed_response("MovLIO", pending, {"outcome": "accepted"})
    assert order[-1] == "pickup Stop"
    assert len(order) == 3
