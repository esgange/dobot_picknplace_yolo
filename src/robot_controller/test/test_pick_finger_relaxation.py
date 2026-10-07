"""Real pickup transport and DO feedback, using synthetic services only."""

from concurrent.futures import Future
from types import SimpleNamespace

import numpy as np
import pytest

from robot_controller.errors import (
    CommandRejected, CommandResponseTimeout, FeedbackFailure, ManagedInterruption,
    OperationCanceled)
from robot_controller.hardware import DobotTransport
from robot_controller.feedback import FeedbackMonitor
from robot_controller.motion import (
    PickExecutor, Target, gripper_open_events, pick_targets, vacuum_suck_events)
from test_motion_v2 import FakeHardware, item_pose, matrix, settings, tray_target
from test_pick_acquisition import pickup_rig, VACUUM


OPEN = VACUUM | (1 << 13)


def relaxation_rig(monkeypatch):
    rig, clock = pickup_rig(monkeypatch)
    node = rig.transport.node
    node.check_command_owner = lambda _name: None
    rig.outputs = []

    def echo(request, *, di1=None, outputs=None):
        sample = rig.monitor.snapshot()
        bits = sample.feed["digital_outputs"] & ~(1 << (request.index - 1))
        rig.emit(z=sample.joints[0], running=sample.feed["RunningStatus"],
                 command_id=sample.feed["currentCommandId"],
                 outputs=bits if outputs is None else outputs,
                 di1=bool(sample.feed["digital_input_bits"] & 1) if di1 is None else di1)

    rig.echo = echo
    rig.on_output = echo

    def client(_kind, endpoint):
        def call(request):
            assert endpoint.endswith("/DO")
            rig.outputs.append((request.index, request.status, request.time))
            result = rig.on_output(request)
            if isinstance(result, Future):
                return result
            future = Future()
            future.set_result(SimpleNamespace(res=0))
            return future
        return SimpleNamespace(service_is_ready=lambda: True, call_async=call)

    node.create_client = client
    DobotTransport.__init__(rig.transport, node, rig.monitor)
    rig.emit(z=.2, outputs=OPEN)

    def admission():
        sample = rig.monitor.snapshot()
        rig.emit(z=sample.joints[0], outputs=sample.feed["digital_outputs"], running=1)

    rig.during_admission = admission
    return rig, clock


def pick(rig, *, relax=True, settling=.3):
    target = Target("pick", matrix(.1), 10, 35,
                    motion_io=gripper_open_events(0) + vacuum_suck_events(20))
    return rig.run(targets=(target,), stop_on_suction=True, relax_pick_fingers=relax,
                   pick_settling_sec=settling, pickup_retract_pose=matrix(.14),
                   return_terminal_pose=True)


def test_relax_at_arrival_before_settling_and_preserve_through_failed_probe(monkeypatch):
    rig, clock = relaxation_rig(monkeypatch)
    rig.steps = iter([dict(z=.14, outputs=OPEN, running=1),
                      dict(z=.1, outputs=OPEN),
                      *[dict(z=.1, outputs=VACUUM)] * 8,
                      dict(z=.12, outputs=VACUUM, command_id=7)])
    positions = []

    def echo(request):
        positions.append((rig.monitor.snapshot().joints[0], clock[0]))
        rig.echo(request)

    rig.on_output = echo
    acquired, origin = pick(rig)
    assert not acquired and origin[2, 3] == pytest.approx(.12)
    assert rig.outputs == [(2, 0, 0), (14, 0, 0)]
    assert positions == [(.1, .1), (.1, .1)]  # Arrival; before .3 s settling expires.
    assert [name for name, _ in rig.calls] == ["MovLIO", "MovL"]
    assert rig.transport.node.expected_outputs == {1: False, 2: False, 13: True, 14: False}
    assert not rig.stops and rig.transport.late_miss_suction


@pytest.mark.parametrize("phase", ["descent", "settling", "probe"])
def test_pickup_relaxes_once_and_returns_actual_pose_without_extra_standstill(monkeypatch, phase):
    rig, _clock = relaxation_rig(monkeypatch)
    before = [] if phase == "descent" else [dict(z=.1, outputs=OPEN)]
    if phase == "probe":
        before += [dict(z=.1, outputs=VACUUM)] * 8
    height = .15 if phase == "descent" else .112 if phase == "probe" else .1
    rig.steps = iter([*before, dict(z=height, outputs=OPEN if phase == "descent" else VACUUM,
                                  running=1, di1=True)])
    acquired, origin = pick(rig)
    assert acquired and origin[2, 3] == pytest.approx(height)
    assert rig.outputs == [(2, 0, 0), (14, 0, 0)]
    assert len(rig.stops) == 1
    assert rig.transport.node.expected_outputs[14] is False
    assert not rig.transport.pending_motion_outputs
    assert not any(a[1] == "motion_batch_completed" for a, _ in rig.events)


@pytest.mark.parametrize("pending", ["reply", "output"])
def test_di1_during_relaxation_wait_requests_stop_and_keeps_acquisition(monkeypatch, pending):
    rig, clock = relaxation_rig(monkeypatch)
    future = Future()

    def output(request):
        if request.index != 2:
            return rig.echo(request)
        if pending == "output":
            return  # Accepted, but no fresh output sample yet.
        rig.echo(request, di1=True)

        def respond(_seconds):
            assert len(rig.stops) == 1  # Stop sent even before the DO response.
            clock[0] += .02
            future.set_result(SimpleNamespace(res=0))
        rig.transport.node.wait_control = respond
        return future

    rig.on_output = output
    rig.steps = iter([dict(z=.1, outputs=OPEN), dict(z=.1, outputs=OPEN, di1=True)])
    acquired, _origin = pick(rig)
    assert acquired and len(rig.stops) == 1
    assert rig.outputs == [(2, 0, 0), (14, 0, 0)]
    assert rig.monitor.snapshot().feed["digital_outputs"] == VACUUM


def test_single_new_feed_packet_releases_output_wait_with_independent_joint_updates(monkeypatch):
    rig, _clock = relaxation_rig(monkeypatch)
    scripted_next = rig.monitor.wait_next

    def wait_next(sequence, timeout, *, cancel=None, position=False):
        scripted_next(sequence, timeout, cancel=cancel, position=position)
        # Exercise the real counter gate with exactly one new feed packet. Joint
        # and status revisions already outnumber feed sequences in this rig.
        return FeedbackMonitor.wait_next(
            rig.monitor, sequence, .001, cancel=cancel, position=position)

    rig.monitor.wait_next = wait_next
    rig.on_output = lambda request: rig.echo(request) if request.index == 14 else None
    rig.steps = iter([dict(z=.1, outputs=OPEN), dict(z=.1, outputs=OPEN, di1=True)])
    acquired, _origin = pick(rig)
    assert acquired and rig.outputs == [(2, 0, 0), (14, 0, 0)]


@pytest.mark.parametrize("failure", ["stop", "reject", "vacuum", "finger"])
def test_relaxation_failure_blocks_probe_or_return(monkeypatch, failure):
    rig, _clock = relaxation_rig(monkeypatch)

    def output(request):
        if failure == "stop":
            def canceled():
                raise OperationCanceled("operator Stop")
            rig.transport.node.raise_if_cancelled = canceled
            rig.transport.node.cancel_requested = lambda: True
            return
        if failure == "reject":
            future = Future()
            future.set_result(SimpleNamespace(res=-1))
            return future
        if failure == "vacuum":
            return rig.echo(request, outputs=1 << 13)
        rig.echo(request, outputs=OPEN if request.index == 2 else VACUUM | 2)

    rig.on_output = output
    rig.steps = iter([dict(z=.1, outputs=OPEN)])
    expected = (OperationCanceled if failure == "stop" else
                CommandRejected if failure == "reject" else FeedbackFailure)
    with pytest.raises(expected):
        pick(rig)
    assert len(rig.calls) == 1
    assert len(rig.outputs) == (2 if failure == "finger" else 1)
    assert not any(a[1] == "pickup_fingers_relaxed" for a, _ in rig.events)


@pytest.mark.parametrize("channel", [2, 14])
def test_pause_during_either_output_prevents_probe_or_return(monkeypatch, channel):
    rig, _clock = relaxation_rig(monkeypatch)
    rig.transport._ready_snapshot = lambda: DobotTransport._ready_snapshot(rig.transport)

    def output(request):
        rig.echo(request)
        if request.index == channel:
            def paused():
                raise ManagedInterruption("operator Pause")
            rig.transport._wait_for_resume = paused

    rig.on_output = output
    rig.steps = iter([dict(z=.1, outputs=OPEN)])
    with pytest.raises(ManagedInterruption, match="operator Pause"):
        pick(rig)
    assert len(rig.calls) == 1
    assert rig.outputs == [(2, 0, 0)] + ([(14, 0, 0)] if channel == 14 else [])
    assert not rig.transport.moving and rig.monitor._motion is None


def test_grip_on_pick_enabled_keeps_open_through_probe(monkeypatch):
    rig, _clock = relaxation_rig(monkeypatch)
    rig.steps = iter([dict(z=.1, outputs=OPEN), *[dict(z=.1, outputs=OPEN)] * 8,
                      dict(z=.12, outputs=OPEN, command_id=7)])
    acquired, _origin = pick(rig, relax=False)
    assert not acquired and not rig.outputs
    assert rig.monitor.snapshot().feed["digital_outputs"] == OPEN


def test_zero_settling_still_confirms_relaxation_before_probe(monkeypatch):
    rig, clock = relaxation_rig(monkeypatch)
    rig.steps = iter([dict(z=.1, outputs=OPEN),
                      dict(z=.12, outputs=VACUUM, command_id=7)])
    acquired, _origin = pick(rig, settling=0.)
    assert not acquired and clock[0] == pytest.approx(.1)
    events = [a[1] for a, _ in rig.events]
    assert events.index("pickup_fingers_relaxed") < events.index("pickup_probe_started")
    assert rig.outputs == [(2, 0, 0), (14, 0, 0)]


@pytest.mark.parametrize("pending", ["reply", "output"])
def test_missing_response_or_output_echo_is_bounded_and_blocks_lift(monkeypatch, pending):
    rig, clock = relaxation_rig(monkeypatch)

    def output(request):
        if pending == "reply":
            rig.transport.node.wait_control = lambda dt: clock.__setitem__(0, clock[0] + dt)
            return Future()
        if request.index == 2:
            rig.echo(request)

    rig.on_output = output
    rig.steps = iter([dict(z=.1, outputs=OPEN)] * 120)
    with pytest.raises(CommandResponseTimeout if pending == "reply" else FeedbackFailure,
                       match="timeout|Timed out"):
        pick(rig)
    assert 5. <= clock[0] < 5.2
    assert len(rig.calls) == 1
    assert len(rig.outputs) == (1 if pending == "reply" else 2)
    assert not any(a[1] == "pickup_fingers_relaxed" for a, _ in rig.events)


@pytest.mark.parametrize("use_grip", [False, True])
@pytest.mark.parametrize("close_on_pick", [False, True])
def test_first_and_retry_acquisitions_follow_pick_flag_independently_of_transport(
        use_grip, close_on_pick):
    taught = settings(use_grip=use_grip, close_on_pick=close_on_pick)
    hardware = FakeHardware([False, True])
    plans = [pick_targets(matrix(1.), item_pose(), taught, i) for i in (1, 2)]
    outcome = PickExecutor(hardware, finish_home=False).run(
        plans, taught, tray_target=tray_target(), check=lambda _i: None,
        return_home=lambda **_k: pytest.fail("Unexpected Home"))
    assert outcome["picked"] and outcome["candidate"] == 2
    acquisitions = [kwargs for name, _value, kwargs in hardware.log
                    if name == "move" and kwargs.get("stop_on_suction")]
    assert len(acquisitions) == 2
    assert all(k["relax_pick_fingers"] == (not close_on_pick) for k in acquisitions)
    close_events = [event for event in hardware.targets[-1][0].motion_io
                    if event.channel == 2 and event.active]
    assert [event.percent for event in close_events] == (
        [50] if use_grip and not close_on_pick else [])
    assert np.array_equal(plans[1][4].matrix, acquisitions[-1]["pickup_retract_pose"])
