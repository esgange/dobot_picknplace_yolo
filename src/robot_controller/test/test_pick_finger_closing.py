"""Grab on Pick closes on observed arrival, with independent early DI1 Stop."""

from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from robot_controller.errors import (
    CommandRejected, CommandResponseTimeout, FeedbackFailure, ManagedInterruption,
    OperationCanceled)
from robot_controller.motion import Target, gripper_neutral_events, vacuum_suck_events
from test_motion_v2 import matrix
from test_pick_acquisition import VACUUM
from test_pick_finger_relaxation import relaxation_rig


CLOSED = VACUUM | 2


def pick(rig, settling=.3):
    target = Target('pick', matrix(.1), 10, 35,
                    motion_io=gripper_neutral_events(0) + vacuum_suck_events(0))
    return rig.run(targets=(target,), stop_on_suction=True, close_pick_fingers=True,
                   pick_settling_sec=settling, pickup_retract_pose=matrix(.14),
                   return_terminal_pose=True)


@pytest.mark.parametrize('settling', [0., .3])
def test_close_at_arrival_without_di1_then_probe_with_closed_fingers(monkeypatch, settling):
    rig, clock = relaxation_rig(monkeypatch)
    rig.steps = iter([dict(z=.14, outputs=VACUUM, running=1), dict(z=.1, outputs=VACUUM),
                      *[dict(z=.1, outputs=CLOSED)] * 8,
                      dict(z=.14, outputs=CLOSED, command_id=7)])
    where = []

    def echo(request):
        where.append((rig.monitor.snapshot().joints[0], clock[0]))
        rig.echo(request)

    rig.on_output = echo
    acquired, origin = pick(rig, settling)
    assert not acquired and origin[2, 3] == pytest.approx(.14)
    assert where == [(.1, .1), (.1, .1)]
    assert rig.outputs == [(14, 0, 0), (2, 1, 0)]
    events = [a[1] for a, _ in rig.events]
    assert events.index('pickup_fingers_closed') < events.index('pickup_probe_started')
    assert not rig.stops
    assert rig.transport.node.expected_outputs[2] is True


@pytest.mark.parametrize('phase', ['descent', 'settling', 'probe'])
def test_early_stop_or_arrival_closes_exactly_once(monkeypatch, phase):
    rig, _clock = relaxation_rig(monkeypatch)
    before = [] if phase == 'descent' else [dict(z=.1, outputs=VACUUM)]
    if phase == 'probe':
        before += [dict(z=.1, outputs=CLOSED)] * 8
    height = .15 if phase == 'descent' else .112 if phase == 'probe' else .1
    rig.steps = iter([
        *before,
        dict(z=height, outputs=VACUUM if phase == 'descent' else CLOSED, running=1, di1=True)])
    acquired, origin = pick(rig)
    assert acquired and origin[2, 3] == pytest.approx(height)
    assert rig.outputs == [(14, 0, 0), (2, 1, 0)] and len(rig.stops) == 1
    closed = [k for a, k in rig.events if a[1] == 'pickup_fingers_closed']
    assert len(closed) == 1
    assert closed[0]['trigger'] == (
        'pickup_stop_accepted' if phase == 'descent' else 'final_pick_arrival')


@pytest.mark.parametrize('channel', [14, 2])
@pytest.mark.parametrize('echo_late', [False, True])
def test_di1_during_close_wait_finishes_reply_and_echo_before_stop(
        monkeypatch, channel, echo_late):
    rig, _clock = relaxation_rig(monkeypatch)
    pending = Future()
    order = []
    stop = rig.transport.request_stop

    def output(request):
        order.append(f'DO{request.index}')
        if request.index == channel:
            rig.echo(request, di1=True, outputs=VACUUM if echo_late else None)
            return pending
        rig.echo(request)

    def respond(_seconds):
        assert not rig.stops
        order.append('reply')
        pending.set_result(SimpleNamespace(res=0))

    def request_stop(reason, **kwargs):
        assert pending.done()
        bits = rig.monitor.snapshot().feed['digital_outputs']
        assert bool(bits & (1 << (channel - 1))) == (channel == 2)
        order.append('Stop')
        return stop(reason, **kwargs)

    rig.transport.request_stop = request_stop
    rig.transport.node.wait_control = respond
    rig.on_output = output
    rig.steps = iter([dict(z=.1, outputs=VACUUM),
                      dict(z=.1, outputs=CLOSED if channel == 2 else VACUUM, di1=True)])
    acquired, _origin = pick(rig)
    assert acquired and len(rig.stops) == 1
    assert order == (['DO14', 'reply', 'Stop', 'DO2'] if channel == 14 else
                     ['DO14', 'DO2', 'reply', 'Stop'])
    assert rig.monitor.snapshot().feed['digital_outputs'] == CLOSED


@pytest.mark.parametrize('failure', ['reject', 'timeout', 'echo'])
def test_close_failure_blocks_probe_and_lift(monkeypatch, failure):
    rig, clock = relaxation_rig(monkeypatch)

    def output(request):
        if request.index == 14:
            return rig.echo(request)
        if failure == 'echo':
            return
        pending = Future()
        if failure == 'reject':
            pending.set_result(SimpleNamespace(res=-1))
        return pending

    rig.on_output = output
    rig.transport.node.wait_control = lambda dt: clock.__setitem__(0, clock[0] + dt)
    rig.steps = iter([dict(z=.1, outputs=VACUUM)] * 150)
    error = {'reject': CommandRejected, 'timeout': CommandResponseTimeout,
             'echo': FeedbackFailure}[failure]
    with pytest.raises(error):
        pick(rig)
    assert len(rig.calls) == 1
    assert not any(a[1] == 'pickup_probe_started' for a, _ in rig.events)


@pytest.mark.parametrize('interruption', [OperationCanceled, ManagedInterruption])
def test_stop_or_pause_during_close_blocks_remaining_output_and_probe(monkeypatch, interruption):
    rig, _clock = relaxation_rig(monkeypatch)
    rig.transport._ready_snapshot = lambda: type(rig.transport)._ready_snapshot(rig.transport)

    def output(request):
        rig.echo(request)

        def interrupted():
            raise interruption('operator interruption')

        rig.transport._wait_for_resume = interrupted

    rig.on_output = output
    rig.steps = iter([dict(z=.1, outputs=VACUUM)])
    with pytest.raises(interruption):
        pick(rig)
    assert rig.outputs == [(14, 0, 0)] and len(rig.calls) == 1


def test_closed_output_loss_during_settling_blocks_probe(monkeypatch):
    rig, _clock = relaxation_rig(monkeypatch)
    rig.steps = iter([dict(z=.1, outputs=VACUUM), dict(z=.1, outputs=VACUUM)])
    with pytest.raises(FeedbackFailure, match='output DO2 changed'):
        pick(rig)
    assert rig.outputs == [(14, 0, 0), (2, 1, 0)] and len(rig.calls) == 1
