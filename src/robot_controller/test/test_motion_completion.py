"""Exercise motion completion with the real feedback and endpoint predicates."""

from types import SimpleNamespace

import numpy as np
import pytest

from robot_controller.errors import FeedbackFailure
import robot_controller.hardware as hardware_module
from robot_controller.hardware import DobotTransport
from robot_controller.motion import MotionIO, Target

from test_feedback_v2 import feed, primed_monitor


class MotionRig:
    def __init__(self):
        self.monitor = primed_monitor()
        self.timer = 1
        self.events = []
        self.calls = []
        self.waited = []
        self.steps = iter(())
        self.during_admission = lambda: self.emit()
        self.transport = object.__new__(DobotTransport)
        self.transport.node = SimpleNamespace(
            expected_outputs={}, cancel_requested=lambda: False,
            raise_if_cancelled=lambda: None, operation_progress=lambda *_a, **_k: None,
            events=SimpleNamespace(record=lambda *a, **k: self.events.append((a, k))))
        self.transport.monitor = self.monitor
        self.transport.current_pose = lambda: np.eye(4)
        self.transport._wait_for_resume = lambda: 0.
        self.transport._ready_snapshot = lambda: self.monitor.snapshot(require_enabled=True)
        self.transport.call_group = self.dispatch
        self.monitor.wait_next = self.next_sample

    def emit(self, *, z=0., command_id=0, running=0, advance=True, outputs=0):
        self.timer += int(advance)
        self.monitor.update_feed(feed(
            controller_timer=self.timer, currentCommandId=command_id,
            tool_vector_actual=[0., 0., z * 1000, 0., 0., 0.],
            RunningStatus=running, isRunQueuedCmd=running, digital_outputs=outputs))

    def dispatch(self, calls, **_kwargs):
        self.calls.extend(calls)
        self.during_admission()
        return tuple(SimpleNamespace(res=0, robot_return="{7}") if name == "MovL"
                     else SimpleNamespace(res=0) for name, _fields in calls)

    def next_sample(self, *_args, **_kwargs):
        values = next(self.steps, None)
        if values is None:
            pytest.fail("Completion waited beyond the final valid feedback sample")
        self.waited.append(values)
        self.emit(**values)
        return self.monitor.sequence

    def run(self, *, z=.001, targets=None):
        matrix = np.eye(4)
        matrix[2, 3] = z
        return self.transport.move_batch(
            targets or (Target("end", matrix, 100, 100),), batch_name="test")


def test_old_idle_near_endpoint_cannot_complete_a_new_move():
    rig = MotionRig()
    rig.steps = iter([
        {},  # Still idle and already within 5 mm, but the new command has not run.
        {"z": .001, "command_id": 7, "running": 1},
        {"z": .001, "command_id": 7},
    ])
    assert not rig.run()
    assert len(rig.waited) == 3
    assert rig.events[-1][1]["terminal_stable_sec"] == 0.


def test_earlier_group_command_at_endpoint_is_not_terminal_completion():
    rig = MotionRig()
    midpoint = np.eye(4)
    midpoint[2, 3] = .1
    rig.steps = iter([
        {"command_id": 6},  # Earlier command, near the final/start pose.
        {"z": .1, "command_id": 7},  # Correct ID but wrong actual endpoint.
        {"command_id": 7},
    ])
    rig.run(targets=(Target("exit", midpoint, 100, 100),
                     Target("entry", midpoint, 100, 100),
                     Target("end", np.eye(4), 100, 100)))
    assert len(rig.calls) == 3
    assert len(rig.waited) == 3
    assert all(not any(p.startswith(("cp=", "r=")) for p in fields["param_value"])
               for _name, fields in rig.calls)


@pytest.mark.parametrize("zero_distance", [False, True])
def test_fast_or_zero_distance_command_needs_no_observed_running_or_dwell(zero_distance):
    rig = MotionRig()
    z = 0. if zero_distance else .1
    rig.during_admission = lambda: rig.emit(z=z, command_id=7)
    rig.steps = iter([{"z": z, "command_id": 7}])
    rig.run(z=z)
    assert len(rig.calls) == len(rig.waited) == 1


def test_frozen_controller_timer_cannot_supply_new_terminal_evidence():
    rig = MotionRig()
    rig.steps = iter([{"z": .001, "command_id": 7, "advance": False},
                      {"z": .001, "command_id": 7}])
    rig.run()
    assert len(rig.waited) == 2


def test_zero_length_required_timed_io_still_queues_and_confirms():
    rig = MotionRig()
    rig.steps = iter([{"command_id": 7}])
    rig.run(targets=(Target("exit", np.eye(4), 100, 100,
                            motion_io=(MotionIO(0, 13, False),)),))
    assert rig.calls[0][0] == "MovLIO"
    assert rig.transport.node.expected_outputs[13] is False


@pytest.mark.parametrize("relative", [False, True])
def test_acceptance_only_services_wait_for_execution_evidence_and_fresh_idle(relative):
    rig = MotionRig()
    target = np.eye(4)
    target[2, 3] = .001
    rig.steps = iter([{}, {"z": .001, "command_id": 7, "running": 1},
                      {"z": .001, "command_id": 7}])
    rig.run(targets=(Target("end", target, 100, 100, relative_z=relative,
                            motion_io=() if relative else (MotionIO(0, 13, False),)),))
    assert len(rig.waited) == 3
    assert rig.calls[0][0] == ("RelMovLUser" if relative else "MovLIO")


def test_execution_evidence_is_retained_when_move_finishes_before_reply():
    rig = MotionRig()

    def finished():
        rig.emit(z=.001, command_id=7, running=1)
        rig.emit(z=.001, command_id=7)
    rig.during_admission = finished
    rig.steps = iter([{"z": .001, "command_id": 7}])
    target = np.eye(4)
    target[2, 3] = .001
    rig.run(targets=(Target("end", target, 100, 100,
                            motion_io=(MotionIO(0, 13, False),)),))
    assert len(rig.waited) == 1
    # Observation is released so the next group can acquire it independently.
    observation = rig.monitor.begin_motion()
    assert not observation.started
    rig.monitor.end_motion(observation)


def test_terminal_pose_idle_and_io_must_match_in_the_same_sample():
    rig = MotionRig()
    target = np.eye(4)
    target[2, 3] = .1
    rig.steps = iter([
        {"z": .1, "command_id": 7},  # Endpoint reached but required OPEN is absent.
        {"z": .08, "command_id": 7, "outputs": 1 << 13},  # I/O alone is insufficient.
        {"z": .1, "command_id": 7, "outputs": 1 << 13},
    ])
    rig.run(targets=(Target("end", target, 100, 100,
                            motion_io=(MotionIO(0, 14, True),)),))
    assert len(rig.waited) == 3
    assert rig.transport.node.expected_outputs[14]


def test_non_pick_terminal_io_keeps_its_five_second_deadline(monkeypatch):
    rig = MotionRig()
    clock = [0.]
    monkeypatch.setattr(hardware_module, "time", SimpleNamespace(monotonic=lambda: clock[0]))

    def next_sample(*args, **kwargs):
        clock[0] += 1.
        return rig.next_sample(*args, **kwargs)
    rig.monitor.wait_next = next_sample
    rig.steps = iter([{"z": .1, "command_id": 7}] * 6)
    target = np.eye(4)
    target[2, 3] = .1
    with pytest.raises(FeedbackFailure, match="motion-timed output feedback"):
        rig.run(targets=(Target("end", target, 100, 100,
                                motion_io=(MotionIO(0, 14, True),)),))
    assert clock[0] == 6.  # Endpoint first observed at 1 s; output deadline at 6 s.


@pytest.mark.parametrize("value", [None, "", "{}", "{-1}", "{1.0}", "{1,2}",
                                   "0,{7},MovL();", "{18446744073709551616}"])
def test_invalid_queue_ids_fail_closed(value):
    with pytest.raises(FeedbackFailure, match="queued command ID"):
        DobotTransport._motion_command_id(SimpleNamespace(robot_return=value))
