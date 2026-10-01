"""Shared motion-timed release geometry and feedback for tray placement and bin return."""

from dataclasses import dataclass, field

from .errors import FeedbackFailure
from .motion import (Target, gripper_neutral_events, rigid_matrix, vacuum_neutral_events,
                     gripper_open_events, vacuum_exhaust_events)


def release_targets(release_pose, settings, home_matrix, *, prefix):
    """Approach at Home Z, release at 80% down, neutral at 20% back up."""
    release = rigid_matrix(release_pose, "Release target")
    home = rigid_matrix(home_matrix, "Home")
    pre_z = home[2, 3]
    if pre_z <= release[2, 3]:
        raise ValueError("Placement approach at Home Z must be above the drop height")
    result = []
    for name, z, rate in (("pre", pre_z, "travel_percent"),
                          ("release", release[2, 3], "approach_percent"),
                          ("retract", pre_z, "retract_percent")):
        point = release.copy()
        point[2, 3] = z
        events = ()
        if name == "release":
            events = gripper_open_events(80) + vacuum_exhaust_events(80)
        elif name == "retract":
            events = gripper_neutral_events(20) + vacuum_neutral_events(20)
        result.append(Target(f"{prefix}_{name}", point, 100,
                             settings["acceleration"][rate], motion_io=events))
    return tuple(result)


@dataclass(kw_only=True)
class ReleaseQueue:
    phase: str = "OBSERVE"
    plan: tuple = ()
    pending_outputs: dict = field(default_factory=dict)
    release_issued: bool = False
    neutral_issued: bool = False
    release_confirmed: bool = False
    history_sequence: int = 0
    queue_start_sequence: int = 0
    initial_outputs: int = 0
    observing: bool = False
    release_index: int = 1
    neutral_index: int = 2

    candidate_state = "PLACED"
    release_event = "placement_release_observed"
    completion_event = "placement_retract_completed"
    completion_message = "Placement queue completed above tray"
    completion_error = "Retract reached; final placement outputs must be neutral and DI1 LOW"

    def move_drop(self, node, *, batch_name, confirmed_start_pose=None):
        """Finish the drop before admitting upward motion, with no settling dwell."""
        _acquired, current = node.hardware.move_batch(
            self.plan[:self.neutral_index], batch_name=batch_name,
            placement=ReleaseSegment(self, 0, final=False),
            confirmed_start_pose=confirmed_start_pose, return_terminal_pose=True)
        return current

    def move_return(self, node, *, batch_name, current, queue_only=False):
        return node.hardware.move_batch(
            self.plan[self.neutral_index:], batch_name=batch_name,
            placement=ReleaseSegment(self, self.neutral_index, final=True),
            confirmed_start_pose=current, queue_only=queue_only)

    def begin_queue(self, node):
        sample = node.monitor.snapshot(require_enabled=True)
        mask = (1 << 13) | (1 << 12) | 3
        self.initial_outputs = sample.feed["digital_outputs"] & mask
        self.history_sequence = sample.sequence
        self.queue_start_sequence = sample.sequence
        self.release_issued = self.neutral_issued = False
        self.release_confirmed = False
        self.observing = True

    def issued(self, index):
        # Called before dispatch: a timed command with an uncertain response may
        # have executed. Never assume it is safe to repeat its release.
        if index == self.release_index:
            self.release_issued = True
            self.phase = "RELEASING"
        elif index == self.neutral_index:
            self.neutral_issued = True

    def observe(self, node, sample):
        """Record available release evidence; never gate the queue on DI1/DI12."""
        if not self.observing:
            return
        with node.managed.lock:
            mask = (1 << 13) | (1 << 12) | 3
            # History is diagnostic only. A gap or unobserved release interval
            # must not interrupt the admitted approach/drop/retract queue.
            for sequence, _timer, outputs, inputs in node.monitor.output_history(
                    self.history_sequence):
                if sequence > sample.sequence:
                    break
                bits = outputs & mask
                self.history_sequence = sequence
                if self.release_issued and bits != self.initial_outputs:
                    node.holding_item = False
                    if not self.release_confirmed:
                        self.phase = "RELEASING"
                if (self.release_issued and bits == ((1 << 13) | 1)
                        and bits != self.initial_outputs
                        and inputs & (1 << 11) and not inputs & 1):
                    if not self.release_confirmed:
                        self.release_confirmed = True
                        self.phase = "RELEASED"
                        node.events.record(
                            "INFO", self.release_event,
                            "Queued release feedback observed")
                node.expected_outputs.update({ch: bool(bits & (1 << (ch - 1)))
                                              for ch in (1, 2, 13, 14)})

    def complete(self, node, sample):
        # The owner calls this after physical arrival at final retract or Home.
        # No intermediate release state or full-open sensor is a success gate.
        self.observe(node, sample)
        if (sample.feed["digital_input_bits"] & 1
                or sample.feed["digital_outputs"] & ((1 << 13) | (1 << 12) | 3)):
            raise FeedbackFailure(self.completion_error)
        node.holding_item = False
        node.expected_outputs.update(dict.fromkeys((1, 2, 13, 14), False))
        session = node.managed.session
        if session is not None and session.held_index is not None:
            if session.attempts[session.held_index - 1].state == "HELD":
                session.set_state(session.held_index, self.candidate_state)
            session.held_index = None
        node.events.record(
            "INFO", self.completion_event, self.completion_message,
            release_feedback_observed=self.release_confirmed)
        self.phase = "DONE"
        self.observing = False


@dataclass(frozen=True)
class ReleaseSegment:
    """Keep one release history across two separately confirmed motion groups."""

    owner: ReleaseQueue
    offset: int
    final: bool

    def issued(self, index):
        self.owner.issued(self.offset + index)

    def observe(self, node, sample):
        self.owner.observe(node, sample)

    def complete(self, node, sample):
        if self.final:
            self.owner.complete(node, sample)
        else:
            self.owner.observe(node, sample)
            node.events.record(
                "INFO", "release_drop_arrived",
                "Drop pose and idle confirmed; queue return without settling",
                target=self.owner.plan[self.owner.release_index].name)
