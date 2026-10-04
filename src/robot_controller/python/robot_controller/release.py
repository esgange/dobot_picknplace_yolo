"""Shared motion-timed release geometry and feedback for tray placement and bin return."""

from dataclasses import dataclass, field

from .errors import FeedbackFailure
from .motion import (Target, gripper_neutral_events, gripper_open_events, rigid_matrix,
                     vacuum_exhaust_events, vacuum_neutral_events)


def release_targets(release_pose, settings, home_matrix, *, prefix):
    """Open fingers and exhaust at 90% descent, neutral as retract starts."""
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
            events = gripper_open_events(90) + vacuum_exhaust_events(90)
        elif name == "retract":
            events = gripper_neutral_events(0) + vacuum_neutral_events(0)
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

    def observe(self, node, sample, *, raise_on_loss=True):
        """Monitor held loss until the commanded suction-off is observed."""
        if not self.observing:
            return
        with node.managed.lock:
            mask = (1 << 13) | (1 << 12) | 3
            # Missing an intermediate release sample is not a completion gate.
            # Reconcile only issued transitions before checking held suction.
            for sequence, _timer, outputs, inputs in node.monitor.output_history(
                    self.history_sequence):
                if sequence > sample.sequence:
                    break
                bits = outputs & mask
                # Only issued timed transitions can end held monitoring. Do not
                # adopt an unexpected suction-off as an intentional release.
                released_bits = (1 << 13) | 1
                for channel in (1, 2, 13, 14):
                    bit = 1 << (channel - 1)
                    allowed = {bool(self.initial_outputs & bit)}
                    if self.release_issued:
                        allowed.add(bool(released_bits & bit))
                    if self.neutral_issued:
                        allowed.add(False)
                    if bool(bits & bit) not in allowed:
                        raise FeedbackFailure(f"Uncommanded release output DO{channel}")
                self.history_sequence = sequence
                releasing = (self.release_issued and not bits & (1 << 12)
                             and bits != self.initial_outputs)
                interrupted = node.managed.held_loss_pending
                if releasing and not interrupted:
                    node.holding_item = False
                    if not self.release_confirmed:
                        self.phase = "RELEASING"
                if (not interrupted and self.release_issued and bits == ((1 << 13) | 1)
                        and bits != self.initial_outputs
                        and not inputs & 1):
                    if not self.release_confirmed:
                        self.release_confirmed = True
                        self.phase = "RELEASED"
                        node.events.record(
                            "INFO", self.release_event,
                            "Queued release feedback observed")
                node.expected_outputs.update({ch: bool(bits & (1 << (ch - 1)))
                                              for ch in (1, 2, 13, 14)})
            node.managed.interrupt_held_loss(sample)
            if (raise_on_loss and node.managed.held_loss_pending
                    and not node.managed.containing_loss):
                node.managed.checkpoint()

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
