"""Shared paused/automatic item-return queue and direct saved-pick continuation."""

from dataclasses import dataclass

from .errors import FeedbackFailure, HeldUnknown
from .motion import CARTESIAN_POSITION_TOLERANCE_M, Target, home_targets
from .release import ReleaseQueue, release_targets
from .recovery import GRIP_MASK


@dataclass(kw_only=True)
class ItemReturnOperation(ReleaseQueue):
    index: int
    dropped: bool = False
    continue_candidates: bool = False
    continuation: object = None

    candidate_state = "RETURNED"
    release_event = "item_return_release_observed"
    completion_event = "item_return_home_completed"
    completion_message = "Queued item return completed at taught Home"
    completion_error = "Home reached; final return outputs must be neutral and DI1 LOW"

    def observe(self, node, sample, *, raise_on_loss=True):
        if self.continuation is not None:
            self.continuation.observe(sample)
        else:
            super().observe(node, sample, raise_on_loss=raise_on_loss)

    def prepare(self, node, *, finish_home=True):
        """Use paused Return Item's exact route/rates/I/O; choose only its tail."""
        managed, config = node.managed, node.configuration
        if self.phase != "APPROACH" or self.release_issued:
            raise FeedbackFailure("Interrupted queued Return Item requires Recover")
        if (managed.session.held_index != self.index
                or managed.session.attempts[self.index - 1].state
                != ("DROPPED" if self.dropped else "HELD")):
            raise HeldUnknown("Return Item requires its retained source candidate")
        config.validate_sources(node.root)
        if self.dropped:
            node.monitor.snapshot(require_enabled=True)
        else:
            node._preflight_item_state(True)
        current = node.hardware.current_pose()
        source = managed.session.attempts[self.index - 1].plan[2].matrix
        release = release_targets(source, config.profile, config.home_matrix, prefix="return")
        preceding = ()
        if current[2, 3] < config.home_matrix[2, 3] - CARTESIAN_POSITION_TOLERANCE_M:
            rise = current.copy()
            rise[2, 3] = config.home_matrix[2, 3]
            preceding = (Target("return_safety", rise, 100,
                                config.profile["acceleration"]["travel_percent"]),)
        home = ()
        if finish_home:
            home = home_targets(
                release[-1].matrix, config.home_matrix, config.home_joints,
                speed_percent=100,
                acceleration_percent=config.profile["acceleration"]["travel_percent"])
        self.plan = (*preceding, *release, *home)
        self.release_index, self.neutral_index = len(preceding) + 1, len(preceding) + 2
        if not finish_home:
            self.completion_event = "item_return_retract_completed"
            self.completion_message = "Queued item return completed above saved source"
            self.completion_error = "Return retract reached; outputs must be neutral and DI1 LOW"
        node._transition("RETURNING_ITEM", "Queueing saved pre-pick release and upward retract")
        self.begin_queue(node)
        node.operation_progress("RETURN_QUEUE", "Queueing shared Return Item route",
                                waypoint=self.plan[-1].name)
        return current

    def run(self, node, *, finish_home=True):
        current = self.prepare(node, finish_home=finish_home)
        batch = "return_item_queued_home" if finish_home else "return_item_queued_retract"
        node.hardware.move_batch(self.plan, batch_name=batch, placement=self,
                                 confirmed_start_pose=current)
        node.managed.session.parked_index = None
        node.managed.return_progress = None
        node.events.record("INFO", "item_return_completed",
                           "Return queue completed; physical placement not measured",
                           home_confirmed=finish_home, dropped=self.dropped,
                           continuing_candidates=False)


class ReturnPickBridge:
    """A single return/next-pick queue, with raw suction armed only after release."""

    def __init__(self, node, operation, origin):
        self.node = node
        self.operation = operation
        operation.continuation = self
        self.origin = origin
        self.next_session = node.managed.session
        self.completed = False
        self.neutral_seen = False
        self.neutral_sequence = None
        self.boundary_id = None
        self.accepted_sequence = None
        self.targets = []
        # Return prefix, next entry (MovLIO), then next clearance (MovL).
        self.boundary_index = len(operation.plan) + 1

    def issued(self, index):
        with self.node.managed.lock:
            if index < len(self.operation.plan):
                if index == self.operation.neutral_index:
                    self.neutral_sequence = self.node.monitor.sequence
                self.operation.issued(index)

    def accepted(self, index, response):
        with self.node.managed.lock:
            if index == self.boundary_index:
                self.boundary_id = self.node.hardware._motion_command_id(response)
                self.accepted_sequence = self.node.monitor.sequence

    def admitted(self, target):
        with self.node.managed.lock:
            self.targets.append(target)
            if self.completed:
                self.next_session.admitted(target)

    def observe(self, sample):
        with self.node.managed.lock:
            self._observe(sample)

    def _observe(self, sample):
        if self.completed:
            return
        for sequence, _timer, outputs, inputs in self.node.monitor.output_history(
                self.operation.queue_start_sequence):
            if (self.neutral_sequence is not None
                    and self.neutral_sequence < sequence <= sample.sequence
                    and not outputs & GRIP_MASK and not inputs & 1):
                self.neutral_seen = True
        if (self.boundary_id is None or sample.sequence <= self.accepted_sequence
                or sample.feed["currentCommandId"] < self.boundary_id):
            ReleaseQueue.observe(self.operation, self.node, sample)
            return
        if not self.neutral_seen:
            raise FeedbackFailure(
                "Return crossed into next Pick without neutral outputs and DI1 LOW")
        self.operation.phase = "DONE"
        self.operation.observing = False
        self.completed = True
        self.next_session.held_index = None
        self.next_session.parked_index = None
        self.node.managed.return_progress = None
        for target in self.targets:
            self.next_session.admitted(target)
        self.node._transition("PICKING", "Return completed; next saved Pick is executing")
        self.node.events.record("INFO", "item_return_completed",
                                "Return passed; continuing saved candidate queue",
                                home_confirmed=False, dropped=True, continuing_candidates=True)
