"""Explicit held-item return: placement timing, saved pre-pick target, queued Home."""

from dataclasses import dataclass

from .errors import FeedbackFailure, HeldUnknown
from .motion import CARTESIAN_POSITION_TOLERANCE_M, Target, home_targets
from .release import ReleaseQueue, release_targets


@dataclass(kw_only=True)
class ItemReturnOperation(ReleaseQueue):
    index: int
    dropped: bool = False
    continue_candidates: bool = False

    candidate_state = "RETURNED"
    release_event = "item_return_release_observed"
    completion_event = "item_return_home_completed"
    completion_message = "Queued item return completed at taught Home"
    completion_error = "Home reached; final return outputs must be neutral and DI1 LOW"

    def run(self, node):
        managed, config = node.managed, node.configuration
        if self.phase != "APPROACH" or self.release_issued:
            raise FeedbackFailure("Interrupted queued Return Item requires Recover")
        if (managed.session.held_index != self.index
                or managed.session.attempts[self.index - 1].state != "HELD"):
            raise HeldUnknown("Return Item requires its trusted held candidate")
        config.validate_sources(node.root)
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
        home = home_targets(
            release[-1].matrix, config.home_matrix, config.home_joints,
            speed_percent=100,
            acceleration_percent=config.profile["acceleration"]["travel_percent"])
        self.plan = (*preceding, *release, *home)
        self.release_index, self.neutral_index = len(preceding) + 1, len(preceding) + 2
        node._transition("RETURNING_ITEM", "Queueing saved pre-pick drop, then retract and Home")
        self.begin_queue(node)
        node.operation_progress("RETURN_QUEUE", "Queueing approach, drop, retract and taught Home",
                                waypoint="home")
        node.hardware.move_batch(self.plan, batch_name="return_item_queued_home",
                                 placement=self, confirmed_start_pose=current)
        managed.session.parked_index = None
        managed.return_progress = None
        node.events.record("INFO", "item_return_completed",
                           "Return queue completed; physical placement not measured",
                           home_confirmed=True, dropped=False, continuing_candidates=False)
