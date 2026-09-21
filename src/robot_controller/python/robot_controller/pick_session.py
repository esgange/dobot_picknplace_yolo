"""Candidate ledger and pure managed-interruption geometry."""

from dataclasses import dataclass, replace

from .motion import (Target, candidate_exit_transit, candidate_transit, gripper_neutral_events,
                     vacuum_neutral_events)


@dataclass
class Attempt:
    identifier: str
    plan: tuple
    state: str = "PENDING"


class PickSession:
    def __init__(self, identifiers, plans, changed=None):
        if len(identifiers) != len(plans):
            raise ValueError("Candidate identifiers and plans must match")
        self.attempts = [Attempt(name, tuple(plan)) for name, plan in zip(identifiers, plans)]
        self.changed = changed
        self.held_index = None
        self.parked_index = None
        self.resuming = False

    def set_state(self, index, state):
        attempt = self.attempts[index - 1]
        allowed = {
            "PENDING": {"ACTIVE"},
            "ACTIVE": {"FAILED", "INTERRUPTED", "HELD"},
            "HELD": {"DROPPED", "RETURNED"},
            "FAILED": set(), "INTERRUPTED": {"ACTIVE"},
            "DROPPED": set(), "RETURNED": set(),
        }
        if state == attempt.state:
            return
        if state not in allowed[attempt.state]:
            raise ValueError(f"Invalid candidate transition {attempt.state} -> {state}")
        attempt.state = state
        if state == "HELD":
            self.held_index = index
        if self.changed:
            self.changed(index, attempt)

    def admitted(self, target):
        for index, attempt in enumerate(self.attempts, 1):
            if target.name in {point.name for point in attempt.plan[:4]}:
                if attempt.state in ("PENDING", "INTERRUPTED"):
                    self.set_state(index, "ACTIVE")
                return

    def interrupt_active(self):
        for index, attempt in enumerate(self.attempts, 1):
            if attempt.state == "ACTIVE":
                self.set_state(index, "INTERRUPTED")

    @property
    def next_eligible(self):
        """An interrupted approach stays eligible ahead of later pending candidates."""
        return next((index for index, attempt in enumerate(self.attempts, 1)
                     if attempt.state in ("PENDING", "INTERRUPTED")), None)

    @property
    def attempted_count(self):
        return sum(attempt.state != "PENDING" for attempt in self.attempts)


def safety_target(current, home, settings):
    """Upward-only at the measured XY and attitude; caller confirms before crossing."""
    matrix = current.copy()
    matrix[2, 3] = max(current[2, 3], home[2, 3])
    return Target("pause_safety", matrix, settings["speed"]["travel_percent"],
                  settings["acceleration"]["travel_percent"])


def transit_from_safety(current, plan):
    return replace(candidate_transit(current, plan), name="park_transit", motion_io=())


def approach_from_safety(current, plan):
    """Put-back approach uses full commanded speed and taught travel acceleration."""
    return (replace(transit_from_safety(current, plan), speed_percent=100),
            replace(plan[2], name="park_prepick", speed_percent=100, motion_io=()))


def return_targets(plan):
    """Release at the taught pre-pick pose; neutralize on the first real rise."""
    rates = {"speed_percent": 100, "acceleration_percent": plan[0].acceleration_percent}
    release = replace(plan[2], name="return_release", motion_io=(), **rates)
    neutral = gripper_neutral_events(0) + vacuum_neutral_events(0)
    retreat = []
    if plan[5].matrix[2, 3] > release.matrix[2, 3] + 1e-9:
        retreat.append(replace(plan[5], name="return_clearance", motion_io=(), **rates))
    origin = retreat[-1].matrix if retreat else release.matrix
    retreat.append(replace(candidate_exit_transit(origin, plan),
                           name="return_park_transit", **rates))
    if retreat[0].matrix[2, 3] <= release.matrix[2, 3] + 1e-9:
        raise ValueError("Put-back requires clearance or safety Z above the taught "
                         "pre-pick release pose for upward retreat")
    retreat[0] = replace(retreat[0], motion_io=neutral)
    return release, tuple(retreat)
