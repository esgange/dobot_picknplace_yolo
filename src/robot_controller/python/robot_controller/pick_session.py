"""Candidate ledger and pure managed-interruption geometry."""

from dataclasses import dataclass, replace

from .motion import (Target, candidate_transit, gripper_neutral_events,
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
    return (transit_from_safety(current, plan),
            replace(plan[2], name="park_prepick", motion_io=()))


def return_targets(plan):
    """Release exactly 50 mm above nominal Link6 pick, never above pre-pick."""
    drop = plan[3].matrix.copy()
    drop[2, 3] += 0.050
    if plan[2].matrix[2, 3] + 1e-9 < drop[2, 3]:
        raise ValueError("Put-back requires pre-pick at least 50 mm above final pick")
    if plan[5].matrix[2, 3] <= drop[2, 3] + 1e-9:
        raise ValueError("Put-back requires clearance above the release pose for upward retreat")
    release = replace(plan[3], name="return_release", matrix=drop, motion_io=())
    neutral = gripper_neutral_events(0) + vacuum_neutral_events(0)
    retreat = []
    if plan[2].matrix[2, 3] > drop[2, 3] + 1e-9:
        retreat.append(replace(plan[4], name="return_prepick", motion_io=neutral))
    if not retreat or plan[5].matrix[2, 3] > plan[2].matrix[2, 3] + 1e-9:
        retreat.append(replace(plan[5], name="return_clearance",
                               motion_io=() if retreat else neutral))
    return release, tuple(retreat)
