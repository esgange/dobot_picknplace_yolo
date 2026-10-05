"""Candidate ledger and pure managed-interruption geometry."""

from dataclasses import dataclass, replace

from .motion import Target, candidate_transit
from .release import release_targets


@dataclass
class Attempt:
    identifier: str
    plan: tuple
    state: str = "PENDING"


class PickSession:
    def __init__(self, identifiers, plans, changed=None, *, previous_attempted=0,
                 batch=None, configuration=None):
        if len(identifiers) != len(plans):
            raise ValueError("Candidate identifiers and plans must match")
        self.attempts = [Attempt(name, tuple(plan)) for name, plan in zip(identifiers, plans)]
        self.changed = changed
        self.held_index = None
        self.parked_index = None
        self.resuming = False
        self.previous_attempted = previous_attempted
        self.batch = batch
        self.configuration = configuration

    def reusable(self, configuration):
        """Saved candidates belong to this exact loaded configuration and process."""
        return (self.batch is not None and self.configuration is configuration
                and self.next_eligible is not None)

    def begin_pick(self, previous_attempted=0):
        """Count this Pick's attempts without resetting the saved candidate states."""
        self.previous_attempted = previous_attempted - sum(
            attempt.state != "PENDING" for attempt in self.attempts)

    def set_state(self, index, state):
        attempt = self.attempts[index - 1]
        allowed = {
            "PENDING": {"ACTIVE", "CANCELED"},
            "ACTIVE": {"FAILED", "INTERRUPTED", "HELD", "CANCELED"},
            "HELD": {"DROPPED", "RETURNED", "PLACED", "CANCELED"},
            "FAILED": set(), "INTERRUPTED": {"ACTIVE", "CANCELED"},
            "DROPPED": set(), "RETURNED": set(), "PLACED": set(), "CANCELED": set(),
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

    def complete_release(self, state):
        """Record an owner-confirmed release; tray placement expires unused poses."""
        if self.held_index is not None:
            if self.attempts[self.held_index - 1].state == "HELD":
                self.set_state(self.held_index, state)
            self.held_index = None
        if state == "PLACED":
            self.cancel_remaining()

    def cancel_remaining(self):
        for index, attempt in enumerate(self.attempts, 1):
            if attempt.state in ("PENDING", "ACTIVE", "INTERRUPTED"):
                self.set_state(index, "CANCELED")
        self.parked_index = None
        self.resuming = False

    @property
    def next_eligible(self):
        """An interrupted approach stays eligible ahead of later pending candidates."""
        return next((index for index, attempt in enumerate(self.attempts, 1)
                     if attempt.state in ("PENDING", "INTERRUPTED")), None)

    @property
    def attempted_count(self):
        return self.previous_attempted + sum(
            attempt.state != "PENDING" for attempt in self.attempts)


def safety_target(current, home, settings):
    """Upward-only at the measured XY and attitude; caller confirms before crossing."""
    matrix = current.copy()
    matrix[2, 3] = max(current[2, 3], home[2, 3])
    return Target("pause_safety", matrix, settings["speed"]["travel_percent"],
                  settings["acceleration"]["travel_percent"])


def transit_from_safety(current, plan):
    return replace(candidate_transit(current, plan), name="park_transit", motion_io=())


def return_targets(plan):
    """Preview/validate the same saved pre-pick release and Home-Z retract as Return Item."""
    settings = {"acceleration": {
        "travel_percent": plan[0].acceleration_percent,
        "approach_percent": plan[3].acceleration_percent,
        "retract_percent": plan[4].acceleration_percent,
    }}
    _approach, release, retract = release_targets(
        plan[2].matrix, settings, plan[0].matrix, prefix="return")
    return release, (retract,)
