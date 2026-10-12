"""Candidate ledger and pure managed-interruption geometry."""

from dataclasses import dataclass, replace

from .motion import Target, candidate_transit
from .release import release_targets
from .repick import TrickleTargetRejected, trickle_plan


@dataclass
class Attempt:
    identifier: str
    plan: tuple
    state: str = "PENDING"
    area_id: int | None = None
    pickup_matrix: object = None


class PickSession:
    def __init__(self, identifiers, plans, changed=None, *, previous_attempted=0,
                 batch=None, configuration=None, repick=None, clearing_area_id=None):
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
        self.repick = repick
        self.nominal_plans = tuple(attempt.plan for attempt in self.attempts)
        self.clearing_area_id = clearing_area_id
        self.trickle_rejections = {}
        self.trickle_details = {}

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
        if self.repick is not None:
            if state in ("HELD", "FAILED"):
                attempt.area_id = self.repick.completed(
                    self.batch.candidates[index - 1].position_m, attempt.identifier)
            elif state == "PLACED":
                self.repick.placed(attempt.area_id)
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
                     if attempt.state in ("PENDING", "INTERRUPTED")
                     and not self.blocked(index) and self.prepare_trickle(index)), None)

    def prepare_trickle(self, index):
        """Refresh pending targets after each outcome; an interrupted attempt is unchanged."""
        step = self.repick.settings["trickle_depth"] if self.repick else 0.
        if not step or not self.repick.settings["blaclist_trigger"]:
            return True
        attempt = self.attempts[index - 1]
        if attempt.state != "PENDING":
            return True
        prior, area_id = self.repick.prior_attempts(
            self.batch.candidates[index - 1].position_m,
            clearing_area_id=self.clearing_area_id)
        extra = prior * step
        details = dict(batch_id=self.batch.identifier, candidate_id=attempt.identifier,
                       area_id=area_id, prior_attempts=prior, extra_depth_mm=extra,
                       nominal_z_mm=self.nominal_plans[index - 1][3].matrix[2, 3] * 1000)
        try:
            plan, height = trickle_plan(self.nominal_plans[index - 1], self.configuration, extra)
        except TrickleTargetRejected as exc:
            self.trickle_rejections[index] = str(exc)
            details["reason"] = str(exc)
            accepted = False
            message = f"Candidate {attempt.identifier} skipped: {exc}"
        else:
            self.trickle_rejections.pop(index, None)
            attempt.plan = plan
            details.update(target_z_mm=height.surface_z_m * 1000,
                           plane_z_mm=height.plane_z_m * 1000)
            accepted = True
            message = (f"Candidate {attempt.identifier}: trickle descent {extra:g} mm "
                       f"after {prior} completed attempts; "
                       f"pick Z={height.surface_z_m * 1000:.3f} mm")
        if self.trickle_details.get(index) != details:
            self.repick.events.record("INFO", "trickle_target" if accepted else "trickle_rejected",
                                      message, **details)
            self.trickle_details[index] = details
        return accepted

    @property
    def trickle_exhausted(self):
        return self.next_eligible is None and bool(self.trickle_rejections)

    def blocked(self, index):
        if self.repick is None:
            return False
        return self.repick.blocked(self.batch.candidates[index - 1].position_m) is not None

    @property
    def blacklist_only(self):
        return self.next_eligible is None and any(
            attempt.state in ("PENDING", "INTERRUPTED") and self.blocked(index)
            for index, attempt in enumerate(self.attempts, 1))

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
