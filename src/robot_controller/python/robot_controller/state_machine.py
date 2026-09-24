"""Small, strict state machine for the robot-controller lifecycle."""

from dataclasses import dataclass
import threading


STATES = (
    "UNCONFIGURED", "INACTIVE", "STARTING", "READY", "HOMING", "PICKING",
    "HOLDING", "PAUSED", "STOPPING", "RECOVERY_REQUIRED", "RECOVERING",
    "HELD_UNKNOWN", "FAULT", "PAUSING", "RETURNING_ITEM", "CALIBRATING",
)


_TRANSITIONS = {
    "UNCONFIGURED": {"INACTIVE", "STOPPING", "FAULT"},
    "INACTIVE": {"STARTING", "UNCONFIGURED", "STOPPING", "FAULT"},
    "STARTING": {"READY", "HELD_UNKNOWN", "STOPPING", "FAULT"},
    "READY": {"INACTIVE", "HOMING", "PICKING", "PAUSED", "STOPPING", "RECOVERING",
              "HELD_UNKNOWN", "FAULT"},
    "HOMING": {
        "READY", "HOLDING", "PAUSED", "STOPPING", "RECOVERY_REQUIRED",
        "HELD_UNKNOWN", "FAULT"},
    "PICKING": {
        "READY", "HOLDING", "PAUSED", "STOPPING", "RECOVERY_REQUIRED",
        "HELD_UNKNOWN", "FAULT", "RETURNING_ITEM"},
    "HOLDING": {"HOMING", "PAUSED", "STOPPING", "RECOVERING", "HELD_UNKNOWN",
                "FAULT"},
    "PAUSED": {"READY", "HOMING", "PICKING", "HOLDING", "STOPPING", "FAULT"},
    "STOPPING": {"UNCONFIGURED", "INACTIVE", "RECOVERY_REQUIRED", "HELD_UNKNOWN", "FAULT"},
    "RECOVERY_REQUIRED": {"RECOVERING", "STOPPING", "FAULT"},
    "RECOVERING": {"READY", "HOLDING", "HELD_UNKNOWN", "STOPPING", "FAULT", "RETURNING_ITEM"},
    "HELD_UNKNOWN": {"STOPPING", "RECOVERY_REQUIRED", "RECOVERING", "FAULT"},
    "FAULT": {"RECOVERING", "STOPPING"},
}

# Managed parking owns the operation executor until Continue or a direct Stop.
for _state in ("READY", "HOLDING", "HOMING", "PICKING", "PAUSED"):
    _TRANSITIONS[_state].add("PAUSING")
_TRANSITIONS["PAUSING"] = {"PAUSED", "RETURNING_ITEM", "STOPPING", "FAULT"}
_TRANSITIONS["PAUSED"].add("RETURNING_ITEM")
_TRANSITIONS["RETURNING_ITEM"] = {"PAUSED", "READY", "PICKING", "STOPPING", "FAULT"}
for _state in ("UNCONFIGURED", "INACTIVE", "READY"):
    _TRANSITIONS[_state].add("CALIBRATING")
_TRANSITIONS["CALIBRATING"] = {"UNCONFIGURED", "INACTIVE", "READY", "STOPPING", "FAULT"}


@dataclass(frozen=True)
class StateSnapshot:
    state: str
    message: str


class ControllerStateMachine:
    def __init__(self, initial="UNCONFIGURED", message="No configuration loaded", on_change=None):
        if initial not in STATES:
            raise ValueError(f"Unknown controller state: {initial}")
        self._state = initial
        self._message = message
        self._lock = threading.RLock()
        self._on_change = on_change

    @property
    def state(self):
        with self._lock:
            return self._state

    @property
    def message(self):
        with self._lock:
            return self._message

    def snapshot(self):
        with self._lock:
            return StateSnapshot(self._state, self._message)

    def update(self, message):
        with self._lock:
            self._message = str(message)
            snapshot = StateSnapshot(self._state, self._message)
        if self._on_change is not None:
            self._on_change(snapshot)
        return snapshot

    def transition(self, target, message):
        if target not in STATES:
            raise ValueError(f"Unknown controller state: {target}")
        with self._lock:
            if target != self._state and target not in _TRANSITIONS[self._state]:
                raise ValueError(f"Illegal controller transition {self._state} -> {target}")
            self._state = target
            self._message = str(message)
            snapshot = StateSnapshot(self._state, self._message)
        if self._on_change is not None:
            self._on_change(snapshot)
        return snapshot


def legal_targets(state):
    if state not in _TRANSITIONS:
        raise ValueError(f"Unknown controller state: {state}")
    return frozenset(_TRANSITIONS[state])
