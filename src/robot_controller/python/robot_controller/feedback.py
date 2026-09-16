"""Condition-driven validation of canonical Dobot feedback streams."""

from collections import deque
from dataclasses import dataclass
import json
import math
import threading
import time

from .errors import FeedbackFailure, OperationCanceled


FEEDBACK_MAX_AGE_SEC = 1.0
REQUIRED_FEED_KEYS = (
    "robot_mode", "digital_input_bits", "digital_outputs", "controller_timer",
    "isRunQueuedCmd", "RunningStatus", "ErrorStatus", "CollisionStates",
    "isPauseCmdFlag", "userCoordinate", "toolCoordinate", "EnableStatus",
)


@dataclass(frozen=True)
class FeedbackSnapshot:
    feed: dict
    joints: tuple
    robot_connected: bool
    robot_enabled: bool
    sequence: int
    received_at: float


class FeedbackMonitor:
    def __init__(self, ros_now_ns, monotonic=time.monotonic):
        self._ros_now_ns = ros_now_ns
        self._monotonic = monotonic
        self._condition = threading.Condition(threading.RLock())
        self._joints = None
        self._status = None
        self._feed = None
        self._sequence = 0
        self._controller_timer = None
        self._controller_progress_at = None
        self._flags = deque(maxlen=16)

    @property
    def sequence(self):
        with self._condition:
            return self._sequence

    def update_joints(self, message):
        names = tuple(message.name)
        values = tuple(message.position)
        if len(names) != 6 or set(names) != {f"joint{i}" for i in range(1, 7)}:
            raise FeedbackFailure("Actual joints must be exactly joint1 through joint6")
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise FeedbackFailure("Actual joints require six finite radians")
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if stamp <= 0 or not 0 <= message.header.stamp.nanosec < 1_000_000_000:
            raise FeedbackFailure("Actual joints require a nonzero source timestamp")
        ordered = tuple(float(values[names.index(f"joint{i}")]) for i in range(1, 7))
        with self._condition:
            self._joints = ordered, stamp, self._monotonic()
            self._condition.notify_all()

    def update_status(self, message):
        with self._condition:
            self._status = bool(message.is_connected), bool(message.is_enable), self._monotonic()
            self._condition.notify_all()

    def update_feed(self, raw):
        try:
            feed = json.loads(raw) if isinstance(raw, str) else dict(raw)
            for key in REQUIRED_FEED_KEYS:
                if type(feed[key]) is not int or feed[key] < 0:
                    raise ValueError(f"Canonical FeedInfo {key} must be a nonnegative integer")
            for key in ("tool_vector_actual", "q_actual", "qd_actual", "TCP_speed_actual"):
                values = feed[key]
                if (len(values) != 6 or not all(
                        type(value) in (int, float) and math.isfinite(value)
                        for value in values)):
                    raise ValueError(f"Canonical FeedInfo {key} must have six finite values")
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise FeedbackFailure(str(exc)) from exc
        now = self._monotonic()
        with self._condition:
            if feed["controller_timer"] != self._controller_timer:
                self._controller_timer = feed["controller_timer"]
                self._controller_progress_at = now
            self._sequence += 1
            self._feed = feed, now
            self._flags.append((self._sequence, feed["isPauseCmdFlag"],
                                feed["ErrorStatus"], feed["CollisionStates"]))
            self._condition.notify_all()

    def snapshot(self, *, require_enabled=False, allow_paused=False):
        with self._condition:
            joints, status, feedback = self._joints, self._status, self._feed
            sequence, progress = self._sequence, self._controller_progress_at
        now = self._monotonic()
        if joints is None or status is None or feedback is None:
            raise FeedbackFailure("Canonical robot feedback is incomplete")
        values, source_stamp, joint_received = joints
        connected, status_enabled, status_received = status
        feed, feed_received = feedback
        source_age = (self._ros_now_ns() - source_stamp) / 1e9
        if (not connected or not 0 <= source_age <= FEEDBACK_MAX_AGE_SEC
                or now - joint_received > FEEDBACK_MAX_AGE_SEC
                or now - status_received > FEEDBACK_MAX_AGE_SEC
                or now - feed_received > FEEDBACK_MAX_AGE_SEC
                or progress is None or now - progress > FEEDBACK_MAX_AGE_SEC):
            raise FeedbackFailure("Canonical robot connection/feedback is unavailable or stale")
        if require_enabled:
            blockers = enabled_blockers(feed, status_enabled, allow_paused=allow_paused)
            if blockers:
                raise FeedbackFailure("Robot readiness blocked: " + "; ".join(blockers))
        return FeedbackSnapshot(feed, values, connected, status_enabled, sequence, feed_received)

    def consistent_flags(self, count=3):
        with self._condition:
            samples = list(self._flags)[-count:]
        if len(samples) < count:
            return None
        values = [sample[1:] for sample in samples]
        return values[0] if all(value == values[0] for value in values) else None

    def wait(self, predicate, timeout, *, cancel=None, pause=None,
             require_enabled=False, allow_paused=False, stable_sec=0.0,
             description="feedback condition"):
        deadline = self._monotonic() + timeout
        stable_since = None
        observed = self.sequence
        evaluated_sequence = None
        while True:
            if cancel is not None and cancel():
                raise OperationCanceled("Controller operation cancelled")
            if pause is not None and pause():
                paused_at = self._monotonic()
                while pause():
                    if cancel is not None and cancel():
                        raise OperationCanceled("Controller operation cancelled")
                    self.snapshot(require_enabled=True, allow_paused=True)
                    with self._condition:
                        self._condition.wait(0.05)
                deadline += self._monotonic() - paused_at
                stable_since = None
                observed = self.sequence
                continue
            try:
                snapshot = self.snapshot(
                    require_enabled=require_enabled, allow_paused=allow_paused)
                new_sample = snapshot.sequence != evaluated_sequence
                matched = bool(predicate(snapshot)) if new_sample or stable_sec == 0 else None
            except FeedbackFailure:
                matched = False
                snapshot = None
            now = self._monotonic()
            if matched is not None:
                if snapshot is not None:
                    evaluated_sequence = snapshot.sequence
                if matched:
                    stable_since = now if stable_since is None else stable_since
                    if now - stable_since >= stable_sec:
                        return snapshot
                else:
                    stable_since = None
            remaining = deadline - now
            if remaining <= 0:
                raise FeedbackFailure(f"Timed out waiting for {description}")
            with self._condition:
                if self._sequence == observed:
                    self._condition.wait(min(remaining, 0.1))
                observed = self._sequence

    def wait_samples(self, predicate, count, timeout, *, cancel=None, pause=None,
                     require_enabled=False, allow_paused=False,
                     description="consistent feedback"):
        deadline = self._monotonic() + timeout
        matched = 0
        observed = self.sequence
        while True:
            if cancel is not None and cancel():
                raise OperationCanceled("Controller operation cancelled")
            if pause is not None and pause():
                paused_at = self._monotonic()
                while pause():
                    if cancel is not None and cancel():
                        raise OperationCanceled("Controller operation cancelled")
                    self.snapshot(require_enabled=True, allow_paused=True)
                    with self._condition:
                        self._condition.wait(0.05)
                deadline += self._monotonic() - paused_at
                matched = 0
                observed = self.sequence
                continue
            try:
                snapshot = self.snapshot(
                    require_enabled=require_enabled, allow_paused=allow_paused)
            except FeedbackFailure:
                snapshot = None
            with self._condition:
                current = self._sequence
            if current != observed and snapshot is not None:
                matched = matched + 1 if predicate(snapshot) else 0
                if matched >= count:
                    return snapshot
                observed = current
            remaining = deadline - self._monotonic()
            if remaining <= 0:
                raise FeedbackFailure(f"Timed out waiting for {description}")
            with self._condition:
                if self._sequence == observed:
                    self._condition.wait(min(remaining, 0.1))

    def wait_next(self, sequence, timeout, *, cancel=None):
        deadline = self._monotonic() + timeout
        with self._condition:
            while self._sequence <= sequence:
                if cancel is not None and cancel():
                    raise OperationCanceled("Controller operation cancelled")
                remaining = deadline - self._monotonic()
                if remaining <= 0:
                    raise FeedbackFailure("Timed out waiting for advancing robot feedback")
                self._condition.wait(min(remaining, 0.1))
            return self._sequence


def enabled_blockers(feed, _status_enabled, *, allow_paused=False):
    """Return authoritative FeedInfo blockers for an enabled operation.

    The vendored bridge publishes ``RobotStatus.is_enable`` as the expression
    ``robot_mode == 5``.  It is therefore an idle-mode alias, not an independent
    enabled latch, and its 10 Hz publication can legitimately disagree with the
    newer FeedInfo sample while entering or leaving motion.  RobotStatus is
    still required and freshness/connection checked by ``snapshot``; active
    readiness comes from FeedInfo EnableStatus, robot_mode, and fault fields.
    Callers that require coherent idle mode additionally check is_enable.
    """
    blockers = []
    if feed["EnableStatus"] != 1:
        blockers.append(f"EnableStatus={feed['EnableStatus']} (required 1)")
    enabled_modes = (5, 7, 8, 10) if allow_paused else (5, 7, 8)
    if feed["robot_mode"] not in enabled_modes:
        expected = "5/7/8/10" if allow_paused else "5/7/8"
        blockers.append(f"robot_mode={feed['robot_mode']} (expected {expected})")
    for key in ("ErrorStatus", "CollisionStates"):
        if feed[key]:
            blockers.append(f"{key}={feed[key]}")
    for key in ("userCoordinate", "toolCoordinate"):
        if feed[key] != 0:
            blockers.append(f"nonzero user/tool: {key}={feed[key]} (required 0)")
    return blockers
