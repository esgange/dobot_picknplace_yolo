"""Dobot transport used only by the headless hardware-authority node."""

import json
import math
import re
import threading
import time

import numpy as np

from .errors import (CommandRejected, CommandResponseTimeout, FeedbackFailure,
                     HeldUnknown, OperationCanceled, StopUnconfirmed)
from .feedback import enabled_blockers
from .kinematics import pose_matrix, pose_values
from .motion import pose_reached


SERVICE_DISCOVERY_TIMEOUT_SEC = 5.0
COMMAND_RESPONSE_TIMEOUT_SEC = 5.0
MODE_TRANSITION_TIMEOUT_SEC = 2.0
READY_STABLE_SEC = 0.2
CONSISTENT_FLAG_SAMPLES = 3
STATIONARY_SEC = 0.3
MOTION_NO_PROGRESS_SEC = 3.0
MOTION_HARD_CAP_SEC = 300.0
CARTESIAN_POSITION_TOLERANCE_M = 0.005
CARTESIAN_ORIENTATION_TOLERANCE_DEG = 1.0
HOME_JOINT_TOLERANCE_RAD = math.radians(1.0)
MOTION_SERVICES = ("MovL", "MovLIO", "RelMovLUser")
LATE_RESPONSE_OUTCOMES = ("timeout", "wait_canceled", "wait_aborted")


def robot_values(raw):
    match = re.fullmatch(r"\{([^{}]+)\}", raw) if isinstance(raw, str) else None
    if match is None:
        raise CommandRejected("Malformed canonical GetPose reply")
    try:
        values = [float(value) for value in match.group(1).split(",")]
    except ValueError as exc:
        raise CommandRejected("Malformed canonical GetPose values") from exc
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise CommandRejected("GetPose must return six finite values")
    return values


class DobotTransport:
    """Serialized normal commands plus an independent pre-emptive Stop path."""

    def __init__(self, node, monitor):
        from dobot_msgs_v4.srv import (CP, ClearError, Continue, DO, DisableRobot,
                                       EnableRobot, GetPose, MovL, MovLIO, Pause,
                                       RelMovLUser, SetTool, SpeedFactor, Stop,
                                       StopMoveJog, Tool, User)
        self.node = node
        self.monitor = monitor
        kinds = (CP, ClearError, DO, DisableRobot, EnableRobot, GetPose, MovL, MovLIO,
                 RelMovLUser, SetTool, SpeedFactor, StopMoveJog, Tool, User)
        self.types = {kind.__name__: kind for kind in kinds}
        self.clients = {
            name: node.create_client(kind, f"/dobot_bringup_ros2/srv/{name}")
            for name, kind in self.types.items()
        }
        self.queue_types = {kind.__name__: kind for kind in (Pause, Continue)}
        self.queue_clients = {
            name: node.create_client(kind, f"/dobot_bringup_ros2/srv/{name}")
            for name, kind in self.queue_types.items()
        }
        self.stop_type = Stop
        self.stop_client = node.create_client(Stop, "/dobot_bringup_ros2/srv/Stop")
        # rclpy may invoke a callback inline when a future is already complete.
        self.response_lock = threading.RLock()
        self.stop_lock = threading.Lock()
        self.service_audit_lock = threading.Lock()
        self.service_audit_sequence = 0
        self.pending_response = None
        self.stop_future = None
        self.stop_audit = None
        self.moving = False
        self.suction_interrupted = False
        self.suction_stop_future = None

    @property
    def required_services(self):
        return tuple(self.clients) + tuple(self.queue_clients) + ("Stop",)

    def close(self):
        for client in self.clients.values():
            self.node.destroy_client(client)
        for client in self.queue_clients.values():
            self.node.destroy_client(client)
        self.node.destroy_client(self.stop_client)
        self.clients.clear()
        self.queue_clients.clear()

    def wait_services(self, *, optional=()):
        deadline = time.monotonic() + SERVICE_DISCOVERY_TIMEOUT_SEC
        while True:
            missing = [name for name, client in self.clients.items()
                       if name not in optional and not client.service_is_ready()]
            missing.extend(
                name for name, client in self.queue_clients.items()
                if name not in optional and not client.service_is_ready())
            if not self.stop_client.service_is_ready():
                missing.append("Stop")
            if not missing:
                return
            self.node.raise_if_cancelled()
            if time.monotonic() >= deadline:
                raise CommandRejected(
                    "Required canonical services unavailable: " + ", ".join(missing))
            self.node.wait_control(0.05)

    def _console_service_log(self, level, message):
        logger_factory = getattr(self.node, "get_logger", None)
        if logger_factory is None:
            return
        logger = logger_factory()
        method = getattr(logger, "warning" if level == "WARNING" else level.lower())
        method(message)

    def _begin_service_audit(self, name, fields, *, reason=""):
        with self.service_audit_lock:
            self.service_audit_sequence += 1
            request_id = self.service_audit_sequence
        endpoint = f"/dobot_bringup_ros2/srv/{name}"
        audit = {
            "request_id": request_id,
            "name": name,
            "endpoint": endpoint,
            "fields": dict(fields),
            "started": time.monotonic(),
            "terminal": False,
        }
        details = json.dumps(fields, sort_keys=True, separators=(",", ":"))
        suffix = f" reason={reason}" if reason else ""
        self._console_service_log(
            "INFO", f"DOBOT SERVICE #{request_id} SEND {endpoint} request={details}{suffix}")
        event_fields = {
            "request_id": request_id, "endpoint": endpoint, "fields": dict(fields),
        }
        if reason:
            event_fields["reason"] = reason
        self.node.events.record("INFO", "robot_service_sent", name, **event_fields)
        return audit

    def _finish_service_audit(self, audit, outcome, *, result=None, detail="",
                              level=None, late=False):
        if audit is None or (audit.get("terminal") and not late):
            return
        if not late:
            audit["terminal"] = True
            audit["outcome"] = outcome
        level = level or ("INFO" if outcome in ("accepted", "late_response") else "ERROR")
        res = getattr(result, "res", None) if result is not None else None
        robot_return = getattr(result, "robot_return", None) if result is not None else None
        duration_ms = round((time.monotonic() - audit["started"]) * 1000.0, 3)
        terminal = (
            f"DOBOT SERVICE #{audit['request_id']} {outcome.upper()} "
            f"{audit['endpoint']} res={res} duration_ms={duration_ms:.3f}")
        if robot_return not in (None, ""):
            terminal += f" robot_return={robot_return!r}"
        if detail:
            terminal += f" detail={detail}"
        self._console_service_log(level, terminal)
        event = "robot_service" if outcome == "accepted" else (
            "robot_service_late_response" if outcome == "late_response"
            else "robot_service_failed")
        self.node.events.record(
            level, event, audit["name"], request_id=audit["request_id"],
            endpoint=audit["endpoint"], fields=audit["fields"], outcome=outcome,
            response_res=res, robot_return=robot_return, duration_ms=duration_ms,
            detail=detail)

    def _pending_completed(self, name, future, audit):
        with self.response_lock:
            if (self.pending_response is not None
                    and self.pending_response[0] == name
                    and self.pending_response[1] is future):
                self.pending_response = None
        if audit.get("outcome") in LATE_RESPONSE_OUTCOMES:
            try:
                result = future.result()
                detail = "response arrived after caller stopped waiting"
            except Exception as exc:  # rclpy future exception is terminal evidence.
                result = None
                detail = f"late response exception: {exc}"
            self._finish_service_audit(
                audit, "late_response", result=result, detail=detail,
                level="WARNING", late=True)
        if name in MOTION_SERVICES and self.node.cancel_requested():
            try:
                stop_future = self.request_stop("late motion acknowledgement")
                self.node.on_late_motion_ack(stop_future)
            except StopUnconfirmed as exc:
                self.node.events.record("ERROR", "late_ack_stop_failed", str(exc))
        elif name in MOTION_SERVICES and self.suction_interrupted:
            # Normal DI1 acquisition intentionally interrupts the descent.  The
            # owning pick thread confirms this Stop before queuing its retract.
            self.request_stop("motion acknowledgement after suction acquisition")

    def _late_service_completed(self, future, audit):
        if audit.get("outcome") not in LATE_RESPONSE_OUTCOMES:
            return
        try:
            result = future.result()
            detail = "response arrived after caller stopped waiting"
        except Exception as exc:
            result = None
            detail = f"late response exception: {exc}"
        self._finish_service_audit(
            audit, "late_response", result=result, detail=detail,
            level="WARNING", late=True)

    def ensure_no_pending_response(self):
        with self.response_lock:
            pending = self.pending_response
        if pending is not None and not pending[1].done():
            raise CommandResponseTimeout(
                f"Still awaiting {pending[0]} response; no later command may be sent")

    def call(self, name, *, check_cancel=True, progress=None, **fields):
        with self.response_lock:
            if self.pending_response is not None:
                previous, future, _audit = self.pending_response
                if not future.done():
                    raise CommandResponseTimeout(
                        f"{name} not sent: still awaiting {previous} response")
                self.pending_response = None
            self.node.check_command_owner(name)
            self.monitor.snapshot(require_enabled=False)
            client = self.clients[name]
            if not client.service_is_ready():
                raise CommandRejected(f"Required canonical {name} service unavailable")
            request = self.types[name].Request(**fields)
            audit = self._begin_service_audit(name, fields)
            try:
                future = client.call_async(request)
            except Exception as exc:
                self._finish_service_audit(
                    audit, "dispatch_error", detail=str(exc), level="ERROR")
                raise CommandRejected(f"{name} dispatch failed: {exc}") from exc
            self.pending_response = (name, future, audit)
            future.add_done_callback(
                lambda done, service=name, record=audit: self._pending_completed(
                    service, done, record))
        deadline = time.monotonic() + COMMAND_RESPONSE_TIMEOUT_SEC
        try:
            while not future.done():
                if check_cancel and self.node.cancel_requested():
                    self._finish_service_audit(
                        audit, "wait_canceled",
                        detail="operation canceled before response", level="WARNING")
                    if name in MOTION_SERVICES:
                        self.request_stop("operation cancellation during acknowledgement")
                    raise OperationCanceled(f"Cancelled while awaiting {name} response")
                snapshot = self.monitor.snapshot(require_enabled=False)
                if progress is not None:
                    progress(snapshot)
                if time.monotonic() >= deadline:
                    self._finish_service_audit(
                        audit, "timeout", detail="no response within 5 seconds",
                        level="ERROR")
                    raise CommandResponseTimeout(
                        f"{name} response timeout; no later normal command sent")
                self.node.wait_control(0.02)
            result = future.result()
        except (OperationCanceled, CommandResponseTimeout):
            raise
        except Exception as exc:
            self._finish_service_audit(
                audit, "wait_aborted", detail=str(exc), level="ERROR")
            raise
        if result is None or result.res != 0:
            self._finish_service_audit(
                audit, "rejected", result=result,
                detail="canonical service returned nonzero/empty result", level="ERROR")
            raise CommandRejected(f"{name} failed: {None if result is None else result.res}")
        self._finish_service_audit(audit, "accepted", result=result)
        if progress is not None:
            progress(self.monitor.snapshot(require_enabled=False))
        return result

    def request_stop(self, reason="operator/cancellation request"):
        with self.stop_lock:
            if self.stop_future is not None and not self.stop_future.done():
                return self.stop_future
            self.node.check_command_owner("Stop")
            if not self.stop_client.service_is_ready():
                raise StopUnconfirmed("Canonical Stop service unavailable")
            audit = self._begin_service_audit("Stop", {}, reason=reason)
            try:
                future = self.stop_client.call_async(self.stop_type.Request())
            except Exception as exc:
                self._finish_service_audit(
                    audit, "dispatch_error", detail=str(exc), level="ERROR")
                raise StopUnconfirmed(f"Stop dispatch failed: {exc}") from exc
            self.stop_future = future
            self.stop_audit = (future, audit)
            future.add_done_callback(
                lambda done, record=audit: self._late_service_completed(done, record))
            self.node.events.record("WARNING", "stop_requested", reason)
            return future

    def confirm_stop(self, future=None):
        future = future or self.stop_future
        if future is None:
            raise StopUnconfirmed("No Stop request exists to confirm")
        stop_audit = getattr(self, "stop_audit", None)
        audit = stop_audit[1] if stop_audit is not None and stop_audit[0] is future else None
        deadline = time.monotonic() + COMMAND_RESPONSE_TIMEOUT_SEC
        while not future.done():
            try:
                self.monitor.snapshot(require_enabled=False)
            except FeedbackFailure:
                pass
            if time.monotonic() >= deadline:
                self._finish_service_audit(
                    audit, "timeout", detail="Stop acknowledgement timeout", level="ERROR")
                raise StopUnconfirmed("Stop acknowledgement timeout")
            self.node.wait_control(0.02)
        try:
            result = future.result()
        except Exception as exc:
            self._finish_service_audit(
                audit, "response_error", detail=str(exc), level="ERROR")
            raise StopUnconfirmed(f"Stop response failed: {exc}") from exc
        if result is None or result.res != 0:
            self._finish_service_audit(
                audit, "rejected", result=result,
                detail="canonical Stop returned nonzero/empty result", level="ERROR")
            raise StopUnconfirmed(
                f"Stop rejected: {None if result is None else result.res}")
        self._finish_service_audit(audit, "accepted", result=result)
        anchor = None
        held_violation = None

        def stationary(snapshot):
            nonlocal anchor, held_violation
            feed = snapshot.feed
            if self.node.holding_item:
                if not feed["digital_input_bits"] & 1:
                    held_violation = "DI1 suction feedback was lost during Stop"
                for channel, active in self.node.expected_outputs.items():
                    actual = bool(feed["digital_outputs"] & (1 << (channel - 1)))
                    if actual != active:
                        held_violation = (
                            f"DO{channel} changed during Stop; expected {int(active)}")
            pose = np.asarray(feed["tool_vector_actual"], dtype=float)
            unmoved = anchor is not None and np.max(np.abs(pose - anchor)) <= 0.05
            anchor = pose
            return (not feed["isRunQueuedCmd"] and not feed["RunningStatus"]
                    and feed["robot_mode"] in (4, 5, 9, 10) and unmoved)
        try:
            self.monitor.wait(stationary, MODE_TRANSITION_TIMEOUT_SEC,
                              stable_sec=STATIONARY_SEC,
                              description="stationary, empty queue after Stop")
        except FeedbackFailure as exc:
            raise StopUnconfirmed(str(exc)) from exc
        self.moving = False
        if held_violation is not None:
            raise StopUnconfirmed(
                "Stop completed but held-item integrity was not preserved: "
                + held_violation)
        self.node.events.record("INFO", "stop_confirmed",
                                "Stop acknowledged; stationary empty queue confirmed")

    def _call_queue_control(self, name):
        self.node.check_command_owner(name)
        self.monitor.snapshot(require_enabled=False)
        client = self.queue_clients[name]
        if not client.service_is_ready():
            raise CommandRejected(f"Required canonical {name} service unavailable")
        audit = self._begin_service_audit(name, {})
        try:
            future = client.call_async(self.queue_types[name].Request())
        except Exception as exc:
            self._finish_service_audit(
                audit, "dispatch_error", detail=str(exc), level="ERROR")
            raise CommandRejected(f"{name} dispatch failed: {exc}") from exc
        future.add_done_callback(
            lambda done, record=audit: self._late_service_completed(done, record))
        deadline = time.monotonic() + COMMAND_RESPONSE_TIMEOUT_SEC
        try:
            while not future.done():
                self.monitor.snapshot(require_enabled=False)
                if time.monotonic() >= deadline:
                    self._finish_service_audit(
                        audit, "timeout", detail="queue state is ambiguous", level="ERROR")
                    raise CommandResponseTimeout(
                        f"{name} response timeout; queue state is ambiguous")
                self.node.wait_control(0.02)
            result = future.result()
        except CommandResponseTimeout:
            raise
        except Exception as exc:
            self._finish_service_audit(
                audit, "wait_aborted", detail=str(exc), level="ERROR")
            raise CommandRejected(f"{name} service wait failed: {exc}") from exc
        if result is None or result.res != 0:
            self._finish_service_audit(
                audit, "rejected", result=result,
                detail="canonical service returned nonzero/empty result", level="ERROR")
            raise CommandRejected(
                f"{name} failed: {None if result is None else result.res}")
        self._finish_service_audit(audit, "accepted", result=result)

    def pause_queue(self):
        self._call_queue_control("Pause")
        anchor = None

        def paused_and_stationary(snapshot):
            nonlocal anchor
            self._validate_held_snapshot(snapshot)
            pose = np.asarray(snapshot.feed["tool_vector_actual"], dtype=float)
            unmoved = anchor is not None and np.max(np.abs(pose - anchor)) <= 0.05
            anchor = pose
            return bool(snapshot.feed["isPauseCmdFlag"]) and unmoved

        return self.monitor.wait(
            paused_and_stationary, MODE_TRANSITION_TIMEOUT_SEC,
            require_enabled=True, allow_paused=True, stable_sec=STATIONARY_SEC,
            description="paused and stationary queue")

    def continue_queue(self):
        self._call_queue_control("Continue")
        return self.monitor.wait_samples(
            self._held_predicate(lambda sample: not sample.feed["isPauseCmdFlag"]),
            CONSISTENT_FLAG_SAMPLES, MODE_TRANSITION_TIMEOUT_SEC,
            require_enabled=True, description="continued queue")

    def _phase(self, phase, message, waypoint=""):
        self.node.operation_progress(phase, message, waypoint=waypoint)

    def _wait_for_resume(self):
        waiter = getattr(self.node, "wait_for_resume", None)
        if waiter is None:
            return 0.0
        started = time.monotonic()
        waiter()
        return time.monotonic() - started

    def _pause_requested(self):
        return getattr(self.node, "pause_requested", lambda: False)()

    def _ready_snapshot(self):
        while True:
            self._wait_for_resume()
            try:
                return self.monitor.snapshot(require_enabled=True)
            except FeedbackFailure:
                if not self._pause_requested():
                    raise

    def _validate_held_snapshot(self, snapshot, *, known_holding=None,
                                expected_outputs=None):
        holding = (self.node.holding_item
                   if known_holding is None else known_holding)
        outputs = (self.node.expected_outputs
                   if expected_outputs is None else expected_outputs)
        if not holding:
            return snapshot
        feed = snapshot.feed
        if not feed["digital_input_bits"] & 1:
            raise HeldUnknown("Known held-item context lost DI1 suction feedback")
        for channel, active in outputs.items():
            actual = bool(feed["digital_outputs"] & (1 << (channel - 1)))
            if actual != active:
                raise HeldUnknown(
                    f"Known held-item output DO{channel}={int(actual)}; "
                    f"expected {int(active)}")
        return snapshot

    def _held_predicate(self, predicate):
        def checked(snapshot):
            self._validate_held_snapshot(snapshot)
            return predicate(snapshot)
        return checked

    def _call_startup(self, name, **fields):
        self._phase("STARTUP_COMMAND", f"{name}: waiting for response", name)
        progress = self._validate_held_snapshot if self.node.holding_item else None
        return self.call(name, progress=progress, **fields)

    @staticmethod
    def _idle(snapshot):
        feed = snapshot.feed
        return (snapshot.robot_enabled and feed["robot_mode"] == 5
                and feed["EnableStatus"] == 1
                and not feed["isRunQueuedCmd"] and not feed["RunningStatus"]
                and not feed["ErrorStatus"] and not feed["CollisionStates"]
                and feed["userCoordinate"] == 0 and feed["toolCoordinate"] == 0)

    def _wait_enabled(self):
        self._phase("MODE_CONFIRM", "Confirming Enabled mode", "EnableRobot")
        return self.monitor.wait_samples(
            self._held_predicate(
                lambda sample: sample.feed["robot_mode"] == 5
                and sample.feed["EnableStatus"] == 1 and sample.robot_enabled),
            CONSISTENT_FLAG_SAMPLES, MODE_TRANSITION_TIMEOUT_SEC,
            cancel=self.node.cancel_requested, description="Enabled mode")

    def _check_held_context(self, known_holding, expected_outputs):
        snapshot = self.monitor.snapshot(require_enabled=False)
        suction = bool(snapshot.feed["digital_input_bits"] & 1)
        if suction and not known_holding:
            raise HeldUnknown(
                "DI1 is active without trusted controller-owned pickup context; outputs preserved")
        if known_holding:
            self._validate_held_snapshot(
                snapshot, known_holding=True, expected_outputs=expected_outputs)
        return snapshot

    def _clear_errors_if_needed(self):
        current = self._validate_held_snapshot(
            self.monitor.snapshot(require_enabled=False))
        feed = current.feed
        if not (feed["ErrorStatus"] or feed["CollisionStates"] or feed["robot_mode"] == 9):
            self.node.events.record("INFO", "clear_error_skipped", "No active robot alarm")
            return
        try:
            self.monitor.wait_samples(
                self._held_predicate(
                    lambda sample: sample.feed["robot_mode"] == 9
                    or bool(sample.feed["ErrorStatus"])
                    or bool(sample.feed["CollisionStates"])),
                CONSISTENT_FLAG_SAMPLES, 0.25,
                cancel=self.node.cancel_requested,
                description="persistent error/collision samples")
        except FeedbackFailure:
            current = self._validate_held_snapshot(
                self.monitor.snapshot(require_enabled=False)).feed
            if not (current["ErrorStatus"] or current["CollisionStates"]
                    or current["robot_mode"] == 9):
                self.node.events.record(
                    "WARNING", "transient_error_ignored",
                    "Error/collision did not persist for three feedback samples")
                return
            raise
        self._call_startup("ClearError")
        self.monitor.wait_samples(
            self._held_predicate(
                lambda sample: sample.feed["robot_mode"] != 9
                and not sample.feed["ErrorStatus"]
                and not sample.feed["CollisionStates"]),
            CONSISTENT_FLAG_SAMPLES, MODE_TRANSITION_TIMEOUT_SEC,
            cancel=self.node.cancel_requested, description="cleared error/collision feedback")

    def _apply_settings(self, speed_percent):
        for name, fields in (
                ("SpeedFactor", {"ratio": speed_percent}),
                ("User", {"index": 0}),
                ("Tool", {"index": 0}),
                ("SetTool", {"index": 1, "value": "{0,0,0,0,0,0}"}),
                ("CP", {"r": 100})):
            self._check_held_context(self.node.holding_item, self.node.expected_outputs)
            self._call_startup(name, **fields)
            if name == "SpeedFactor":
                self.node.global_speed_percent = speed_percent
                self.node.publish_status()

    def _confirm_ready(self):
        self._phase("READY_CONFIRM", "Confirming coherent enabled/idle feedback")
        try:
            return self.monitor.wait(
                self._held_predicate(self._idle), MODE_TRANSITION_TIMEOUT_SEC,
                cancel=self.node.cancel_requested, require_enabled=True,
                stable_sec=READY_STABLE_SEC, description="stable READY feedback")
        except FeedbackFailure as exc:
            try:
                snapshot = self.monitor.snapshot(require_enabled=False)
            except FeedbackFailure:
                raise exc
            blockers = enabled_blockers(snapshot.feed, snapshot.robot_enabled)
            feed = snapshot.feed
            if feed["robot_mode"] in (7, 8):
                blockers.append(
                    f"robot_mode={feed['robot_mode']} (required idle mode 5)")
            if feed["isRunQueuedCmd"]:
                blockers.append(f"isRunQueuedCmd={feed['isRunQueuedCmd']}")
            if feed["RunningStatus"]:
                blockers.append(f"RunningStatus={feed['RunningStatus']}")
            if blockers:
                raise FeedbackFailure(
                    "Stable READY blocked: " + "; ".join(blockers)) from exc
            raise FeedbackFailure(
                "READY fields were valid but did not remain coherent for 200 ms") from exc

    def startup(self):
        self.wait_services(optional=("StopMoveJog",))
        strict = tuple(name for name in self.required_services if name != "StopMoveJog")
        self.node.check_all_command_owners(strict)
        self.node.check_feedback_owners()
        self.monitor.snapshot(require_enabled=False)
        try:
            self._call_startup("StopMoveJog")
        except (CommandRejected, CommandResponseTimeout) as exc:
            self.node.events.record("WARNING", "startup_best_effort", str(exc),
                                    service="StopMoveJog")
            if isinstance(exc, CommandResponseTimeout):
                raise
        self._phase("QUEUE_RESET", "Stopping and discarding any queued motion", "Stop")
        self.confirm_stop(self.request_stop("explicit Startup queue reset"))
        self._check_held_context(False, {})
        self._call_startup("DisableRobot")
        self.monitor.wait_samples(
            lambda sample: sample.feed["robot_mode"] == 4
            and sample.feed["EnableStatus"] == 0 and not sample.robot_enabled,
            CONSISTENT_FLAG_SAMPLES, MODE_TRANSITION_TIMEOUT_SEC,
            cancel=self.node.cancel_requested, description="Disabled mode")
        self._clear_errors_if_needed()
        self._call_startup("EnableRobot")
        self._wait_enabled()
        self._apply_settings(100)
        self._reset_outputs_if_unheld()
        self._confirm_ready()

    def recover(self, speed_percent):
        self.wait_services(optional=("StopMoveJog",))
        strict = tuple(name for name in self.required_services if name != "StopMoveJog")
        self.node.check_all_command_owners(strict)
        self.node.check_feedback_owners()
        self.ensure_no_pending_response()
        self._phase("QUEUE_RESET", "Stopping and discarding any queued motion", "Stop")
        self.confirm_stop(self.request_stop("explicit Recover queue reset"))
        self._check_held_context(self.node.holding_item, self.node.expected_outputs)
        self._clear_errors_if_needed()
        self._call_startup("EnableRobot")
        self._wait_enabled()
        self._apply_settings(speed_percent if speed_percent is not None else 100)
        self._reset_outputs_if_unheld()
        self._confirm_ready()
        self._check_held_context(self.node.holding_item, self.node.expected_outputs)

    def _reset_outputs_if_unheld(self):
        if self.node.holding_item:
            return
        for channel in (1, 2, 13, 14):
            self._check_held_context(False, {})
            self._phase("OUTPUT_RESET", f"Resetting DO{channel}=0", f"DO{channel}")
            before = self.monitor.sequence
            self.call("DO", index=channel, status=0, time=0)
            mask = 1 << (channel - 1)
            self.monitor.wait(
                lambda sample, m=mask: sample.sequence > before
                and not bool(sample.feed["digital_outputs"] & m),
                COMMAND_RESPONSE_TIMEOUT_SEC, cancel=self.node.cancel_requested,
                require_enabled=False, description=f"startup DO{channel}=0 feedback")
            self.node.expected_outputs[channel] = False

    def set_global_speed(self, percent):
        if type(percent) is not int or not 1 <= percent <= 100:
            raise CommandRejected("Global speed must be an integer from 1 through 100")
        snapshot = self.monitor.snapshot(require_enabled=True)
        if not self._idle(snapshot) or self.moving:
            raise FeedbackFailure("SpeedFactor requires stationary READY/HOLDING feedback")
        self._check_held_context(self.node.holding_item, self.node.expected_outputs)
        progress = self._validate_held_snapshot if self.node.holding_item else None
        self.call("SpeedFactor", ratio=percent, progress=progress)
        # The successful service acknowledgement establishes the latest known
        # factor even if the following held-item integrity check faults.
        self.node.global_speed_percent = percent
        self.node.publish_status()
        self._check_held_context(self.node.holding_item, self.node.expected_outputs)

    def current_pose(self):
        snapshot = self._ready_snapshot()
        if not self._idle(snapshot):
            raise FeedbackFailure("Current-pose acquisition requires stationary READY feedback")
        result = self.call("GetPose", user=0, tool=0)
        return pose_matrix(robot_values(result.robot_return))

    def _target_values(self, target):
        matrix = target.matrix
        if (not isinstance(matrix, np.ndarray) or matrix.shape != (4, 4)
                or not np.issubdtype(matrix.dtype, np.number) or np.iscomplexobj(matrix)
                or not np.all(np.isfinite(matrix))
                or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
                or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3),
                                   atol=1e-6, rtol=0)
                or not math.isclose(float(np.linalg.det(matrix[:3, :3])), 1., abs_tol=1e-6)):
            raise CommandRejected(f"Invalid rigid target transform for {target.name}")
        values = pose_values(matrix)
        if target.joints_rad is not None:
            modeled = self.node.kinematics.forward(target.joints_rad)
            if not pose_reached(modeled, matrix, translation_m=0.002, rotation_deg=0.5):
                raise CommandRejected(f"Taught joint/FK mismatch for {target.name}")
        return values

    def _target_reached(self, target, snapshot):
        if target.joints_rad is not None:
            return max(abs(actual - expected) for actual, expected in zip(
                snapshot.joints, target.joints_rad)) <= HOME_JOINT_TOLERANCE_RAD
        return pose_reached(
            pose_matrix(snapshot.feed["tool_vector_actual"]), target.matrix,
            translation_m=CARTESIAN_POSITION_TOLERANCE_M,
            rotation_deg=CARTESIAN_ORIENTATION_TOLERANCE_DEG)

    def _monitor_motion_policy(self, snapshot, *, require_suction, forbid_suction,
                               stop_on_suction, before_suction):
        feed = snapshot.feed
        detected = bool(feed["digital_input_bits"] & 1)
        vacuum = bool(feed["digital_outputs"] & (1 << 12))
        if require_suction and not detected:
            raise FeedbackFailure("Suction lost during retract/Home")
        if require_suction:
            if not vacuum:
                raise FeedbackFailure("Suction output DO13 lost during retract/Home")
            for channel, active in self.node.expected_outputs.items():
                actual = bool(feed["digital_outputs"] & (1 << (channel - 1)))
                if actual != active:
                    raise FeedbackFailure(
                        f"Held-item output DO{channel} changed during retract/Home")
        if forbid_suction and detected:
            raise FeedbackFailure("Late DI1 after missed pickup; candidate retry forbidden")
        if stop_on_suction:
            if detected and not vacuum:
                raise FeedbackFailure("Unexpected DI1 before suction output became active")
            if detected and not self.suction_interrupted:
                self.suction_interrupted = True
                self.suction_stop_future = self.request_stop("DI1 acquired during final approach")
            if not vacuum and before_suction is not None:
                before_suction()

    def move_batch(self, targets, *, require_suction=False, forbid_suction=False,
                   stop_on_suction=False, before_suction=None):
        targets = tuple(targets)
        if not targets or sum((require_suction, forbid_suction, stop_on_suction)) > 1:
            raise CommandRejected("Invalid motion-batch suction policy or empty batch")
        start = self.current_pose()
        origin = start
        prepared = []
        expected_outputs = {}
        for target in targets:
            values = self._target_values(target)
            if target.relative_z and (
                    not np.allclose(origin[:2, 3], target.matrix[:2, 3], atol=1e-12, rtol=0)
                    or not np.allclose(origin[:3, :3], target.matrix[:3, :3],
                                       atol=1e-12, rtol=0)
                    or target.matrix[2, 3] < origin[2, 3]):
                raise CommandRejected("Relative Home-height target must rise at unchanged XY")
            for event in target.motion_io:
                expected_outputs[event.channel] = event.active
            prepared.append((target, values, origin.copy()))
            origin = target.matrix
        self.suction_interrupted = False
        self.suction_stop_future = None
        initial = self._ready_snapshot()
        before_sequence = initial.sequence
        self.moving = True
        started = time.monotonic()
        last_progress = started
        last_vector = np.asarray(initial.feed["tool_vector_actual"], dtype=float)

        def progress(snapshot):
            self._monitor_motion_policy(
                snapshot, require_suction=require_suction, forbid_suction=forbid_suction,
                stop_on_suction=stop_on_suction, before_suction=before_suction)

        try:
            for target, values, previous in prepared:
                self._wait_for_resume()
                self.node.raise_if_cancelled()
                progress(self._ready_snapshot())
                if self.suction_interrupted:
                    break
                self.node.operation_progress("MOTION", f"Dispatching {target.name}",
                                             waypoint=target.name)
                params = ["user=0", "tool=0", f"v={target.speed_percent}",
                          f"a={target.acceleration_percent}", "cp=0"]
                if target.relative_z:
                    self.call("RelMovLUser", a=0., b=0.,
                              c=(target.matrix[2, 3] - previous[2, 3]) * 1000,
                              d=0., e=0., f=0., param_value=params, progress=progress)
                else:
                    command = values if target.joints_rad is None else list(
                        np.rad2deg(target.joints_rad))
                    events = [event.vendor_value() for event in target.motion_io]
                    service = "MovLIO" if events else "MovL"
                    fields = dict(zip("abcdef", map(float, command)))
                    if events:
                        fields["mdis"] = events
                    self.call(service, mode=target.joints_rad is not None, **fields,
                              param_value=params, progress=progress)
                self.node.events.record(
                    "INFO", "motion_queued", target.name,
                    speed_percent=target.speed_percent,
                    acceleration_percent=target.acceleration_percent,
                    motion_io=[event.vendor_value() for event in target.motion_io])
            if self.suction_interrupted:
                self.confirm_stop(self.suction_stop_future)
                sample = self.monitor.snapshot(require_enabled=True)
                if (not sample.feed["digital_input_bits"] & 1
                        or not sample.feed["digital_outputs"] & (1 << 12)):
                    raise FeedbackFailure("Suction lost after acquisition Stop")
                for channel, active in expected_outputs.items():
                    actual = bool(sample.feed["digital_outputs"] & (1 << (channel - 1)))
                    if actual != active:
                        raise FeedbackFailure(
                            f"Motion-timed DO{channel} mismatch after suction Stop")
                self.node.expected_outputs.update(expected_outputs)
                return True
            tail = targets[-1]
            stable_since = None
            sequence = self.monitor.sequence
            while True:
                paused_for = self._wait_for_resume()
                started += paused_for
                last_progress += paused_for
                self.node.raise_if_cancelled()
                snapshot = self._ready_snapshot()
                progress(snapshot)
                now = time.monotonic()
                vector = np.asarray(snapshot.feed["tool_vector_actual"], dtype=float)
                if np.max(np.abs(vector - last_vector)) > 0.05:
                    last_progress, last_vector = now, vector
                reached = self._target_reached(tail, snapshot)
                idle = (snapshot.sequence > before_sequence and snapshot.robot_enabled
                        and reached
                        and not snapshot.feed["isRunQueuedCmd"]
                        and not snapshot.feed["RunningStatus"]
                        and snapshot.feed["robot_mode"] == 5)
                stable_since = now if idle and stable_since is None else (
                    stable_since if idle else None)
                if stable_since is not None and now - stable_since >= STATIONARY_SEC:
                    break
                if now - last_progress >= MOTION_NO_PROGRESS_SEC:
                    raise FeedbackFailure("Motion made no measurable progress for three seconds")
                if now - started >= MOTION_HARD_CAP_SEC:
                    raise FeedbackFailure("Motion exceeded the 300-second hard deadline")
                sequence = self.monitor.wait_next(
                    sequence, min(1.0, MOTION_HARD_CAP_SEC - (now - started)),
                    cancel=self.node.cancel_requested)
            if expected_outputs:
                self.monitor.wait(
                    lambda sample: all(
                        bool(sample.feed["digital_outputs"] & (1 << (channel - 1))) == active
                        for channel, active in expected_outputs.items()),
                    COMMAND_RESPONSE_TIMEOUT_SEC, cancel=self.node.cancel_requested,
                    pause=self._pause_requested, require_enabled=True,
                    description="motion-timed output feedback")
                self.node.expected_outputs.update(expected_outputs)
            self.node.events.record("INFO", "motion_batch_completed", tail.name,
                                    targets=[target.name for target in targets])
            return False
        except OperationCanceled:
            self.request_stop("motion operation cancelled")
            raise
        finally:
            self.moving = False

    def move(self, target, *, require_suction=False):
        return self.move_batch((target,), require_suction=require_suction)

    def output(self, channel, active, *, require_clear=False):
        snapshot = self._ready_snapshot()
        if require_clear and snapshot.feed["digital_input_bits"] & 1:
            raise FeedbackFailure("Unexpected DI1 before output change")
        before = snapshot.sequence
        self.call("DO", index=channel, status=int(active), time=0)
        mask = 1 << (channel - 1)
        self.monitor.wait(
            lambda sample: sample.sequence > before
            and bool(sample.feed["digital_outputs"] & mask) == active,
            COMMAND_RESPONSE_TIMEOUT_SEC, cancel=self.node.cancel_requested,
            pause=self._pause_requested, require_enabled=True,
            description=f"DO{channel} output feedback")
        self.node.expected_outputs[channel] = active

    def sensor(self, active, timeout, *, settling_sec):
        self._wait_for_resume()

        def expected(sample):
            return bool(sample.feed["digital_input_bits"] & 1) == active
        try:
            self.monitor.wait(expected, timeout, cancel=self.node.cancel_requested,
                              pause=self._pause_requested, require_enabled=True,
                              stable_sec=settling_sec,
                              description=f"DI1={int(active)}")
            return True
        except FeedbackFailure as exc:
            # A coherent, fresh opposite state is an ordinary missed-suction
            # result.  Stale/malformed feedback is never converted into a miss.
            try:
                snapshot = self._ready_snapshot()
            except FeedbackFailure:
                raise exc
            actual = bool(snapshot.feed["digital_input_bits"] & 1)
            if actual != active:
                return False
            raise exc
