"""Dobot transport used only by the headless hardware-authority node."""

import json
import math
import re
import threading
import time
from contextlib import nullcontext
from dataclasses import replace

import numpy as np

from .errors import (CommandRejected, CommandResponseTimeout, FeedbackFailure,
                     HeldSuctionLost, HeldUnknown, ManagedInterruption,
                     OperationCanceled, StopUnconfirmed,
                     UNKNOWN_ITEM_GUIDANCE, EmergencyStopPressed, EMERGENCY_STOP_GUIDANCE,
                     alarm_ids, command_failure_message, command_rejection)
from .feedback import SUCTION_LOSS_DEBOUNCE_SEC, enabled_blockers
from .kinematics import pose_values
from .motion import CARTESIAN_POSITION_TOLERANCE_M, pose_reached
from .recovery import GRIP_MASK


SERVICE_DISCOVERY_TIMEOUT_SEC = 5.0
COMMAND_RESPONSE_TIMEOUT_SEC = 5.0
OUTPUT_FEEDBACK_TIMEOUT_SEC = 5.0
MODE_TRANSITION_TIMEOUT_SEC = 2.0
READY_STABLE_SEC = 0.2
CONSISTENT_FLAG_SAMPLES = 3
MOTION_NO_PROGRESS_SEC = 3.0
MOTION_HARD_CAP_SEC = 300.0
CARTESIAN_ORIENTATION_TOLERANCE_DEG = 1.0
HOME_JOINT_TOLERANCE_RAD = math.radians(1.0)
MOTION_SERVICES = ("MovL", "MovLIO", "RelMovLUser")
LATE_RESPONSE_OUTCOMES = ("timeout", "wait_canceled", "wait_aborted")


def retained_release_progress(node):
    placement = getattr(node, "placement", None)
    if placement is not None and placement.needs_recovery:
        return placement
    return getattr(getattr(node, "managed", None), "return_progress", None)


class DobotTransport:
    """Serialized normal commands plus an independent pre-emptive Stop path."""

    def __init__(self, node, monitor):
        from dobot_msgs_v4.srv import (CP, ClearError, DO, DisableRobot,
                                       EnableRobot, GetErrorID, MovL, MovLIO,
                                       RelMovLUser, SetTool, SpeedFactor, Stop,
                                       StopMoveJog, Tool, User)
        self.node = node
        self.monitor = monitor
        kinds = (CP, ClearError, DO, DisableRobot, EnableRobot, GetErrorID, MovL, MovLIO,
                 RelMovLUser, SetTool, SpeedFactor, StopMoveJog, Tool, User)
        self.types = {kind.__name__: kind for kind in kinds}
        self.clients = {
            name: node.create_client(kind, f"/dobot_bringup_ros2/srv/{name}")
            for name, kind in self.types.items()
        }
        self.stop_type = Stop
        self.stop_client = node.create_client(Stop, "/dobot_bringup_ros2/srv/Stop")
        # rclpy may invoke a callback inline when a future is already complete.
        self.response_lock = threading.RLock()
        self.stop_lock = threading.Lock()
        self.service_audit_lock = threading.Lock()
        self.service_audit_sequence = 0
        self.pending_response = None
        self.pending_group = None
        self.pending_motion_outputs = {}
        self.stop_future = None
        self.stop_audit = None
        self.moving = False
        self.suction_interrupted = False
        self.suction_stop_future = None

    @property
    def required_services(self):
        return tuple(self.clients) + ("Stop",)

    def close(self):
        for client in self.clients.values():
            self.node.destroy_client(client)
        self.node.destroy_client(self.stop_client)
        self.clients.clear()

    def wait_services(self, *, optional=()):
        deadline = time.monotonic() + SERVICE_DISCOVERY_TIMEOUT_SEC
        while True:
            missing = [name for name, client in self.clients.items()
                       if name not in optional and not client.service_is_ready()]
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
        if logger_factory is not None:
            logger = logger_factory()
            # rclpy fixes severity by caller location. Keep one source line per
            # severity so a later failure cannot mask the real Stop response.
            if level == "ERROR":
                logger.error(message)
            elif level == "WARNING":
                logger.warning(message)
            else:
                logger.info(message)
        operator_log = getattr(self.node, "publish_operator_log", None)
        if operator_log is not None:
            operator_log(level, message)

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
        self._completed_response(name, future, audit)

    def _group_completed(self, name, future, audit):
        # call_group deliberately retains response_lock until the complete
        # ordered group is admitted. A group completion callback must not wait
        # for that lock: doing so can occupy every executor worker and prevent
        # later service responses and feedback subscriptions from running.
        self._completed_response(name, future, audit)

    def _completed_response(self, name, future, audit):
        late_response = audit.get("outcome") in LATE_RESPONSE_OUTCOMES
        if late_response:
            try:
                result = future.result()
                detail = "response arrived after caller stopped waiting"
            except Exception as exc:  # rclpy future exception is terminal evidence.
                result = None
                detail = f"late response exception: {exc}"
            self._finish_service_audit(
                audit, "late_response", result=result, detail=detail,
                level="WARNING", late=True)
        if name in MOTION_SERVICES and (late_response or self.node.cancel_requested()):
            try:
                reason = ("late motion acknowledgement"
                          if late_response else "motion acknowledgement after cancellation")
                stop_future = self.request_stop(reason)
                self.node.on_late_motion_ack(stop_future)
            except StopUnconfirmed as exc:
                self.node.events.record("ERROR", "late_ack_stop_failed", str(exc))
        # Normal pickup interruption is serialized by the owning motion group.
        # A delayed callback must not send another Stop into its return queue.

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
            group = getattr(self, "pending_group", None)
        if pending is not None and not pending[1].done():
            raise CommandResponseTimeout(
                f"Still awaiting {pending[0]} response; no later command may be sent")
        if group is not None:
            unfinished = [name for name, future, _audit in group if not future.done()]
            if unfinished:
                raise CommandResponseTimeout(
                    "Still awaiting motion-group responses: " + ", ".join(unfinished))
            with self.response_lock:
                if self.pending_group is group:
                    self.pending_group = None

    def resolve_interrupted_commands(self):
        """Drain and validate every issued reply before the final discard Stop."""
        with self.response_lock:
            pending = list(self.pending_group or ())
            if self.pending_response is not None:
                pending.append(self.pending_response)
        for name, future, audit in pending:
            deadline = audit["started"] + COMMAND_RESPONSE_TIMEOUT_SEC
            while not future.done():
                self.node.raise_if_cancelled()
                self.monitor.snapshot(require_enabled=False)
                if time.monotonic() >= deadline:
                    raise CommandResponseTimeout(
                        f"Unresolved interrupted {name}; automatic return blocked")
                self.node.wait_control(.02)
            result = future.result()
            if result is None or result.res != 0:
                raise command_rejection(name, result)
            self._finish_service_audit(audit, "accepted", result=result)
        self.ensure_no_pending_response()

    def _dispatch_normal(self, client, request, *, check_cancel=True, issuing=None):
        managed = getattr(self.node, "managed", None)
        with getattr(managed, "lock", nullcontext()):
            if check_cancel:
                self._wait_for_resume()
            if issuing is not None:
                issuing()
            return client.call_async(request)

    def call(self, name, *, check_cancel=True, progress=None, **fields):
        if check_cancel:
            self._wait_for_resume()
        with self.response_lock:
            group = getattr(self, "pending_group", None)
            if group is not None and any(not item[1].done() for item in group):
                raise CommandResponseTimeout(
                    f"{name} not sent: still awaiting motion-group responses")
            if group is not None:
                self.pending_group = None
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
                future = self._dispatch_normal(client, request, check_cancel=check_cancel)
            except (HeldSuctionLost, ManagedInterruption, OperationCanceled):
                raise
            except Exception as exc:
                self._finish_service_audit(
                    audit, "dispatch_error", detail=str(exc), level="ERROR")
                raise CommandRejected(f"{name} dispatch failed: {exc}") from exc
            self.pending_response = (name, future, audit)
            if name == "DO" and fields.get("time", 0) == 0:
                self.pending_motion_outputs[fields["index"]] = bool(fields["status"])
            return_progress = retained_release_progress(self.node)
            if (name == "DO" and return_progress is not None
                    and return_progress.phase == "RELEASING"):
                return_progress.pending_outputs[fields["index"]] = bool(fields["status"])
                if (fields["index"] == 1 and fields.get("time") == 50
                        and hasattr(return_progress, "pulse_dispatched")):
                    return_progress.pulse_dispatched = True
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
                        audit, "timeout",
                        detail=f"no response within {COMMAND_RESPONSE_TIMEOUT_SEC:g} seconds",
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
                detail=command_failure_message(name, result), level="ERROR")
            raise command_rejection(name, result)
        self._finish_service_audit(audit, "accepted", result=result)
        if progress is not None:
            progress(self.monitor.snapshot(require_enabled=False))
        return result

    def call_group(self, calls, *, progress=None, outputs_by_call=None, admitted=None,
                   issuing=None, accepted=None):
        """Admit motion in dashboard order, without intermediate arrival waits."""
        calls = tuple((name, dict(fields)) for name, fields in calls)
        if not calls or any(name not in MOTION_SERVICES for name, _fields in calls):
            raise CommandRejected("Motion group requires canonical motion services")
        if outputs_by_call is None:
            outputs_by_call = tuple({} for _call in calls)
        else:
            outputs_by_call = tuple(dict(outputs) for outputs in outputs_by_call)
        if len(outputs_by_call) != len(calls):
            raise CommandRejected("Motion group output plan must match its request count")
        if not hasattr(self, "pending_motion_outputs"):
            self.pending_motion_outputs = {}
        names = [name for name, _fields in calls]
        self.node.check_all_command_owners(names)
        self.monitor.snapshot(require_enabled=False)
        for name in dict.fromkeys(names):
            if not self.clients[name].service_is_ready():
                raise CommandRejected(f"Required canonical {name} service unavailable")

        with self.response_lock:
            pending = self.pending_response
            if pending is not None and not pending[1].done():
                raise CommandResponseTimeout(
                    f"Motion group not sent: still awaiting {pending[0]} response")
            group = getattr(self, "pending_group", None)
            if group is not None and any(not item[1].done() for item in group):
                raise CommandResponseTimeout(
                    "Motion group not sent: earlier group responses are pending")
            self.pending_response = None
            group = []
            self.pending_group = group
            results = []
            try:
                for (name, fields), planned_outputs in zip(calls, outputs_by_call):
                    self._wait_for_resume()
                    if self.node.cancel_requested():
                        raise OperationCanceled("Cancelled before motion-group dispatch")
                    if self.suction_interrupted:
                        break
                    audit = self._begin_service_audit(name, fields)
                    try:
                        future = self._dispatch_normal(
                            self.clients[name], self.types[name].Request(**fields),
                            issuing=(lambda: issuing(len(group))) if issuing is not None else None)
                    except (HeldSuctionLost, ManagedInterruption, OperationCanceled):
                        raise
                    except Exception as exc:
                        self._finish_service_audit(
                            audit, "dispatch_error", detail=str(exc), level="ERROR")
                        raise CommandRejected(f"{name} dispatch failed: {exc}") from exc
                    self.pending_motion_outputs.update(planned_outputs)
                    group.append((name, future, audit))
                    future.add_done_callback(
                        lambda done, service=name, record=audit: self._group_completed(
                            service, done, record))
                    # ROS services have independent callbacks. Dispatch order is
                    # not dashboard TCP order: actual hardware logs showed a
                    # joint Home command overtaking its earlier vertical rise.
                    # Require each queue-admission response before the next send.
                    deadline = audit["started"] + COMMAND_RESPONSE_TIMEOUT_SEC
                    suction_loss = None
                    while not future.done():
                        if self.node.cancel_requested():
                            self._finish_service_audit(
                                audit, "wait_canceled",
                                detail="operation canceled during motion acknowledgement",
                                level="WARNING")
                            raise OperationCanceled(
                                f"Cancelled while awaiting {name} response")
                        snapshot = self.monitor.snapshot(require_enabled=False)
                        if progress is not None:
                            try:
                                progress(snapshot)
                            except HeldSuctionLost as exc:
                                if suction_loss is None:
                                    self.request_stop("Held suction lost during motion admission")
                                    suction_loss = exc
                        if self.suction_interrupted:
                            # The immediate Stop may precede this command's
                            # admission. Discard again after both replies.
                            self.suction_stop_needs_refresh = True
                        if time.monotonic() >= deadline:
                            self._finish_service_audit(
                                audit, "timeout",
                                detail=("no response within "
                                        f"{COMMAND_RESPONSE_TIMEOUT_SEC:g} seconds"),
                                level="ERROR")
                            raise CommandResponseTimeout(
                                f"{name} response timeout; no later motion sent")
                        self.node.wait_control(0.02)
                    try:
                        result = future.result()
                    except Exception as exc:
                        self._finish_service_audit(
                            audit, "wait_aborted", detail=str(exc), level="ERROR")
                        raise CommandRejected(f"{name} response failed: {exc}") from exc
                    if result is None or result.res != 0:
                        self._finish_service_audit(
                            audit, "rejected", result=result,
                            detail=command_failure_message(name, result),
                            level="ERROR")
                        raise command_rejection(name, result)
                    if name == "MovL":
                        self._motion_command_id(result)
                    self._finish_service_audit(audit, "accepted", result=result)
                    results.append(result)
                    if accepted is not None:
                        accepted(len(results) - 1, result)
                    if admitted is not None:
                        admitted(len(results) - 1)
                    if suction_loss is not None:
                        # The accepted outstanding request cannot be mistaken for
                        # a late reply that cancels the owner's put-back routine.
                        # No subsequent command from this group may be sent.
                        raise suction_loss
                    if progress is not None:
                        progress(self.monitor.snapshot(require_enabled=False))
                    self._wait_for_resume()
            except Exception:
                for _name, _future, audit in group:
                    if "outcome" not in audit:
                        self._finish_service_audit(
                            audit, "wait_aborted",
                            detail="motion-group admission did not complete", level="ERROR")
                if not group or all(future.done() for _name, future, _audit in group):
                    self.pending_group = None
                self.request_stop("motion-group dispatch failure")
                raise
            if self.pending_group is group:
                self.pending_group = None
        if progress is not None:
            progress(self.monitor.snapshot(require_enabled=False))
        return tuple(results)

    def request_stop(self, reason="operator/cancellation request", *, fresh=False):
        with self.stop_lock:
            if not fresh and self.stop_future is not None and not self.stop_future.done():
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

    def _acknowledge_stop(self, future=None, *, check_cancel=False):
        """Validate command acceptance without asserting a physical standstill."""
        future = future or self.stop_future
        if future is None:
            raise StopUnconfirmed("No Stop request exists to confirm")
        stop_audit = getattr(self, "stop_audit", None)
        audit = stop_audit[1] if stop_audit is not None and stop_audit[0] is future else None
        deadline = time.monotonic() + COMMAND_RESPONSE_TIMEOUT_SEC
        while not future.done():
            if check_cancel:
                self.node.raise_if_cancelled()
            try:
                self.monitor.snapshot(require_enabled=False)
            except FeedbackFailure:
                pass
            if time.monotonic() >= deadline:
                self._finish_service_audit(
                    audit, "timeout", detail="Stop acknowledgement timeout", level="ERROR")
                raise StopUnconfirmed("Stop acknowledgement timeout")
            self.node.wait_control(0.02)
        if check_cancel:
            self.node.raise_if_cancelled()
        try:
            result = future.result()
        except Exception as exc:
            self._finish_service_audit(
                audit, "response_error", detail=str(exc), level="ERROR")
            raise StopUnconfirmed(f"Stop response failed: {exc}") from exc
        if result is None or result.res != 0:
            detail = (command_failure_message("Stop", result) if result is not None
                      and result.res == -3 else
                      f"Stop rejected: {None if result is None else result.res}")
            self._finish_service_audit(
                audit, "rejected", result=result,
                detail=detail, level="ERROR")
            raise StopUnconfirmed(detail)
        self._finish_service_audit(audit, "accepted", result=result)

    def confirm_stop(self, future=None, *, allow_suction_loss=False, recovery_home=None,
                     after_enable=False):
        self._acknowledge_stop(future, check_cancel=after_enable)
        anchor = None
        anchor_sequence = None
        anchor_io = None
        anchor_stopped = False
        held_violation = None
        suction_lost = False
        last_snapshot = None
        planned_outputs = dict(getattr(self, "pending_motion_outputs", {}))
        return_progress = retained_release_progress(self.node)
        if return_progress is not None:
            planned_outputs.update(return_progress.pending_outputs)
        pulse_pending = bool(return_progress is not None
                             and return_progress.pending_outputs.get(1) is True)
        placement = getattr(self.node, "placement", None)
        if (placement is not None and not placement.require_held_item
                and not placement.returning_to_bin):
            allow_suction_loss = True

        def stationary(snapshot):
            nonlocal anchor, anchor_sequence, anchor_io, anchor_stopped
            nonlocal held_violation, last_snapshot, suction_lost
            last_snapshot = snapshot
            feed = snapshot.feed
            if recovery_home is not None:
                # Explicit cancel-and-Home uses newly stopped I/O, never the
                # interrupted placement/put-back history. No outputs are sent.
                if after_enable:
                    try:
                        recovery_home.check(snapshot)
                    except (FeedbackFailure, HeldUnknown) as exc:
                        held_violation = str(exc)
            elif return_progress is not None and getattr(return_progress, "observing", False):
                try:
                    return_progress.observe(self.node, snapshot)
                except (FeedbackFailure, HeldUnknown) as exc:
                    held_violation = str(exc)
            elif placement is not None and placement.observing:
                try:
                    placement.observe(self.node, snapshot)
                except (FeedbackFailure, HeldUnknown) as exc:
                    held_violation = str(exc)
            elif (self.node.holding_item or getattr(self, "return_recovery", False)
                    or return_progress is not None):
                releasing = return_progress is not None and return_progress.phase != "APPROACH"
                if not snapshot.suction_present and not allow_suction_loss and not releasing:
                    suction_lost = True
                    managed = getattr(self.node, "managed", None)
                    if managed is not None:
                        managed.note_suction_loss(snapshot)
                for channel, active in self.node.expected_outputs.items():
                    actual = bool(feed["digital_outputs"] & (1 << (channel - 1)))
                    planned = planned_outputs.get(channel, active)
                    if actual not in (active, planned) and not (channel == 1 and pulse_pending):
                        held_violation = (
                            f"DO{channel} changed during Stop; expected {int(active)}")
            joints = np.asarray(snapshot.joints, dtype=float)
            unmoved = (anchor is not None and snapshot.joint_stamp_ns > anchor_sequence
                       and np.max(np.abs(joints - anchor)) <= math.radians(0.05))
            current_io = (feed["digital_outputs"] & GRIP_MASK, feed["digital_input_bits"] & 1)
            stopped = (not feed["isRunQueuedCmd"] and not feed["RunningStatus"]
                       and (snapshot.robot_enabled or feed["robot_mode"] in (4, 9, 10)))
            if recovery_home is not None:
                unmoved = unmoved and anchor_io == current_io and anchor_stopped
            anchor = joints
            anchor_sequence = snapshot.joint_stamp_ns
            anchor_io = current_io
            anchor_stopped = stopped
            return stopped and unmoved
        try:
            wait_options = ({"cancel": self.node.cancel_requested, "require_enabled": True}
                            if after_enable else {})
            endpoint = "EnableRobot" if after_enable else "Stop"
            self.monitor.wait(stationary, MODE_TRANSITION_TIMEOUT_SEC,
                              description=f"stationary joints and empty queue after {endpoint}",
                              **wait_options)
        except FeedbackFailure as exc:
            if held_violation is not None:
                raise StopUnconfirmed(held_violation) from exc
            raise StopUnconfirmed(str(exc)) from exc
        self.moving = False
        if held_violation is not None:
            raise StopUnconfirmed(
                "Stop completed but held-item integrity was not preserved: "
                + held_violation)
        if recovery_home is not None:
            self.pending_motion_outputs = {}
            self.node.events.record(
                "INFO", "stop_confirmed",
                "Stationary empty queue and current gripper I/O confirmed")
            if not after_enable:
                recovery_home.capture(self.node, last_snapshot)
            return
        if last_snapshot is not None:
            outputs = last_snapshot.feed["digital_outputs"]
            for channel, planned in planned_outputs.items():
                current = self.node.expected_outputs.get(channel, planned)
                actual = bool(outputs & (1 << (channel - 1)))
                if actual in (current, planned) or channel == 1 and pulse_pending:
                    self.node.expected_outputs[channel] = actual
        self.pending_motion_outputs = {}
        self.node.events.record("INFO", "stop_confirmed",
                                "Stop acknowledged; stationary empty queue confirmed")
        if suction_lost:
            self.node.events.record(
                "WARNING", "stop_suction_lost",
                "Stop confirmed; suction is unconfirmed, outputs preserved; "
                "explicit recovery required")

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
                snapshot = self.monitor.snapshot(require_enabled=True)
                observer = getattr(self.node, "observe_managed_feedback", None)
                if observer is not None:
                    observer(snapshot)
                self._wait_for_resume()
                return snapshot
            except FeedbackFailure:
                if not self._pause_requested():
                    raise

    def _validate_held_snapshot(self, snapshot, *, known_holding=None,
                                expected_outputs=None):
        recovery = getattr(self, "home_recovery", None)
        if recovery is not None:
            return recovery.check(snapshot)
        returning = getattr(self, "return_recovery", False)
        placement = getattr(self.node, "placement", None)
        manual_placement = (placement is not None and not placement.require_held_item
                            and not placement.returning_to_bin)
        holding = returning or (self.node.holding_item if known_holding is None else known_holding)
        outputs = (self.node.expected_outputs
                   if expected_outputs is None else expected_outputs)
        if not holding and not manual_placement:
            return snapshot
        feed = snapshot.feed
        progress = retained_release_progress(self.node)
        if returning and progress is not None and progress.phase == "RELEASED":
            if feed["digital_input_bits"] & 1:
                raise HeldUnknown("DI1 HIGH after confirmed release; clear item or obstruction")
        for channel, active in outputs.items():
            actual = bool(feed["digital_outputs"] & (1 << (channel - 1)))
            if actual != active:
                raise HeldUnknown(
                    f"Known held-item output DO{channel}={int(actual)}; "
                    f"expected {int(active)}")
        if not snapshot.suction_present and not returning and not manual_placement:
            managed = getattr(self.node, "managed", None)
            if managed is not None:
                managed.note_suction_loss(snapshot)
            raise HeldSuctionLost("Known held-item context lost DI1 suction feedback")
        return snapshot

    def _held_predicate(self, predicate):
        def checked(snapshot):
            self._validate_held_snapshot(snapshot)
            return predicate(snapshot)
        return checked

    def _call_startup(self, name, **fields):
        self._phase("STARTUP_COMMAND", f"{name}: waiting for response", name)
        progress = (self._validate_held_snapshot if self.node.holding_item
                    or getattr(self, "home_recovery", None) is not None else None)
        return self.call(name, progress=progress, **fields)

    @staticmethod
    def _idle(snapshot):
        return not DobotTransport._idle_blockers(snapshot)

    @staticmethod
    def _idle_blockers(snapshot):
        feed = snapshot.feed
        blockers = []
        if not snapshot.robot_enabled:
            blockers.append("RobotStatus.is_enable=False (required true in idle mode 5)")
        if feed["robot_mode"] != 5:
            blockers.append(f"robot_mode={feed['robot_mode']} (required idle mode 5)")
        if feed["EnableStatus"] != 1:
            blockers.append(f"EnableStatus={feed['EnableStatus']} (required 1)")
        for key in ("isRunQueuedCmd", "RunningStatus", "ErrorStatus", "CollisionStates"):
            if feed[key]:
                blockers.append(f"{key}={feed[key]} (required 0)")
        for key in ("userCoordinate", "toolCoordinate"):
            if feed[key] != 0:
                blockers.append(f"{key}={feed[key]} (required 0)")
        return blockers

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
        if getattr(self, "home_recovery", None) is not None:
            return self.home_recovery.check(snapshot)
        suction = bool(snapshot.feed["digital_input_bits"] & 1)
        returning = getattr(self, "return_recovery", False)
        if suction and not known_holding and not returning:
            raise HeldUnknown(UNKNOWN_ITEM_GUIDANCE)
        if known_holding or returning:
            self._validate_held_snapshot(
                snapshot, known_holding=True, expected_outputs=expected_outputs)
        return snapshot

    def _check_emergency_stop(self):
        # A read-only canonical query, with the normal ownership, feedback,
        # cancellation and five-second response guards. Never infer this alarm
        # from generic ErrorStatus/mode 9 or the driver's safety_status double.
        progress = (self._validate_held_snapshot if self.node.holding_item
                    or getattr(self, "home_recovery", None) is not None else None)
        result = self.call("GetErrorID", progress=progress)
        codes = alarm_ids(result.robot_return)
        self.node.events.record("INFO", "robot_alarm_check", "GetErrorID", alarm_ids=list(codes))
        if 1537 in codes:
            raise EmergencyStopPressed(f"{EMERGENCY_STOP_GUIDANCE} (Robot alarm 1537.)")

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
        try:
            self.monitor.wait_samples(
                self._held_predicate(
                    lambda sample: sample.feed["robot_mode"] != 9
                    and not sample.feed["ErrorStatus"]
                    and not sample.feed["CollisionStates"]),
                CONSISTENT_FLAG_SAMPLES, MODE_TRANSITION_TIMEOUT_SEC,
                cancel=self.node.cancel_requested, description="cleared error/collision feedback")
        except HeldSuctionLost:
            raise
        except FeedbackFailure:
            # ClearError may acknowledge an alarm that is still physically active.
            # Diagnose after the explicit clear attempt, so a latched E-stop does
            # not permanently block recovery once the operator releases it.
            self._check_emergency_stop()
            raise

    def _apply_settings(self, speed_percent, cp_percent=100):
        for name, fields in (
                ("SpeedFactor", {"ratio": speed_percent}),
                ("User", {"index": 0}),
                ("Tool", {"index": 0}),
                ("SetTool", {"index": 1, "value": "{0,0,0,0,0,0}"}),
                ("CP", {"r": cp_percent})):
            self._check_held_context(self.node.holding_item, self.node.expected_outputs)
            self._call_startup(name, **fields)
            if name == "SpeedFactor":
                self.node.global_speed_percent = speed_percent
                self.node.publish_status()
            elif name == "CP":
                self.node.global_cp_percent = cp_percent
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
        self._check_emergency_stop()
        try:
            self._call_startup("StopMoveJog")
        except EmergencyStopPressed:
            raise
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

    def recover(self, speed_percent, *, return_item=False, home_recovery=None):
        """Restore readiness; uncertain-item recovery preserves every output."""
        if return_item and not self.node.managed.recovery_return_needed():
            raise CommandRejected(
                "Item recovery requires a retained source with loss or return progress")
        self.return_recovery = return_item
        self.home_recovery = home_recovery
        try:
            self._recover(speed_percent)
        finally:
            self.return_recovery = False
            self.home_recovery = None

    def _recover(self, speed_percent):
        self.wait_services(optional=("StopMoveJog",))
        strict = tuple(name for name in self.required_services if name != "StopMoveJog")
        self.node.check_all_command_owners(strict)
        self.node.check_feedback_owners()
        self.ensure_no_pending_response()
        self._phase("QUEUE_RESET", "Stopping and discarding any queued motion", "Stop")
        stop = self.request_stop("explicit Recover queue reset")
        self._acknowledge_stop(stop, check_cancel=True)
        if self.home_recovery is not None:
            # Alarm/collision mode may prevent standstill confirmation until
            # ClearError/EnableRobot. Preserve fresh I/O now, with no pose wait.
            self.home_recovery.capture(self.node, self.monitor.snapshot(require_enabled=False))
        extra = ({"recovery_home": self.home_recovery}
                 if self.home_recovery is not None else {})
        self._check_held_context(self.node.holding_item, self.node.expected_outputs)
        self._clear_errors_if_needed()
        self._call_startup("EnableRobot")
        self._wait_enabled()
        self._phase("RECOVERY_STANDSTILL", "Confirming stationary joints and empty queue "
                    "after EnableRobot")
        self.confirm_stop(stop, allow_suction_loss=self.return_recovery, after_enable=True, **extra)
        cp_percent = self.node.global_cp_percent
        self._apply_settings(speed_percent if speed_percent is not None else 100,
                             cp_percent if cp_percent is not None else 100)
        if self.home_recovery is None:
            self._reset_outputs_if_unheld()
        self._confirm_ready()
        self._check_held_context(self.node.holding_item, self.node.expected_outputs)

    def _reset_outputs_if_unheld(self):
        if self.node.holding_item or getattr(self, "return_recovery", False):
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
                OUTPUT_FEEDBACK_TIMEOUT_SEC, cancel=self.node.cancel_requested,
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

    def set_global_cp(self, percent):
        if type(percent) is not int or not 0 <= percent <= 100:
            raise CommandRejected("Global CP must be an integer from 0 through 100")
        snapshot = self.monitor.snapshot(require_enabled=True)
        if not self._idle(snapshot) or self.moving:
            raise FeedbackFailure("CP requires stationary READY/HOLDING feedback")
        self._check_held_context(self.node.holding_item, self.node.expected_outputs)
        progress = self._validate_held_snapshot if self.node.holding_item else None
        self.call("CP", r=percent, progress=progress)
        # An accepted CP command is the last known setting, including when the
        # following held-item check fails. Recovery must retain this value.
        self.node.global_cp_percent = percent
        self.node.publish_status()
        self._check_held_context(self.node.holding_item, self.node.expected_outputs)

    def current_pose(self):
        initial = self._validate_held_snapshot(self._ready_snapshot())
        snapshot = self._wait_position(
            lambda sample: sample.robot_enabled and self._queue_idle(sample), initial,
            MODE_TRANSITION_TIMEOUT_SEC, "idle RobotStatus and advancing joints for motion origin")
        return self.pose_from_snapshot(snapshot)

    def pose_from_snapshot(self, snapshot):
        """One authoritative base_link <- Link6 pose from canonical joint feedback."""
        try:
            result = self.node.kinematics.forward(snapshot.joints)
            if not self._valid_rigid_matrix(result):
                raise ValueError("invalid FK transform")
            return result
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise FeedbackFailure(f"Invalid joint-derived Link6 pose: {exc}") from exc

    @staticmethod
    def _queue_idle(sample):
        """Command execution guard, separate from position and RobotStatus arrival."""
        return not (sample.feed["isRunQueuedCmd"] or sample.feed["RunningStatus"])

    @staticmethod
    def _fresh_position(sample, previous):
        return (sample.joint_stamp_ns > previous.joint_stamp_ns
                and sample.status_sequence > previous.status_sequence)

    def _wait_position(self, predicate, initial, timeout, description):
        deadline = time.monotonic() + timeout
        while True:
            self.node.raise_if_cancelled()
            sample = self._validate_held_snapshot(self._ready_snapshot())
            if self._fresh_position(sample, initial) and predicate(sample):
                return sample
            remaining = deadline - time.monotonic()
            details = (
                f"RobotStatus idle={sample.robot_enabled}; "
                f"joint stream advanced={sample.joint_stamp_ns > initial.joint_stamp_ns}; "
                f"RobotStatus advanced={sample.status_sequence > initial.status_sequence}; "
                f"command queue idle={self._queue_idle(sample)}")
            if remaining <= 0:
                raise FeedbackFailure(f"Timed out waiting for {description}; {details}")
            try:
                self.monitor.wait_next(sample.revision, remaining,
                                       cancel=self.node.cancel_requested, position=True)
            except FeedbackFailure as exc:
                raise FeedbackFailure(
                    f"Waiting for {description}: {details}; {exc}") from exc

    @staticmethod
    def _valid_rigid_matrix(matrix):
        return (isinstance(matrix, np.ndarray) and matrix.shape == (4, 4)
                and np.issubdtype(matrix.dtype, np.number)
                and not np.iscomplexobj(matrix) and np.all(np.isfinite(matrix))
                and np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
                and np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3),
                                atol=1e-6, rtol=0)
                and math.isclose(float(np.linalg.det(matrix[:3, :3])), 1.,
                                 abs_tol=1e-6))

    def _target_values(self, target):
        matrix = target.matrix
        if not self._valid_rigid_matrix(matrix):
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
            self.pose_from_snapshot(snapshot), target.matrix,
            translation_m=CARTESIAN_POSITION_TOLERANCE_M,
            rotation_deg=CARTESIAN_ORIENTATION_TOLERANCE_DEG)

    @staticmethod
    def _motion_command_id(result):
        value = getattr(result, "robot_return", None)
        match = (re.fullmatch(r"\{\s*([0-9]+)\s*\}", value.strip())
                 if isinstance(value, str) else None)
        if match is None or int(match[1]) > 2**64 - 1:
            raise FeedbackFailure("Motion acceptance lacks a valid queued command ID")
        return int(match[1])

    def home_already_reached(self, joints_rad):
        """Confirm the normal joint-Home completion gate without commanding motion."""
        if len(joints_rad) != 6:
            raise CommandRejected("Home skip check requires six taught joints")

        def reached(snapshot):
            return (snapshot.robot_enabled
                    and max(abs(actual - expected) for actual, expected in zip(
                        snapshot.joints, joints_rad)) <= HOME_JOINT_TOLERANCE_RAD)

        initial = self._validate_held_snapshot(self._ready_snapshot())
        return reached(initial)

    def wait_tray_position(self, joints_rad):
        """Read-only three-second arrival window; never queue observation travel."""
        deadline = time.monotonic() + 3.0
        while True:
            self.node.raise_if_cancelled()
            if self.home_already_reached(joints_rad):
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise FeedbackFailure(
                    "Not at Tray Detect position after 3 seconds; tray detection blocked")
            self.monitor.wait_next(
                self.monitor.revision, min(.1, remaining),
                cancel=self.node.cancel_requested, position=True)

    def confirm_home(self, joints_rad):
        """Bound post-output confirmation without interpreting a busy queue as motion."""
        initial = self._validate_held_snapshot(self._ready_snapshot())

        def reached(sample):
            return (sample.robot_enabled and self._queue_idle(sample)
                    and max(abs(a - b) for a, b in zip(
                        sample.joints, joints_rad)) <= HOME_JOINT_TOLERANCE_RAD)
        try:
            return self._wait_position(
                reached, initial, OUTPUT_FEEDBACK_TIMEOUT_SEC,
                "Home joints and idle RobotStatus after gripper reset")
        except FeedbackFailure as exc:
            sample = self.monitor.snapshot(require_enabled=True)
            error = math.degrees(max(abs(a - b) for a, b in zip(sample.joints, joints_rad)))
            raise FeedbackFailure(
                f"Home reset confirmation failed: RobotStatus idle={sample.robot_enabled}; "
                f"maximum joint error={error:.4f} degrees (limit 1); {exc}") from exc

    def _monitor_motion_policy(self, snapshot, *, require_suction, forbid_suction,
                               stop_on_suction, before_suction, planned_outputs,
                               suction_armed=True):
        feed = snapshot.feed
        detected = bool(feed["digital_input_bits"] & 1)
        vacuum = bool(feed["digital_outputs"] & (1 << 12))
        exhaust = bool(feed["digital_outputs"] & 1)
        finger_close = bool(feed["digital_outputs"] & (1 << 1))
        finger_open = bool(feed["digital_outputs"] & (1 << 13))
        if vacuum and exhaust:
            raise FeedbackFailure("Invalid vacuum state: DO13 SUCK and DO1 EXHAUST are both ON")
        if finger_close and finger_open:
            raise FeedbackFailure("Invalid gripper state: DO2 CLOSE and DO14 OPEN are both ON")
        for channel, planned in planned_outputs.items():
            actual = bool(feed["digital_outputs"] & (1 << (channel - 1)))
            if actual == planned:
                self.node.expected_outputs[channel] = planned
        observer = getattr(self.node, "observe_managed_feedback", None)
        managed_drop = observer(snapshot) if observer is not None else False
        if require_suction:
            if not vacuum:
                raise FeedbackFailure("Suction output DO13 lost during retract/Home")
            for channel, active in self.node.expected_outputs.items():
                actual = bool(feed["digital_outputs"] & (1 << (channel - 1)))
                if actual != active:
                    raise FeedbackFailure(
                        f"Held-item output DO{channel} changed during retract/Home")
            if not snapshot.suction_present and not managed_drop:
                raise HeldSuctionLost("Suction lost during retract/Home")
        if forbid_suction and detected:
            raise FeedbackFailure("Late DI1 after missed pickup; candidate retry forbidden")
        if stop_on_suction and suction_armed:
            if detected and not self.suction_interrupted:
                self.suction_interrupted = True
                self.suction_stop_future = self.request_stop("DI1 acquired during pickup")
            if not vacuum and before_suction is not None:
                before_suction()

    def move_batch(self, targets, *, queue_only=False, **kwargs):
        """Admit in order; optionally hand placement completion to its owning worker."""
        if queue_only and kwargs.get("placement") is None:
            raise CommandRejected("Queue-only admission is reserved for supervised placement")
        batch = self._move_batch(targets, **kwargs)
        try:
            next(batch)
        except StopIteration as completed:
            return completed.value
        except BaseException:
            batch.close()
            raise
        if queue_only:
            return batch
        return self.finish_batch(batch)

    def defer_pickup_drop(self, retract_pose, sample):
        """Arm loss detection at actual first-lift height without a queue barrier."""
        height = max(retract_pose[2, 3], self.pose_from_snapshot(sample)[2, 3])

        def armed(actual_height):
            self.node.events.record(
                "INFO", "drop_detection_armed", "First retract height reached; monitor DI1 loss",
                retract_z_m=height, actual_z_m=actual_height,
                debounce_ms=SUCTION_LOSS_DEBOUNCE_SEC * 1000)

        self.monitor.defer_suction_loss_until_retract(
            height, height_from_joints=lambda joints: self.node.kinematics.forward(joints)[2, 3],
            on_armed=armed)
        self.node.events.record(
            "INFO", "drop_detection_deferred", "Defer DI1 loss monitoring until first retract",
            retract_z_m=height, debounce_ms=SUCTION_LOSS_DEBOUNCE_SEC * 1000)

    @staticmethod
    def finish_batch(batch, *, handoff=None):
        """Resume the admitted group's existing guards and physical completion loop."""
        try:
            batch.send(handoff)
        except StopIteration as completed:
            return completed.value
        else:
            raise FeedbackFailure("Motion batch yielded more than one admission boundary")
        finally:
            batch.close()

    def _move_batch(self, targets, *, batch_name="motion", require_suction=False,
                    forbid_suction=False, stop_on_suction=False,
                    before_suction=None, pick_settling_sec=0.0,
                    require_suction_reset=False, return_terminal_pose=False,
                    confirmed_start_pose=None, preserve_outputs=False, placement=None,
                    placement_bridge=None, pickup_retract_pose=None):
        targets = tuple(targets)
        if (not targets or not isinstance(batch_name, str) or not batch_name.strip()
                or sum((require_suction, forbid_suction, stop_on_suction,
                        placement is not None)) > 1
                or (type(pick_settling_sec) not in (int, float)
                    or not math.isfinite(pick_settling_sec) or pick_settling_sec < 0)
                or (pick_settling_sec and not stop_on_suction)
                or (require_suction_reset and not stop_on_suction)
                or (pickup_retract_pose is not None and (
                    not stop_on_suction or not self._valid_rigid_matrix(pickup_retract_pose)))
                or (placement_bridge is not None and (
                    placement is not None or require_suction or forbid_suction
                    or require_suction_reset or confirmed_start_pose is None))
                or type(return_terminal_pose) is not bool
                or (confirmed_start_pose is not None
                    and not self._valid_rigid_matrix(confirmed_start_pose))):
            raise CommandRejected("Invalid motion-batch suction policy or empty batch")
        batch_name = batch_name.strip()
        start = (self.current_pose() if confirmed_start_pose is None
                 else confirmed_start_pose.copy())
        origin = start
        prepared = []
        expected_outputs = {}
        for target in targets:
            values = self._target_values(target)
            if target.relative_z:
                same_height_frame = target.matrix.copy()
                same_height_frame[2, 3] = origin[2, 3]
                if (not pose_reached(
                        origin, same_height_frame, translation_m=0.001,
                        rotation_deg=0.5)
                        or target.matrix[2, 3] < origin[2, 3]):
                    raise CommandRejected(
                        "Relative Home-height target must rise at unchanged XY/attitude")
            for event in target.motion_io:
                expected_outputs[event.channel] = event.active
            prepared.append((target, values, origin.copy()))
            origin = target.matrix
        self.suction_interrupted = False
        self.suction_stop_future = None
        self.suction_stop_needs_refresh = False
        self.acquisition_eligible = False
        self.late_miss_suction = require_suction_reset
        initial = self._ready_snapshot()
        fixed_outputs = dict(self.node.expected_outputs) if preserve_outputs else {}
        if (require_suction_reset
                and not initial.feed["digital_outputs"] & (1 << 12)):
            raise FeedbackFailure("Previous final approach lacks active DO13 before retry")
        suction_reset_seen = not require_suction_reset
        suction_clear_seen = (not require_suction_reset
                              and not bool(initial.feed["digital_input_bits"] & 1))
        suction_armed = False
        probe_started = False
        self.pending_motion_outputs = {}
        self.moving = True
        started = time.monotonic()
        last_progress = started
        last_vector = np.asarray(initial.joints, dtype=float)
        queued_targets = []
        self.node.events.record(
            "INFO", "motion_batch_dispatch_started", batch_name,
            batch=batch_name, targets=[target.name for target in targets],
            policy="admit_each_reply_in_order_then_verify_terminal_feedback")

        def progress(snapshot):
            nonlocal suction_reset_seen, suction_clear_seen, suction_armed
            if placement_bridge is not None:
                placement_bridge.observe(snapshot)
                if (placement_bridge.completed and not stop_on_suction
                        and (snapshot.feed["digital_outputs"] & GRIP_MASK
                             or snapshot.feed["digital_input_bits"] & 1)):
                    raise FeedbackFailure("Auto Run Home requires neutral outputs and DI1 LOW")
            if placement is not None:
                placement.observe(self.node, snapshot)
            for channel, active in fixed_outputs.items():
                actual = bool(snapshot.feed["digital_outputs"] & (1 << (channel - 1)))
                if actual != active:
                    context = "Pickup probe" if probe_started else "Return"
                    raise FeedbackFailure(f"{context} output DO{channel} changed before release")
            if require_suction_reset:
                vacuum = bool(snapshot.feed["digital_outputs"] & (1 << 12))
                detected = bool(snapshot.feed["digital_input_bits"] & 1)
                if not vacuum:
                    suction_reset_seen = True
                if suction_reset_seen and not detected:
                    suction_clear_seen = True
            vacuum = bool(snapshot.feed["digital_outputs"] & (1 << 12))
            bridge_ready = placement_bridge is None or placement_bridge.completed
            if placement_bridge is not None:
                suction_clear_seen = placement_bridge.completed
            if bridge_ready and suction_reset_seen and suction_clear_seen and vacuum:
                suction_armed = True
            self.acquisition_eligible = stop_on_suction and suction_armed
            if self.acquisition_eligible:
                self.late_miss_suction = False
            self._monitor_motion_policy(
                snapshot, require_suction=require_suction, forbid_suction=forbid_suction,
                stop_on_suction=stop_on_suction, before_suction=before_suction,
                planned_outputs={} if placement is not None else self.pending_motion_outputs,
                suction_armed=suction_armed)

        def finish_suction_interrupt():
            self.node.events.record(
                "INFO", "motion_batch_interrupted", batch_name,
                batch=batch_name, queued_targets=queued_targets,
                reason="DI1 acquired during pickup")
            # Retire any outstanding Stop before dispatching the return. When
            # pickup interrupted admission, a second ordered Stop must discard
            # the command that could have been admitted after the first Stop.
            self._acknowledge_stop(self.suction_stop_future, check_cancel=True)
            if self.suction_stop_needs_refresh:
                self.suction_stop_future = self.request_stop(
                    "discard motion admitted during pickup Stop", fresh=True)
                self._acknowledge_stop(self.suction_stop_future, check_cancel=True)
            sample = self._ready_snapshot()
            if (not sample.suction_present
                    or not sample.feed["digital_outputs"] & (1 << 12)):
                raise FeedbackFailure("Suction lost after acquisition Stop")
            for channel, active in expected_outputs.items():
                actual = bool(sample.feed["digital_outputs"] & (1 << (channel - 1)))
                if actual != active:
                    raise FeedbackFailure(
                        f"Motion-timed DO{channel} mismatch after suction Stop")
            self.node.expected_outputs.update(expected_outputs)
            self.pending_motion_outputs = {}
            return_origin = self.pose_from_snapshot(sample)
            if pickup_retract_pose is not None:
                self.defer_pickup_drop(pickup_retract_pose, sample)
            self.node.events.record(
                "INFO", "pickup_stop_acknowledged",
                "Pickup accepted; queue return without stationary confirmation",
                batch=batch_name, stationary_confirmation=False)
            return ((True, return_origin) if return_terminal_pose else True)

        observation = self.monitor.begin_motion()
        try:
            calls = []
            outputs_by_call = []
            target_records = []
            for target, values, previous in prepared:
                params = ["user=0", "tool=0", f"v={target.speed_percent}",
                          f"a={target.acceleration_percent}"]
                if target.relative_z:
                    service = "RelMovLUser"
                    fields = {
                        "a": 0., "b": 0.,
                        "c": (target.matrix[2, 3] - previous[2, 3]) * 1000,
                        "d": 0., "e": 0., "f": 0., "param_value": params,
                    }
                else:
                    command = values if target.joints_rad is None else list(
                        np.rad2deg(target.joints_rad))
                    events = [event.vendor_value() for event in target.motion_io]
                    service = "MovLIO" if events else "MovL"
                    fields = dict(zip("abcdef", map(float, command)))
                    if events:
                        fields["mdis"] = events
                    fields.update(mode=target.joints_rad is not None, param_value=params)
                calls.append((service, fields))
                outputs_by_call.append({
                    event.channel: event.active for event in target.motion_io})
                target_records.append((target, [
                    event.vendor_value() for event in target.motion_io]))

            self._wait_for_resume()
            self.node.raise_if_cancelled()
            progress(self._ready_snapshot())
            self.node.operation_progress(
                "MOTION", f"Dispatching {batch_name} as one command group",
                waypoint=targets[-1].name)
            extra = {"issuing": placement.issued} if placement is not None else {}
            if placement_bridge is not None:
                extra["accepted"] = placement_bridge.accepted
                issuing = getattr(placement_bridge, "issued", None)
                if issuing is not None:
                    extra["issuing"] = issuing
            replies = self.call_group(
                calls, progress=progress, outputs_by_call=outputs_by_call,
                admitted=lambda index: (
                    placement_bridge.admitted(targets[index]) if placement_bridge is not None
                    else self._target_admitted(targets[index])), **extra)
            queued_targets.extend(
                target.name for target, _events in target_records[0:len(replies)])
            expected_outputs = {}
            for target, events in target_records[0:len(replies)]:
                expected_outputs.update({event.channel: event.active
                                         for event in target.motion_io})
                self.node.events.record(
                    "INFO", "motion_queued", target.name,
                    batch=batch_name,
                    speed_percent=target.speed_percent,
                    acceleration_percent=target.acceleration_percent,
                    motion_io=events)
            if self.suction_interrupted:
                return finish_suction_interrupt()
            if len(replies) != len(calls):
                raise FeedbackFailure("Motion group stopped before all targets were admitted")
            # These are the fixed vendor service schemas: only MovL exposes the
            # queued ID. MovLIO/RelMovLUser return res alone, so retain observed
            # execution from the stream rather than inventing their queue IDs.
            terminal_command_id = (self._motion_command_id(replies[-1])
                                   if calls[-1][0] == "MovL" else None)
            accepted = self.monitor.snapshot(require_enabled=True)
            self.node.events.record(
                "INFO", "motion_batch_queued", batch_name,
                batch=batch_name, targets=queued_targets,
                terminal_target=targets[-1].name, terminal_command_id=terminal_command_id)
            # The generator retains moving/execution evidence and cleanup until
            # its one owner finishes or closes it. Normal actions resume inline.
            handoff = yield
            if handoff is not None and placement is None:
                raise CommandRejected("Only placement may hand off its admitted queue")
            tail = targets[-1]
            terminal_stable_sec = (pick_settling_sec if stop_on_suction
                                   else 0.0)
            stable_since = None
            settle_origin = None
            output_wait_since = None
            sequence = accepted.revision
            while True:
                paused_for = self._wait_for_resume()
                started += paused_for
                last_progress += paused_for
                self.node.raise_if_cancelled()
                snapshot = self._ready_snapshot()
                progress(snapshot)
                if self.suction_interrupted:
                    return finish_suction_interrupt()
                now = time.monotonic()
                vector = np.asarray(snapshot.joints, dtype=float)
                if np.max(np.abs(vector - last_vector)) > math.radians(0.05):
                    last_progress, last_vector = now, vector
                reached = self._target_reached(tail, snapshot)
                # Placement's action acknowledges admission; its worker checks
                # the final retract and grip without intermediate release gates.
                outputs_ready = placement is not None or all(
                    bool(snapshot.feed["digital_outputs"] & (1 << (channel - 1)))
                    == active for channel, active in expected_outputs.items())
                executed = (snapshot.feed["currentCommandId"] == terminal_command_id
                            if terminal_command_id is not None else observation.started)
                motion_idle = (
                    self._fresh_position(snapshot, accepted)
                    and executed and self._queue_idle(snapshot)
                    and reached and snapshot.robot_enabled
                    and (placement_bridge is None or placement_bridge.completed))
                idle = motion_idle and outputs_ready
                waiting_outputs = motion_idle and not outputs_ready and not stop_on_suction
                output_wait_since = now if waiting_outputs and output_wait_since is None else (
                    output_wait_since if waiting_outputs else None)
                if (output_wait_since is not None
                        and now - output_wait_since >= OUTPUT_FEEDBACK_TIMEOUT_SEC):
                    raise FeedbackFailure("Timed out waiting for motion-timed output feedback")
                if idle and stable_since is None:
                    stable_since, settle_origin = now, snapshot
                elif not idle:
                    stable_since, settle_origin = None, None
                if (stable_since is not None
                        and now - stable_since >= terminal_stable_sec
                        and (terminal_stable_sec == 0
                             or self._fresh_position(snapshot, settle_origin))):
                    if pickup_retract_pose is not None and not probe_started:
                        # Keep this attempt and its acquisition filter alive across
                        # settling and the last-chance lift. Do not latch a miss,
                        # reset arming, release vacuum or move the fingers here.
                        probe_started = True
                        actual = self.pose_from_snapshot(snapshot)
                        lift = .5 * max(0., pickup_retract_pose[2, 3] - actual[2, 3])
                        bits = snapshot.feed["digital_outputs"]
                        if not suction_armed or not bits & (1 << 12):
                            raise FeedbackFailure("Pickup probe requires armed suction DO13 ON")
                        fixed_outputs.update({
                            channel: bool(bits & (1 << (channel - 1)))
                            for channel in (1, 2, 13, 14)})
                        self.node.events.record(
                            "INFO", "pickup_probe_started", "Settling expired; try 50% upward lift",
                            batch=batch_name, settling_sec=pick_settling_sec,
                            lift_mm=lift * 1000, speed_percent=tail.speed_percent,
                            acceleration_percent=tail.acceleration_percent)
                        # A zero-height first retract offers no upward travel.
                        # Never invent a clearance or command a downward probe.
                        if lift <= 0:
                            break
                        destination = actual.copy()
                        destination[2, 3] += lift
                        tail = replace(tail, name=f"{tail.name}_pickup_probe", matrix=destination,
                                       motion_io=(), joints_rad=None, relative_z=False)
                        fields = dict(zip("abcdef", map(float, self._target_values(tail))))
                        fields.update(mode=False, param_value=[
                            "user=0", "tool=0", f"v={tail.speed_percent}",
                            f"a={tail.acceleration_percent}"])
                        self._wait_for_resume()
                        self.node.raise_if_cancelled()
                        progress(self._ready_snapshot())
                        if self.suction_interrupted:
                            return finish_suction_interrupt()
                        self.node.operation_progress(
                            "MOTION", "No DI1 after settling; trying 50% upward lift",
                            waypoint=tail.name)
                        replies = self.call_group(
                            [("MovL", fields)], progress=progress, outputs_by_call=[{}],
                            admitted=lambda _index: self._target_admitted(tail))
                        if replies:
                            queued_targets.append(tail.name)
                            self.node.events.record(
                                "INFO", "motion_queued", tail.name, batch=batch_name,
                                speed_percent=tail.speed_percent,
                                acceleration_percent=tail.acceleration_percent, motion_io=[])
                        if self.suction_interrupted:
                            return finish_suction_interrupt()
                        if len(replies) != 1:
                            raise FeedbackFailure("Pickup probe was not admitted")
                        terminal_command_id = self._motion_command_id(replies[0])
                        accepted = self.monitor.snapshot(require_enabled=True)
                        self.node.events.record(
                            "INFO", "pickup_probe_queued", tail.name, batch=batch_name,
                            terminal_command_id=terminal_command_id)
                        terminal_stable_sec = 0.
                        stable_since = settle_origin = output_wait_since = None
                        last_progress = time.monotonic()
                        last_vector = np.asarray(accepted.joints, dtype=float)
                        sequence = accepted.revision
                        continue
                    if placement is not None:
                        placement.complete(self.node, snapshot)
                    break
                if handoff is not None and handoff():
                    # One Auto Run owner immediately appends behind this queue.
                    # Prefer already-confirmable completion for accurate counts.
                    return
                if not waiting_outputs and now - last_progress >= MOTION_NO_PROGRESS_SEC:
                    raise FeedbackFailure("Motion made no measurable progress for three seconds")
                if now - started >= MOTION_HARD_CAP_SEC:
                    raise FeedbackFailure("Motion exceeded the 300-second hard deadline")
                sequence = self.monitor.wait_next(
                    sequence, min(1.0, MOTION_HARD_CAP_SEC - (now - started)),
                    cancel=self.node.cancel_requested, position=True)
            # Release queues reconcile observed I/O themselves; acceptance of
            # timed outputs is not evidence of their physical execution.
            if expected_outputs and not stop_on_suction and placement is None:
                self.node.expected_outputs.update(expected_outputs)
            if require_suction_reset and not suction_reset_seen:
                raise FeedbackFailure(
                    "Missed-pick DO13 OFF was not observed before the next pick")
            if stop_on_suction:
                self.node.expected_outputs.update(expected_outputs)
                self.acquisition_eligible = False
                self.late_miss_suction = True
            self.pending_motion_outputs = {}
            acquired_at_settle = False
            self.node.events.record(
                "INFO", "motion_batch_completed", batch_name,
                batch=batch_name, terminal_target=tail.name,
                targets=queued_targets, pickup_probe_attempted=probe_started,
                terminal_stable_sec=terminal_stable_sec,
                suction_confirmed=acquired_at_settle)
            stopped_pose = self.pose_from_snapshot(snapshot)
            return ((acquired_at_settle, stopped_pose)
                    if return_terminal_pose else acquired_at_settle)
        except OperationCanceled:
            self.request_stop("motion operation cancelled")
            raise
        finally:
            self.monitor.end_motion(observation)
            self.moving = False

    def move(self, target, *, require_suction=False):
        return self.move_batch((target,), require_suction=require_suction)

    def _target_admitted(self, target):
        callback = getattr(self.node, "motion_admitted", None)
        if callback is not None:
            callback(target)

    def exhaust_pulse(self):
        """Controller-timed 50 ms pulse, with ON/OFF evidence retained during reply."""
        initial = self._ready_snapshot()
        bits = initial.feed["digital_outputs"]
        if bits & ((1 << 12) | (1 << 1) | 1) or not bits & (1 << 13):
            raise FeedbackFailure("Release pulse requires suction/CLOSE/exhaust OFF and OPEN ON")
        self.call("DO", index=1, status=1, time=50)

        def completed(sample):
            history = self.monitor.output_history(initial.sequence)
            on = next((entry for entry in history if entry[2] & 1), None)
            if on is None:
                return False
            off = next((entry for entry in history
                        if entry[0] > on[0] and not entry[2] & 1), None)
            if off is None:
                return False
            return (sample.sequence >= off[0] and not sample.feed["digital_input_bits"] & 1
                    and sample.feed["digital_outputs"] & ((1 << 13) | (1 << 12) | 3)
                    == (1 << 13))

        self.monitor.wait(completed, OUTPUT_FEEDBACK_TIMEOUT_SEC,
                          cancel=self.node.cancel_requested, require_enabled=True,
                          description="50 ms exhaust pulse ON/OFF and released DI1")
        self.node.expected_outputs.update({1: False, 2: False, 13: False, 14: True})

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
            OUTPUT_FEEDBACK_TIMEOUT_SEC, cancel=self.node.cancel_requested,
            pause=self._pause_requested, require_enabled=True,
            description=f"DO{channel} output feedback")
        self.node.expected_outputs[channel] = active
        self.pending_motion_outputs.pop(channel, None)

    def sensor(self, active, timeout):
        self._wait_for_resume()

        def expected(sample):
            return bool(sample.feed["digital_input_bits"] & 1) == active
        try:
            self.monitor.wait(expected, timeout, cancel=self.node.cancel_requested,
                              pause=self._pause_requested, require_enabled=True,
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
