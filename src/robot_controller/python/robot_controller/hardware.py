"""Canonical Dobot service transport; instantiated only while Live is ON."""

import math
import re
import threading
import time

import numpy as np

from .kinematics import pose_matrix, pose_values
from .motion import pose_reached


SERVICE_TIMEOUT_SEC = 5.0
MOTION_TIMEOUT_SEC = 30.0
STATIONARY_SEC = 0.3


class ResponsePending(ValueError):
    """An unanswered command forbids dispatching a later normal command."""


def robot_values(raw, command):
    match = re.fullmatch(r"0,\{([^{}]+)\}," + re.escape(command) + r"\([^;]*\);?\s*", raw)
    if match is None:
        raise ValueError(f"Malformed canonical {command} reply")
    try:
        values = [float(v) for v in match.group(1).split(",")]
    except ValueError as exc:
        raise ValueError(f"Malformed canonical {command} values") from exc
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError(f"{command} must return six finite values")
    return values


class DobotHardware:
    def __init__(self, node):
        # Debug construction never imports these types or creates command clients.
        from dobot_msgs_v4.srv import (CP, DO, DisableRobot, EnableRobot, GetPose, InverseKin,
                                       MovLIO, RelMovLUser, SetTool, SpeedFactor, Stop, StopMoveJog,
                                       Tool)
        self.node = node
        kinds = (CP, DO, DisableRobot, EnableRobot, GetPose, InverseKin, MovLIO, RelMovLUser,
                 SetTool, SpeedFactor, Stop, StopMoveJog, Tool)
        self.types = {kind.__name__: kind for kind in kinds}
        self.clients = {name: node.create_client(kind, f"/dobot_bringup_ros2/srv/{name}")
                        for name, kind in self.types.items()}
        self.moving = False
        self.suction_stop = None
        self.suction_interrupted = False
        self.response_lock = threading.Lock()
        self.pending_response = None

    def call(self, name, *, monitor_suction=False, require_clear=False, progress=None, **fields):
        with self.response_lock:
            try:
                if self.pending_response is not None:
                    previous, future = self.pending_response
                    if not future.done():
                        raise ResponsePending(
                            f"{name} not sent: still awaiting {previous} response")
                    self.pending_response = None
                return self._call(name, monitor_suction=monitor_suction,
                                  require_clear=require_clear, progress=progress, **fields)
            except Exception as exc:
                self.node.events.record("ERROR", "robot_service_failed", str(exc), service=name)
                raise

    def _call(self, name, *, monitor_suction=False, require_clear=False, progress=None, **fields):
        self.node.check_cancelled()
        self.node.check_command_owner(name)
        self.node.feedback_snapshot(enabled=False)
        client = self.clients[name]
        if not client.service_is_ready():
            raise ValueError(f"Required canonical {name} service unavailable")
        request = self.types[name].Request(**fields)
        self.node.events.record("INFO", "robot_service_sent", name, fields=fields)
        future = client.call_async(request)
        self.pending_response = name, future
        if name in ("MovLIO", "RelMovLUser"):
            # A late accepted command after cancellation must receive another safety Stop.
            # This is containment, not a retry of motion or a claim of confirmed stopping.
            future.add_done_callback(self._late_motion_ack)
        deadline = time.monotonic() + SERVICE_TIMEOUT_SEC
        while not future.done():
            self.node.check_cancelled()
            snapshot = self.node.feedback_snapshot(enabled=progress is not None)
            if progress is not None:
                progress(snapshot)
            if require_clear and snapshot["feed"]["digital_input_bits"] & 1:
                raise ValueError("Unexpected DI1 during descent before suction; Stop required")
            if monitor_suction and snapshot["feed"]["digital_input_bits"] & 1:
                self._interrupt_suction()
            if time.monotonic() >= deadline:
                raise ResponsePending(f"{name} response timeout; no later commands sent; no retry")
            time.sleep(0.02)
        result = future.result()
        self.pending_response = None
        self.node.check_cancelled()
        self.node.feedback_snapshot(enabled=False)
        if result is None or result.res != 0:
            raise ValueError(f"{name} failed: {None if result is None else result.res}")
        self.node.events.record("INFO", "robot_service", name, fields=fields)
        if name in ("MovLIO", "RelMovLUser") and self.suction_interrupted:
            # Stop might have been processed BEFORE this motion was accepted.
            # Contain late acceptance synchronously after the awaited response,
            # not via a callback that could run after stationary confirmation.
            self.suction_stop = self.stop()
            if self.suction_stop is None:
                raise ValueError("Late motion acknowledgement after DI1 Stop; "
                                 "new Stop unavailable, retract blocked")
            self.node.events.record("WARNING", "late_suction_ack_stop",
                                    "Motion acknowledged after DI1 Stop; Stop sent again")
        if progress is not None:
            progress(self.node.feedback_snapshot(enabled=True))
        return result

    def _late_motion_ack(self, _future):
        if self.node.cancel.is_set():
            try:
                self.stop()
            except Exception as exc:
                self.node.events.record("ERROR", "late_ack_stop_failed", str(exc))

    def wait(self, predicate, timeout, *, enabled=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.node.check_cancelled()
            snapshot = self.node.feedback_snapshot(enabled=enabled)
            if predicate(snapshot):
                return True
            time.sleep(0.02)
        return False

    def initialize(self):
        self.node.startup_settings_applied = False
        self.node.set_execution_state("INITIALIZING", "Checking canonical robot feedback")
        deadline = time.monotonic() + SERVICE_TIMEOUT_SEC
        while True:
            try:
                snapshot = self.node.feedback_snapshot(enabled=False)
                if snapshot["feed"]["robot_mode"] not in (4, 5, 11):
                    raise ValueError("Initialization requires robot mode Disabled/Enabled/Jogging")
                break
            except ValueError:
                if time.monotonic() >= deadline:
                    raise
                self.node.check_cancelled()
                time.sleep(0.02)
        required = ("EnableRobot", "SpeedFactor", "Tool", "SetTool", "CP")
        self.node.set_execution_state("INITIALIZING", "Waiting for required startup services")
        deadline = time.monotonic() + SERVICE_TIMEOUT_SEC
        while True:
            missing = [name for name in required if not self.clients[name].service_is_ready()]
            if not missing:
                break
            self.node.check_cancelled()
            self.node.feedback_snapshot(enabled=False)
            if time.monotonic() >= deadline:
                raise ValueError("Startup required services unavailable: " + ", ".join(missing))
            time.sleep(0.02)
        for name in ("StopMoveJog", "DisableRobot"):
            try:
                self._startup_call(name)
                if name == "DisableRobot" and not self.wait(
                        lambda s: s["feed"]["robot_mode"] == 4, SERVICE_TIMEOUT_SEC):
                    raise ValueError("DisableRobot did not confirm Disabled mode")
            except ResponsePending:
                raise  # No response is not a returned failure: never overlap the next call.
            except ValueError as exc:
                self.node.events.record("WARNING", "startup_best_effort", str(exc), service=name)
        self._startup_call("EnableRobot")
        self.node.set_execution_state("INITIALIZING", "Startup EnableRobot: confirming Enabled")
        if not self.wait(lambda s: s["feed"]["robot_mode"] == 5 and s["enabled"]
                         and s["feed"]["EnableStatus"] == 1,
                         SERVICE_TIMEOUT_SEC):
            raise ValueError("EnableRobot did not confirm Enabled mode")
        self._startup_call("SpeedFactor", ratio=100)
        self._startup_call("Tool", index=0)
        self._startup_call("SetTool", index=1, value="{0,0,0,0,0,0}")
        self._startup_call("CP", r=100)
        self.node.startup_settings_applied = True
        try:
            self.confirm_ready()
        except ValueError as exc:
            message = f"Startup calls completed; {exc}"
            self.node.events.record("ERROR", "startup_readiness_blocked", message)
            self.node.set_execution_state("FAILED", message)
            return
        self.node.set_execution_state("READY", "Initialized at SpeedFactor 100%, Tool 0, CP 100%")

    def confirm_ready(self):
        """Boundedly await coherent feedback, without reissuing any command."""
        deadline = time.monotonic() + SERVICE_TIMEOUT_SEC
        while True:
            self.node.check_cancelled()
            try:
                snapshot = self.node.feedback_snapshot(enabled=True)
                feed = snapshot["feed"]
                if feed["robot_mode"] != 5 or feed["isRunQueuedCmd"] or feed["RunningStatus"]:
                    raise ValueError(
                        f"Robot readiness blocked: robot_mode={feed['robot_mode']}, "
                        f"isRunQueuedCmd={feed['isRunQueuedCmd']}, "
                        f"RunningStatus={feed['RunningStatus']} (required idle Enabled mode 5)")
                return snapshot
            except ValueError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.02)

    def enable_robot(self):
        self.node.set_execution_state("ENABLING", "EnableRobot: waiting for response")
        self.call("EnableRobot")
        self.node.set_execution_state("ENABLING", "EnableRobot: confirming enabled readiness")
        self.confirm_ready()
        self.node.set_execution_state("READY", "EnableRobot confirmed; no Home or Pick sent")

    def _startup_call(self, name, **fields):
        self.node.set_execution_state("INITIALIZING", f"Startup {name}: waiting for response")
        try:
            return self.call(name, **fields)
        except ResponsePending as exc:
            raise ResponsePending(f"Startup {name}: {exc}") from exc
        except ValueError as exc:
            raise ValueError(f"Startup {name}: {exc}") from exc

    def current_pose(self):
        snapshot = self.node.feedback_snapshot(enabled=True)
        if (snapshot["feed"]["isRunQueuedCmd"] or snapshot["feed"]["RunningStatus"]
                or snapshot["feed"]["robot_mode"] != 5):
            raise ValueError("Current-pose acquisition requires stationary enabled robot")
        result = self.call("GetPose", user=0, tool=0)
        matrix = pose_matrix(robot_values(result.robot_return, "GetPose"))
        modeled = self.node.kinematics.forward(self.node.current_joints())
        if not pose_reached(modeled, matrix, translation_m=0.002, rotation_deg=0.5):
            raise ValueError("CR10 FK does not match GetPose(user=0,tool=0); motion blocked")
        return matrix

    def _prepare_batch(self, targets, start, *, before_suction):
        """Validate every endpoint/IK while idle, before creating any motion queue."""
        prepared = []
        origin = start
        near = tuple(self.node.current_joints())
        for target in targets:
            if before_suction is not None:
                before_suction()
            self.node.check_cancelled()
            snapshot = self.node.feedback_snapshot(enabled=True)
            if (snapshot["feed"]["isRunQueuedCmd"] or snapshot["feed"]["RunningStatus"]
                    or snapshot["feed"]["robot_mode"] != 5):
                raise ValueError("Queue preflight requires stationary enabled robot")
            values = pose_values(target.matrix)
            if target.joints_rad is None:
                result = self.call(
                    "InverseKin", x=values[0], y=values[1], z=values[2],
                    rx=values[3], ry=values[4], rz=values[5], use_joint_near="1",
                    joint_near="{" + ",".join(map(str, np.rad2deg(near))) + "}",
                    user="0", tool="0")
                near = tuple(math.radians(v) for v in robot_values(
                    result.robot_return, "InverseKin"))
            else:
                near = target.joints_rad
            modeled = self.node.kinematics.forward(near)
            if not pose_reached(modeled, target.matrix, translation_m=0.002, rotation_deg=0.5):
                raise ValueError(f"Queue InverseKin/FK mismatch for {target.name}; no motion sent")
            if target.relative_z:
                if (not np.allclose(origin[:2, 3], target.matrix[:2, 3], atol=1e-12, rtol=0)
                        or not np.allclose(origin[:3, :3], target.matrix[:3, :3],
                                           atol=1e-12, rtol=0)
                        or target.matrix[2, 3] < origin[2, 3]):
                    raise ValueError("Queued relative Home-height move must rise only at same XY")
            prepared.append((target, values, origin.copy()))
            origin = target.matrix
        return prepared

    def move_batch(self, targets, *, require_suction=False, forbid_suction=False,
                   stop_on_suction=False, before_suction=None):
        """Queue response-serialized waypoints, then confirm the whole owned queue."""
        targets = tuple(targets)
        if not targets:
            raise ValueError("Motion batch cannot be empty")
        if sum((require_suction, forbid_suction, stop_on_suction)) > 1:
            raise ValueError("Motion batch suction policies are mutually exclusive")
        start = self.current_pose()  # Includes current GetPose versus canonical FK validation.
        self.suction_stop, self.suction_interrupted = None, False
        suction_started = False
        expected_outputs = {}
        for target in targets:
            for event in target.motion_io:
                expected_outputs[event.channel] = event.active

        def monitor(snapshot):
            nonlocal suction_started
            self.node.check_cancelled()
            feed = snapshot["feed"]
            detected = bool(feed["digital_input_bits"] & 1)
            if require_suction and not detected:
                raise ValueError("Suction lost during queued retract/Home")
            if forbid_suction and detected:
                raise ValueError("Late DI1 after missed pickup; Stop required, no candidate retry")
            if stop_on_suction:
                vacuum = bool(feed["digital_outputs"] & (1 << 12))
                if not suction_started and not vacuum and before_suction is not None:
                    before_suction()  # Expiry during queued transit stops the pending descent.
                suction_started = suction_started or vacuum
                if detected:
                    if not vacuum:
                        raise ValueError("Unexpected DI1 before queued suction start; "
                                         "Stop required")
                    self._interrupt_suction()
                elif suction_started and not vacuum:
                    raise ValueError("Suction output lost during queued approach")

        monitor(self.node.feedback_snapshot(enabled=True))
        prepared = self._prepare_batch(targets, start, before_suction=before_suction)
        before = self.node.feedback_snapshot(enabled=True)["sequence"]
        deadline = time.monotonic() + MOTION_TIMEOUT_SEC
        self.node.events.record("INFO", "motion_batch_started", "Queueing owned waypoints",
                                targets=[target.name for target in targets])
        for target, values, origin in prepared:
            monitor(self.node.feedback_snapshot(enabled=True))
            if self.suction_interrupted:
                break  # Stop discarded the forward queue; never submit the rest of descent.
            if time.monotonic() >= deadline:
                raise ValueError("Motion batch deadline expired; no later waypoint sent")
            params = ["user=0", "tool=0", f"v={target.speed_percent}",
                      f"a={target.acceleration_percent}", "cp=0"]
            self.moving = True  # An unanswered acknowledgement is ambiguous motion acceptance.
            if target.relative_z:
                self.call("RelMovLUser", a=0., b=0.,
                          c=(target.matrix[2, 3] - origin[2, 3])*1000,
                          d=0., e=0., f=0., param_value=params, progress=monitor)
            else:
                command_values = (values if target.joints_rad is None
                                  else list(np.rad2deg(target.joints_rad)))
                self.call("MovLIO", mode=target.joints_rad is not None,
                          **dict(zip("abcdef", map(float, command_values))),
                          mdis=[event.vendor_value() for event in target.motion_io],
                          param_value=params, progress=monitor)
            self.node.events.record("INFO", "motion_queued", target.name,
                                    motion_io=[event.vendor_value() for event in target.motion_io],
                                    speed_percent=target.speed_percent,
                                    acceleration_percent=target.acceleration_percent)
        stable_since = None
        tail = targets[-1]

        def complete(snapshot):
            nonlocal stable_since
            monitor(snapshot)
            if self.suction_interrupted:
                return True
            feed = snapshot["feed"]
            reached = pose_reached(pose_matrix(feed["tool_vector_actual"]), tail.matrix)
            if tail.joints_rad is not None:
                reached = reached and max(abs(a-b) for a, b in zip(
                    self.node.current_joints(), tail.joints_rad)) <= 0.005
            if (snapshot["sequence"] <= before or not reached or feed["isRunQueuedCmd"]
                    or feed["RunningStatus"] or feed["robot_mode"] != 5):
                stable_since = None
                return False
            stable_since = time.monotonic() if stable_since is None else stable_since
            return time.monotonic() - stable_since >= STATIONARY_SEC

        if not self.wait(complete, max(0., deadline-time.monotonic()), enabled=True):
            raise ValueError("Motion batch completion timeout; acceptance is not arrival")
        if self.suction_interrupted:
            self._confirm_suction_stop()
            feed = self.node.feedback_snapshot(enabled=True)["feed"]
            if not feed["digital_input_bits"] & 1 or not feed["digital_outputs"] & (1 << 12):
                raise ValueError("Suction lost after queued descent Stop; retract blocked")
        # Hardware events must actually be reflected by fresh output feedback.

        def outputs_confirmed(snapshot):
            monitor(snapshot)
            return snapshot["sequence"] > before and all(
                bool(snapshot["feed"]["digital_outputs"] & (1 << (channel-1))) == active
                for channel, active in expected_outputs.items())
        if not self.wait(outputs_confirmed, SERVICE_TIMEOUT_SEC, enabled=True):
            raise ValueError("MovLIO motion I/O feedback confirmation timeout; no retry")
        self.moving = False
        self.node.events.record("INFO", "motion_batch_completed", tail.name,
                                targets=[target.name for target in targets],
                                suction_stop=self.suction_interrupted)
        return self.suction_interrupted

    def move(self, target, *, require_suction=False, stop_on_suction=False):
        snapshot = self.node.feedback_snapshot(enabled=True)
        if require_suction and not snapshot["feed"]["digital_input_bits"] & 1:
            raise ValueError("Suction lost before movement")
        self.suction_stop, self.suction_interrupted = None, False
        if stop_on_suction and snapshot["feed"]["digital_input_bits"] & 1:
            self.moving = True
            self._interrupt_suction()
            self._confirm_suction_stop()
            self.moving = False
            return True  # Vacuum may seal while its output acknowledgement arrives.
        values = pose_values(target.matrix)
        require_clear = (not require_suction and not stop_on_suction and
                         snapshot["feed"]["tool_vector_actual"][2] > values[2] + 0.1)
        if require_clear and snapshot["feed"]["digital_input_bits"] & 1:
            raise ValueError("Unexpected DI1 before suction descent")
        if target.joints_rad is None:
            # Strict nearest-joint IK and model-limit check; no optional IK bypass.
            joint_near = "{" + ",".join(map(str, np.rad2deg(self.node.current_joints()))) + "}"
            result = self.call("InverseKin", x=values[0], y=values[1], z=values[2],
                               rx=values[3], ry=values[4], rz=values[5], use_joint_near="1",
                               joint_near=joint_near, user="0", tool="0")
            joints = tuple(math.radians(v) for v in robot_values(result.robot_return, "InverseKin"))
            modeled = self.node.kinematics.forward(joints)
            if not pose_reached(modeled, target.matrix, translation_m=0.002, rotation_deg=0.5):
                raise ValueError("InverseKin does not match the target in canonical CR10 geometry")
        before = self.node.feedback_snapshot(enabled=True)["sequence"]
        if stop_on_suction and self.node.feedback_snapshot(enabled=True)["feed"][
                "digital_input_bits"] & 1:
            self.moving = True
            self._interrupt_suction()
            self._confirm_suction_stop()
            self.moving = False
            return True
        self.moving = True  # A timeout is ambiguous acceptance: Stop must be attempted.
        params = ["user=0", "tool=0", f"v={target.speed_percent}",
                  f"a={target.acceleration_percent}"]
        if target.relative_z:
            actual = self.current_pose()
            self.call("RelMovLUser", a=0.0, b=0.0,
                      c=values[2] - actual[2, 3]*1000, d=0.0, e=0.0, f=0.0, param_value=params,
                      require_clear=require_clear)
        else:
            command_values = values if target.joints_rad is None else list(
                np.rad2deg(target.joints_rad))
            self.call("MovLIO", mode=target.joints_rad is not None,
                      **dict(zip("abcdef", map(float, command_values))),
                      mdis=[event.vendor_value() for event in target.motion_io], param_value=params,
                      monitor_suction=stop_on_suction, require_clear=require_clear)
        stable_since = None
        suction_detected = False

        def complete(s):
            nonlocal stable_since, suction_detected
            if require_clear and s["feed"]["digital_input_bits"] & 1:
                raise ValueError("Unexpected DI1 during descent before suction; Stop required")
            if stop_on_suction and (
                    self.suction_interrupted or s["feed"]["digital_input_bits"] & 1):
                suction_detected = True
                return True
            if require_suction and not s["feed"]["digital_input_bits"] & 1:
                raise ValueError("Suction lost during retract/Home")
            reached = pose_reached(pose_matrix(s["feed"]["tool_vector_actual"]), target.matrix)
            if target.joints_rad is not None:
                pairs = zip(self.node.current_joints(), target.joints_rad)
                reached = reached and max(abs(a-b) for a, b in pairs) <= 0.005
            if (s["sequence"] <= before or not reached or s["feed"]["isRunQueuedCmd"]
                    or s["feed"]["RunningStatus"] or s["feed"]["robot_mode"] != 5):
                stable_since = None
                return False
            stable_since = time.monotonic() if stable_since is None else stable_since
            return time.monotonic() - stable_since >= STATIONARY_SEC

        if not self.wait(complete, MOTION_TIMEOUT_SEC, enabled=True):
            raise ValueError("Movement completion timeout; command acceptance is not completion")
        if suction_detected:
            self._interrupt_suction()
            self._confirm_suction_stop()
        self.moving = False
        self.node.events.record("INFO", "motion_completed", target.name,
                                target=values, joint_target=target.joints_rad is not None,
                                speed_percent=target.speed_percent,
                                acceleration_percent=target.acceleration_percent,
                                suction_stop=suction_detected)
        return suction_detected

    def _interrupt_suction(self):
        if self.suction_stop is None:
            self.node.check_command_owner("Stop")
            self.suction_stop = self.stop()
            if self.suction_stop is None:
                raise ValueError("DI1 Stop service unavailable; retract blocked")
        self.suction_interrupted = True

    def _confirm_suction_stop(self):
        if self.suction_stop is None:
            raise ValueError("DI1 Stop unavailable after motion acknowledgement; retract blocked")
        deadline = time.monotonic() + SERVICE_TIMEOUT_SEC
        while not self.suction_stop.done():
            self.node.check_cancelled()
            self.node.feedback_snapshot(enabled=True)
            if time.monotonic() >= deadline:
                raise ValueError("DI1 Stop acknowledgement timeout; retract blocked")
            time.sleep(0.02)
        result = self.suction_stop.result()
        if result is None or result.res != 0:
            raise ValueError("DI1 Stop rejected; retract blocked")
        before = self.node.feedback_snapshot(enabled=True)["sequence"]
        stable_since, anchor = None, None

        def stopped(snapshot):
            nonlocal stable_since, anchor
            feed = snapshot["feed"]
            actual = pose_matrix(feed["tool_vector_actual"])
            stationary = (snapshot["sequence"] > before and not feed["isRunQueuedCmd"]
                          and not feed["RunningStatus"] and feed["robot_mode"] == 5
                          and anchor is not None and pose_reached(anchor, actual))
            if not stationary:
                stable_since = None
                anchor = actual
                return False
            stable_since = time.monotonic() if stable_since is None else stable_since
            return time.monotonic() - stable_since >= STATIONARY_SEC

        if not self.wait(stopped, SERVICE_TIMEOUT_SEC, enabled=True):
            raise ValueError("DI1 descent Stop did not confirm stationary robot")

    def confirm_stop(self, future):
        """Confirm an operator Stop without allowing cancellation to hide its result."""
        deadline = time.monotonic() + SERVICE_TIMEOUT_SEC
        while not future.done():
            self.node.feedback_snapshot(enabled=False)
            if time.monotonic() >= deadline:
                raise ValueError("Stop acknowledgement timeout")
            time.sleep(0.02)
        result = future.result()
        if result is None or result.res != 0:
            raise ValueError("Stop rejected")
        before = self.node.feedback_snapshot(enabled=False)["sequence"]
        stable_since, anchor = None, None

        def stopped(snapshot):
            nonlocal stable_since, anchor
            feed = snapshot["feed"]
            actual = pose_matrix(feed["tool_vector_actual"])
            stationary = (snapshot["sequence"] > before and not feed["isRunQueuedCmd"]
                          and not feed["RunningStatus"] and feed["robot_mode"] == 5
                          and anchor is not None and pose_reached(anchor, actual))
            if not stationary:
                stable_since = None
                anchor = actual
                return False
            stable_since = time.monotonic() if stable_since is None else stable_since
            return time.monotonic() - stable_since >= STATIONARY_SEC

        deadline = time.monotonic() + SERVICE_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if stopped(self.node.feedback_snapshot(enabled=False)):
                self.moving = False
                self.node.events.record("INFO", "stop_confirmed", "Robot stationary after Stop")
                return
            time.sleep(0.02)
        raise ValueError("Stop did not confirm stationary robot")

    def output(self, channel, active, *, require_clear=False):
        def clear(snapshot):
            if require_clear and snapshot["feed"]["digital_input_bits"] & 1:
                raise ValueError("Unexpected/late DI1 before vacuum OFF; no candidate retry")

        clear(self.node.feedback_snapshot(enabled=True))
        before = self.node.feedback_snapshot(enabled=True)["sequence"]
        self.call("DO", index=channel, status=int(active), time=0,
                  progress=clear if require_clear else None)
        mask = 1 << (channel - 1)

        def confirmed(snapshot):
            clear(snapshot)
            return (snapshot["sequence"] > before and
                    bool(snapshot["feed"]["digital_outputs"] & mask) == active)

        if not self.wait(confirmed, SERVICE_TIMEOUT_SEC, enabled=True):
            raise ValueError(f"DO{channel} feedback confirmation timeout")

    def sensor(self, active, timeout, *, settling_sec):
        stable_since = None

        def confirmed(s):
            nonlocal stable_since
            if bool(s["feed"]["digital_input_bits"] & 1) != active:
                stable_since = None
                return False
            stable_since = time.monotonic() if stable_since is None else stable_since
            return time.monotonic() - stable_since >= settling_sec

        return (confirmed(self.node.feedback_snapshot(enabled=True))
                or self.wait(confirmed, timeout, enabled=True))

    def stop(self):
        # Cancellation cannot prevent the safety stop. Never disable or release a held item.
        try:
            self.node.check_command_owner("Stop")
        except (ValueError, RuntimeError) as exc:
            self.node.events.record("ERROR", "stop_owner_invalid", str(exc))
            return None
        client = self.clients["Stop"]
        if client.service_is_ready():
            future = client.call_async(self.types["Stop"].Request())
            self.node.events.record("WARNING", "stop_requested", "Stop sent; confirmation pending")
            return future
        self.node.events.record("ERROR", "stop_unavailable", "Cannot confirm robot stopped")
        return None

    def close(self):
        for client in self.clients.values():
            self.node.destroy_client(client)
        self.clients.clear()
