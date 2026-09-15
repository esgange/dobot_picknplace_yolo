"""Canonical Dobot service transport; instantiated only while Live is ON."""

import math
import re
import time

import numpy as np

from .kinematics import pose_matrix, pose_values
from .motion import pose_reached


SERVICE_TIMEOUT_SEC = 5.0
MOTION_TIMEOUT_SEC = 30.0
STATIONARY_SEC = 0.3


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

    def call(self, name, *, monitor_suction=False, require_clear=False, **fields):
        self.node.check_cancelled()
        self.node.check_command_owner(name)
        self.node.feedback_snapshot(enabled=False)
        client = self.clients[name]
        if not client.service_is_ready():
            raise ValueError(f"Required canonical {name} service unavailable")
        request = self.types[name].Request(**fields)
        future = client.call_async(request)
        if name in ("MovLIO", "RelMovLUser"):
            # A late accepted command after cancellation must receive another safety Stop.
            # This is containment, not a retry of motion or a claim of confirmed stopping.
            future.add_done_callback(self._late_motion_ack)
        deadline = time.monotonic() + SERVICE_TIMEOUT_SEC
        while not future.done():
            self.node.check_cancelled()
            snapshot = self.node.feedback_snapshot(enabled=False)
            if require_clear and snapshot["feed"]["digital_input_bits"] & 1:
                raise ValueError("Unexpected DI1 during descent before suction; Stop required")
            if monitor_suction and snapshot["feed"]["digital_input_bits"] & 1:
                self._interrupt_suction()
            if time.monotonic() >= deadline:
                raise ValueError(f"{name} response timeout; no retry")
            time.sleep(0.02)
        result = future.result()
        self.node.check_cancelled()
        self.node.feedback_snapshot(enabled=False)
        if result is None or result.res != 0:
            raise ValueError(f"{name} failed: {None if result is None else result.res}")
        self.node.events.record("INFO", "robot_service", name, fields=fields)
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
        for name in ("StopMoveJog", "DisableRobot"):
            try:
                self.call(name)
                if name == "DisableRobot" and not self.wait(
                        lambda s: s["feed"]["robot_mode"] == 4, SERVICE_TIMEOUT_SEC):
                    raise ValueError("DisableRobot did not confirm Disabled mode")
            except ValueError as exc:
                self.node.events.record("WARNING", "startup_best_effort", str(exc), service=name)
        self.call("EnableRobot")
        if not self.wait(lambda s: s["feed"]["robot_mode"] == 5 and s["enabled"],
                         SERVICE_TIMEOUT_SEC):
            raise ValueError("EnableRobot did not confirm Enabled mode")
        self.call("SpeedFactor", ratio=100)
        self.call("Tool", index=0)
        self.call("SetTool", index=1, value="{0,0,0,0,0,0}")
        self.call("CP", r=100)
        self.node.set_execution_state("READY", "Initialized at SpeedFactor 100%, Tool 0, CP 100%")

    def current_pose(self):
        snapshot = self.node.feedback_snapshot(enabled=True)
        if snapshot["feed"]["isRunQueuedCmd"] or snapshot["feed"]["robot_mode"] != 5:
            raise ValueError("Current-pose acquisition requires stationary enabled robot")
        result = self.call("GetPose", user=0, tool=0)
        matrix = pose_matrix(robot_values(result.robot_return, "GetPose"))
        modeled = self.node.kinematics.forward(self.node.current_joints())
        if not pose_reached(modeled, matrix, translation_m=0.002, rotation_deg=0.5):
            raise ValueError("CR10 FK does not match GetPose(user=0,tool=0); motion blocked")
        return matrix

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
                      mdis=[], param_value=params,
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

    def output(self, channel, active):
        self.node.feedback_snapshot(enabled=True)
        before = self.node.feedback_snapshot(enabled=True)["sequence"]
        self.call("DO", index=channel, status=int(active), time=0)
        mask = 1 << (channel - 1)
        if not self.wait(lambda s: s["sequence"] > before and
                         bool(s["feed"]["digital_outputs"] & mask) == active,
                         SERVICE_TIMEOUT_SEC, enabled=True):
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
