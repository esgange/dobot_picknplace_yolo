"""Placement geometry and retained release progress, owned by the controller."""

from dataclasses import dataclass, field, replace
import math

import numpy as np

from .errors import FeedbackFailure, HeldUnknown
from .kinematics import pose_matrix
from .motion import (Target, gripper_neutral_events, pose_reached, rigid_matrix,
                     vacuum_neutral_events)


def validate_target(x_mm, y_mm, rotation_deg):
    values = (x_mm, y_mm, rotation_deg)
    if any(type(v) not in (float, int) or not math.isfinite(v) for v in values):
        raise ValueError("Placement X, Y and Rotation must be finite numbers")
    if x_mm <= 0 or y_mm <= 0:
        raise ValueError("Placement X and Y must be positive millimetres")
    if not -180 <= rotation_deg <= 180:
        raise ValueError("Placement Rotation must be between -180 and +180 degrees")
    return tuple(map(float, values))


def place_targets(detect_matrix, surface, settings, rotation_deg):
    """Saved tool attitude plus local Z spin; base-Z standoff and vertical retract."""
    validate_target(1., 1., rotation_deg)
    matrix = rigid_matrix(detect_matrix, "Recorded Tray Detect Pose").copy()
    angle = math.radians(rotation_deg)
    c, s = math.cos(angle), math.sin(angle)
    matrix[:3, :3] = matrix[:3, :3] @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    surface = np.asarray(surface, dtype=float)
    if surface.shape != (3,) or not np.isfinite(surface).all():
        raise ValueError("Placement surface must be a finite base-frame point")
    motion = settings["motion"]
    if any(not math.isfinite(v) or v < 0 for v in motion.values()):
        raise ValueError("Invalid taught placement heights")
    release_z = surface[2] + motion["standoff_height"] / 1000
    pre_z = release_z + motion["prepick_height"] / 1000
    final_z = pre_z + motion["retract_height"] / 1000
    result = []
    for name, z, rate in (("place_pre", pre_z, "travel_percent"),
                          ("place_release", release_z, "approach_percent"),
                          ("place_retract", pre_z, "retract_percent"),
                          ("place_clearance", final_z, "travel_percent")):
        point = matrix.copy()
        point[:3, 3] = [surface[0], surface[1], z]
        result.append(Target(name, point, settings["speed"][rate],
                             settings["acceleration"][rate]))
    return tuple(result)


@dataclass
class PlacementOperation:
    x_mm: float
    y_mm: float
    rotation_deg: float
    phase: str = "OBSERVE"
    plan: tuple = ()
    release_step: int = 0
    pending_outputs: dict = field(default_factory=dict)
    pulse_dispatched: bool = False

    @property
    def needs_recovery(self):
        return self.phase in ("RELEASING", "RELEASED")

    def run(self, node):
        node.wait_for_resume()
        config = node.configuration
        config.validate_sources(node.root)
        if self.phase == "OBSERVE":
            node._preflight_item_state(True)
            node._execute_tray_position()
            surface = node.trays.request(config, self.x_mm, self.y_mm)
            if not node.hardware.home_already_reached(config.tray.detect_joints):
                raise FeedbackFailure("Robot moved away from Tray Detect Pose during observation")
            self.plan = place_targets(config.tray.detect_matrix, surface,
                                      config.profile, self.rotation_deg)
            self.phase = "APPROACH"
        if self.phase == "APPROACH":
            node._preflight_item_state(True)
            node.operation_progress("PLACE_APPROACH", "Pre-place then depth-based release pose",
                                    waypoint=self.plan[1].name)
            node.hardware.move_batch(
                self.plan[:2], batch_name="place_approach", require_suction=True,
                terminal_stable_sec=config.profile["timing"]["pick_settling"])
            with node.managed.lock:
                node.wait_for_resume()
                config.validate_sources(node.root)
                node._preflight_item_state(True)
                self.phase = "RELEASING"
                # From this stationary release intent onwards, DI1 loss is expected.
                node.holding_item = False
        if self.phase == "RELEASING":
            self.release(node)
        if self.phase == "RELEASED":
            self.retract(node)

    def release(self, node):
        node.configuration.validate_sources(node.root)
        current = node.hardware.current_pose()
        if not pose_reached(current, self.plan[1].matrix):
            raise FeedbackFailure("Interrupted release pose changed; refusing to descend or release")
        node.operation_progress("PLACE_RELEASE", "Opening fingers and releasing at taught standoff")
        outputs = ((2, False), (13, False), (14, True), (1, False))
        while self.release_step < len(outputs):
            node.wait_for_resume()
            channel, active = outputs[self.release_step]
            node.hardware.output(channel, active)
            self.pending_outputs.pop(channel, None)
            self.release_step += 1
        node.monitor.wait(
            lambda s: bool(s.feed["digital_input_bits"] & (1 << 11)),
            5., cancel=node.cancel_requested, pause=node.pause_requested,
            require_enabled=True, description="DI12 fingers fully open for placement")
        node.wait_for_resume()
        if not self.pulse_dispatched:
            node.hardware.exhaust_pulse()
        else:
            # A Stop can interrupt pulse confirmation. Never fire it a second time.
            # A fresh, stationary released state is required before any retract.
            node.monitor.wait(
                lambda s: not s.feed["digital_input_bits"] & 1
                and s.feed["digital_outputs"] & ((1 << 13) | (1 << 12) | 3) == 1 << 13,
                5., cancel=node.cancel_requested, pause=node.pause_requested,
                require_enabled=True, description="Previously issued placement pulse completed")
        self.pending_outputs.clear()
        self.phase = "RELEASED"
        session = node.managed.session
        if session is not None and session.held_index is not None:
            session.set_state(session.held_index, "PLACED")
            session.held_index = None
        node.events.record("INFO", "item_placed", "Placement release confirmed",
                           x_mm=self.x_mm, y_mm=self.y_mm, rotation_deg=self.rotation_deg)

    def retract(self, node):
        node.configuration.validate_sources(node.root)
        node.operation_progress("PLACE_RETRACT", "Retracting vertically from the released item")
        current = node.hardware.current_pose()
        if node.monitor.snapshot(require_enabled=True).feed["digital_input_bits"] & 1:
            raise HeldUnknown("DI1 HIGH after placement release; retract blocked")
        retreat = []
        origin = current
        for target in self.plan[2:]:
            matrix = origin.copy()
            matrix[2, 3] = max(origin[2, 3], target.matrix[2, 3])
            if matrix[2, 3] > origin[2, 3] + 1e-9:
                retreat.append(replace(target, matrix=matrix))
                origin = matrix
        if retreat:
            retreat[0] = replace(retreat[0], motion_io=(
                gripper_neutral_events(0) + vacuum_neutral_events(0)))
            node.hardware.move_batch(tuple(retreat), batch_name="place_retract",
                                     forbid_suction=True, confirmed_start_pose=current)
        else:
            for channel in (2, 14, 13, 1):
                node.hardware.output(channel, False, require_clear=True)
        self.phase = "DONE"

    def check_paused(self, node, sample):
        if sample.feed["isRunQueuedCmd"] or sample.feed["RunningStatus"]:
            raise FeedbackFailure("Unexpected movement during placement Pause")
        if not pose_reached(pose_matrix(sample.feed["tool_vector_actual"]),
                            node.managed.parked_pose):
            raise FeedbackFailure("Placement pose changed while paused")
        bits = sample.feed["digital_outputs"]
        if any(bool(bits & (1 << (ch - 1))) != value
               for ch, value in node.expected_outputs.items()):
            raise FeedbackFailure("Placement outputs changed while paused")
        if self.phase in ("OBSERVE", "APPROACH"):
            node._preflight_item_state(True)
        elif self.phase in ("RELEASED", "DONE") and sample.feed["digital_input_bits"] & 1:
            raise HeldUnknown("Unexpected suction after confirmed placement release")

    def handle_pause(self, node):
        """Stop in place; uncertain release must not trigger the bin put-back path."""
        managed = node.managed
        managed.executing = True
        try:
            node.hardware.ensure_no_pending_response()
            node.hardware.confirm_stop(node.hardware.request_stop("Placement Pause"),
                                       allow_suction_loss=self.needs_recovery)
            managed.parked_pose = node.hardware.current_pose()
            node.configuration.validate_sources(node.root)
            node._transition("PAUSED", "Placement stopped in place; Continue resumes it")
            while True:
                node.raise_if_cancelled()
                self.check_paused(node, node.monitor.snapshot(require_enabled=True))
                with managed.lock:
                    if managed.resume.is_set():
                        if self.phase == "APPROACH":
                            self.phase = "OBSERVE"  # Reobserve after an interrupted approach.
                        managed._clear_request()
                        node._transition("PLACING", "Continuing placement")
                        return
                node.wait_control(.02)
        finally:
            with managed.lock:
                if managed.executing:
                    managed._clear_request()
