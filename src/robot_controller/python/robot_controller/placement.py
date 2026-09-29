"""Placement geometry and retained release progress, owned by the controller."""

from dataclasses import dataclass, field, replace
import math

import numpy as np

from .errors import FeedbackFailure, HeldUnknown
from .kinematics import pose_matrix
from .motion import (Target, gripper_neutral_events, pose_reached, rigid_matrix,
                     vacuum_neutral_events, gripper_open_events, vacuum_exhaust_events)


def validate_target(x_mm, y_mm, rotation_deg):
    values = (x_mm, y_mm, rotation_deg)
    if any(type(v) not in (float, int) or not math.isfinite(v) for v in values):
        raise ValueError("Placement X, Y and Rotation must be finite numbers")
    if x_mm <= 0 or y_mm <= 0:
        raise ValueError("Placement X and Y must be positive millimetres")
    if not -180 <= rotation_deg <= 180:
        raise ValueError("Placement Rotation must be between -180 and +180 degrees")
    return tuple(map(float, values))


def place_targets(detect_matrix, surface, settings, rotation_deg, home_matrix):
    """Saved tool attitude plus local Z spin; drop at the tray-relative pre-pick Z."""
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
    # The first item-pick approach is its target XY at Home Z, then pre-pick.
    # Mirror those heights over the tray, using pre-pick as the drop endpoint.
    release_z = (surface[2] + motion["standoff_height"] / 1000
                 + motion["prepick_height"] / 1000)
    home = rigid_matrix(home_matrix, "Home")
    pre_z = home[2, 3]
    if pre_z <= release_z:
        raise ValueError("Placement approach at Home Z must be above the drop height")
    result = []
    for name, z, rate in (("place_pre", pre_z, "travel_percent"),
                          ("place_release", release_z, "approach_percent"),
                          ("place_retract", pre_z, "retract_percent")):
        point = matrix.copy()
        point[:3, 3] = [surface[0], surface[1], z]
        events = ()
        if name == "place_release":
            events = gripper_open_events(80) + vacuum_exhaust_events(80)
        elif name == "place_retract":
            events = gripper_neutral_events(50) + vacuum_neutral_events(50)
        result.append(Target(name, point, settings["speed"][rate],
                             settings["acceleration"][rate], motion_io=events))
    result.append(Target("place_home", home.copy(),
                         settings["speed"]["travel_percent"],
                         settings["acceleration"]["travel_percent"]))
    return tuple(result)


@dataclass
class PlacementOperation:
    x_mm: float
    y_mm: float
    rotation_deg: float
    require_held_item: bool = True
    phase: str = "OBSERVE"
    plan: tuple = ()
    pending_outputs: dict = field(default_factory=dict)
    release_issued: bool = False
    neutral_issued: bool = False
    release_confirmed: bool = False
    history_sequence: int = 0
    initial_outputs: int = 0
    observing: bool = False

    @property
    def needs_recovery(self):
        return self.release_issued and self.phase != "DONE"

    def preflight(self, node):
        if self.require_held_item:
            node._preflight_item_state(True)
        else:
            # Attended placement still requires healthy, enabled robot feedback.
            node.monitor.snapshot(require_enabled=True)

    def begin_queue(self, node):
        sample = node.monitor.snapshot(require_enabled=True)
        mask = (1 << 13) | (1 << 12) | 3
        self.initial_outputs = sample.feed["digital_outputs"] & mask
        self.history_sequence = sample.sequence
        self.release_issued = self.neutral_issued = False
        self.release_confirmed = False
        self.observing = True

    def issued(self, index):
        # Called before dispatch: a timed command with an uncertain response may
        # have executed. Never assume it is safe to repeat its release.
        if index == 1:
            self.release_issued = True
            self.phase = "RELEASING"
        elif index == 2:
            self.neutral_issued = True

    def observe(self, node, sample):
        """Record available release evidence; never gate the queue on DI1/DI12."""
        if not self.observing:
            return
        with node.managed.lock:
            mask = (1 << 13) | (1 << 12) | 3
            # History is diagnostic only. A gap or unobserved release interval
            # must not interrupt the admitted approach/drop/retract/Home queue.
            for sequence, _timer, outputs, inputs in node.monitor.output_history(
                    self.history_sequence):
                if sequence > sample.sequence:
                    break
                bits = outputs & mask
                self.history_sequence = sequence
                if self.release_issued and bits != self.initial_outputs:
                    node.holding_item = False
                    if not self.release_confirmed:
                        self.phase = "RELEASING"
                if (self.release_issued and bits == ((1 << 13) | 1)
                        and bits != self.initial_outputs
                        and inputs & (1 << 11) and not inputs & 1):
                    if not self.release_confirmed:
                        self.release_confirmed = True
                        self.phase = "RELEASED"
                        node.events.record(
                            "INFO", "placement_release_observed", "Queued release feedback observed")
                node.expected_outputs.update({ch: bool(bits & (1 << (ch - 1)))
                                              for ch in (1, 2, 13, 14)})

    def complete(self, node, sample):
        # The transport calls this only after physical arrival at final Home.
        # No intermediate release state or full-open sensor is a success gate.
        self.observe(node, sample)
        if (sample.feed["digital_input_bits"] & 1
                or sample.feed["digital_outputs"] & ((1 << 13) | (1 << 12) | 3)):
            raise FeedbackFailure(
                "Home reached; final placement outputs must be neutral and DI1 LOW")
        node.holding_item = False
        node.expected_outputs.update(dict.fromkeys((1, 2, 13, 14), False))
        session = node.managed.session
        if session is not None and session.held_index is not None:
            if session.attempts[session.held_index - 1].state == "HELD":
                session.set_state(session.held_index, "PLACED")
            session.held_index = None
        node.events.record(
            "INFO", "placement_home_completed", "Placement queue completed at Home",
            release_feedback_observed=self.release_confirmed)
        self.phase = "DONE"
        self.observing = False

    def run(self, node):
        node.wait_for_resume()
        config = node.configuration
        config.validate_sources(node.root)
        if self.phase == "DONE":
            return
        if self.phase in ("RELEASING", "RELEASED"):
            self.recover(node)
            return
        # An interrupted queue with unchanged outputs can be reobserved.
        self.observing = False
        self.preflight(node)
        node._execute_tray_position()
        surface = node.trays.request(config, self.x_mm, self.y_mm,
                                     require_held_item=self.require_held_item)
        if not node.hardware.home_already_reached(config.tray.detect_joints):
            raise FeedbackFailure("Robot moved away from Tray Detect Pose during observation")
        self.plan = place_targets(config.tray.detect_matrix, surface,
                                  config.profile, self.rotation_deg, config.home_matrix)
        config.validate_sources(node.root)
        self.preflight(node)
        self.phase = "APPROACH"
        self.begin_queue(node)
        node.operation_progress("PLACE_QUEUE", "Queueing pre-place, release, retract and Home",
                                waypoint="place_home")
        node.hardware.move_batch(self.plan, batch_name="place_to_home", placement=self)

    def recover(self, node):
        # An interrupted release is never repeated and never descends back to the
        # tray. Only confirmed release can resume an upward retreat and Home.
        self.observe(node, node.monitor.snapshot(require_enabled=True))
        if not self.release_confirmed:
            raise FeedbackFailure("Placement release unconfirmed; cannot repeat release or descend")
        if node.monitor.snapshot(require_enabled=True).feed["digital_input_bits"] & 1:
            raise HeldUnknown("DI1 HIGH after placement release; recovery blocked")
        self.neutral_issued = True
        self.observing = False
        for channel in (2, 14, 1, 13):
            node.hardware.output(channel, False, require_clear=True)
        current = node.hardware.current_pose()
        target = self.plan[2]
        rise = current.copy()
        rise[2, 3] = max(current[2, 3], target.matrix[2, 3])
        retreat = []
        if rise[2, 3] > current[2, 3] + 1e-9:
            retreat.append(replace(target, matrix=rise, motion_io=()))
        retreat.append(self.plan[3])
        node.hardware.move_batch(tuple(retreat), batch_name="place_recovery_home",
                                 forbid_suction=True, confirmed_start_pose=current)
        self.complete(node, node.monitor.snapshot(require_enabled=True))

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
        self.observe(node, sample)
        if self.phase in ("OBSERVE", "APPROACH"):
            self.preflight(node)
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
