"""Placement geometry and retained release progress, owned by the controller."""

from dataclasses import dataclass, field, replace
import math

import numpy as np

from .errors import FeedbackFailure, HeldUnknown, ManagedInterruption, ReturnedToHome
from .motion import pose_reached, rigid_matrix
from .release import ReleaseQueue, release_targets
from .tray_client import TrayAcquisitionExhausted, TrayAttempts


TRAY_SPEED_PERCENT = 100


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
    """Saved tool attitude plus local Z spin; explicit drop height above tray surface."""
    validate_target(1., 1., rotation_deg)
    matrix = rigid_matrix(detect_matrix, "Recorded Tray Detect Pose").copy()
    angle = math.radians(rotation_deg)
    c, s = math.cos(angle), math.sin(angle)
    matrix[:3, :3] = matrix[:3, :3] @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    surface = np.asarray(surface, dtype=float)
    if surface.shape != (3,) or not np.isfinite(surface).all():
        raise ValueError("Placement surface must be a finite base-frame point")
    motion = settings["motion"]
    if any(type(v) not in (int, float) or not math.isfinite(v) or v < 0
           for v in motion.values()):
        raise ValueError("Invalid taught placement heights")
    if "trayplace_height" not in motion:
        raise ValueError("Placement requires an explicit trayplace_height")
    release_z = surface[2] + motion["trayplace_height"] / 1000
    matrix[:3, 3] = [surface[0], surface[1], release_z]
    return release_targets(matrix, settings, home_matrix, prefix="place")


@dataclass
class PlacementOperation(ReleaseQueue):
    x_mm: float
    y_mm: float
    rotation_deg: float
    require_held_item: bool = True
    tray_attempts: TrayAttempts = field(default_factory=TrayAttempts)
    pending_motion: object = None
    acquisition_failure: str = ""
    returning_to_bin: bool = False

    @property
    def acquisition_paused(self):
        return bool(self.acquisition_failure and self.phase == "OBSERVE"
                    and not self.release_issued and not self.returning_to_bin)

    @property
    def needs_recovery(self):
        return self.release_issued and self.phase != "DONE"

    def preflight(self, node):
        if self.require_held_item:
            node._preflight_item_state(True)
        else:
            # Attended placement still requires healthy, enabled robot feedback.
            node.monitor.snapshot(require_enabled=True)

    def check_observation(self, node):
        node.wait_for_resume()
        self.preflight(node)
        if not node.hardware.home_already_reached(node.configuration.tray.detect_joints):
            raise FeedbackFailure("Not at Tray Detect position; tray detection blocked")

    def finish_pending(self, node):
        pending, self.pending_motion = self.pending_motion, None
        if pending is not None:
            node.hardware.finish_batch(pending)

    def close_pending(self):
        pending, self.pending_motion = self.pending_motion, None
        if pending is not None:
            pending.close()

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
        node.operation_progress("TRAY_POSITION", "Checking saved Tray Detect position")
        if not node.hardware.home_already_reached(config.tray.detect_joints):
            node._execute_tray_position()
        node.hardware.wait_tray_position(config.tray.detect_joints)
        try:
            surface = node.trays.request(config, self.x_mm, self.y_mm,
                                         require_held_item=self.require_held_item,
                                         attempts=self.tray_attempts,
                                         check_state=lambda: self.check_observation(node))
        except TrayAcquisitionExhausted as exc:
            if getattr(node, "auto_run", None) is not None:
                # Auto Run reports partial quantity; it never grants extra retries.
                raise
            with node.managed.lock:
                node.wait_for_resume()
                self.acquisition_failure = str(exc)
                node.managed.request("pause")
            node.events.record("WARNING", "tray_acquisition_paused", str(exc))
            raise ManagedInterruption(str(exc)) from exc
        self.check_observation(node)
        self.plan = place_targets(config.tray.detect_matrix, surface,
                                  config.profile, self.rotation_deg, config.home_matrix)
        config.validate_sources(node.root)
        self.preflight(node)
        self.phase = "APPROACH"
        self.begin_queue(node)
        node.operation_progress("PLACE_QUEUE", "Queueing pre-place, release and final retract",
                                waypoint="place_retract")
        self.pending_motion = node.hardware.move_batch(
            self.plan, batch_name="place_to_retract", placement=self, queue_only=True)

    def recover(self, node):
        # An interrupted release is never repeated and never descends back to the
        # tray. Only confirmed release can resume an upward retreat.
        self.observe(node, node.monitor.snapshot(require_enabled=True))
        if not self.release_confirmed:
            raise FeedbackFailure(
                "Placement release unconfirmed; cannot repeat release or descend")
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
        if retreat:
            node.hardware.move_batch(tuple(retreat), batch_name="place_recovery_retract",
                                     forbid_suction=True, confirmed_start_pose=current)
        self.complete(node, node.monitor.snapshot(require_enabled=True))

    def check_parked_feedback(self, node, sample):
        """Read-only paused pose/output check shared with operator eligibility."""
        if sample.feed["isRunQueuedCmd"] or sample.feed["RunningStatus"]:
            raise FeedbackFailure("Unexpected movement during placement Pause")
        if not pose_reached(node.hardware.pose_from_snapshot(sample),
                            node.managed.parked_pose):
            raise FeedbackFailure("Placement pose changed while paused")
        bits = sample.feed["digital_outputs"]
        if any(bool(bits & (1 << (ch - 1))) != value
               for ch, value in node.expected_outputs.items()):
            raise FeedbackFailure("Placement outputs changed while paused")

    def check_paused(self, node, sample):
        self.check_parked_feedback(node, sample)
        self.observe(node, sample)
        if self.phase in ("OBSERVE", "APPROACH"):
            self.preflight(node)
            if self.acquisition_paused and node.holding_item:
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
            if self.acquisition_paused:
                if not node.hardware.home_already_reached(node.configuration.tray.detect_joints):
                    raise FeedbackFailure("Not at Tray Detect position; acquisition Pause blocked")
                choices = "Use Place Item (Retry)"
                if node.holding_item:
                    choices += " or Return Item"
                message = (f"{self.acquisition_failure}; paused at Tray Detect. "
                           f"{choices}")
            else:
                message = "Placement stopped in place; Continue resumes it"
            node._transition("PAUSED", message)
            if self.acquisition_paused:
                node.operation_progress("TRAY_ACQUISITION_PAUSED", message)
            while True:
                node.raise_if_cancelled()
                self.check_paused(node, node.monitor.snapshot(require_enabled=True))
                with managed.lock:
                    returning = managed.kind == "return"
                    if returning:
                        if not self.acquisition_paused:
                            raise FeedbackFailure("Bin return requires paused tray acquisition")
                        # Return owns the saved pick source and its I/O history.
                        # Placement never issued motion/release in this branch.
                        self.returning_to_bin = True
                        managed.resume.clear()
                    elif managed.resume.is_set():
                        if self.acquisition_paused:
                            self.tray_attempts = TrayAttempts()
                            self.acquisition_failure = ""
                            node.events.record(
                                "INFO", "tray_acquisition_retry",
                                "Operator requested 3 new attempts")
                        if self.phase == "APPROACH":
                            self.phase = "OBSERVE"  # Reobserve after an interrupted approach.
                        managed._clear_request()
                        node._transition("PLACING", "Continuing placement")
                        return
                if returning:
                    node._preflight_item_state(True)
                    managed._put_back(dropped=False)
                    node._transition(
                        "READY", "Item returned to bin; robot Home; placement canceled")
                    raise ReturnedToHome("Item returned to bin and robot Home; placement canceled")
                node.wait_control(.02)
        finally:
            with managed.lock:
                if managed.executing:
                    managed._clear_request()
