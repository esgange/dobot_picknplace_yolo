"""Cancel-and-Home recovery, suction check and optional saved-source return."""

from dataclasses import dataclass, field
import time

from .errors import FeedbackFailure, HeldSuctionLost, HeldUnknown, UNKNOWN_ITEM_GUIDANCE
from .motion import Target


GRIP_CHANNELS = (1, 2, 13, 14)
GRIP_MASK = sum(1 << (channel - 1) for channel in GRIP_CHANNELS)
SUCTION_TEST_SEC = 1.0


@dataclass
class HomeRecovery:
    trusted_held: bool
    release_confirmed: bool
    outputs: dict = field(default_factory=dict)
    holding: bool = False
    motion_started: bool = False
    relaxing: bool = False
    pending_outputs: dict = field(default_factory=dict)
    testing: bool = False
    test_started: bool = False
    suction_seen: bool = False
    awaiting_return: bool = False

    @classmethod
    def cancel_action(cls, node):
        with node.managed.lock:
            session = node.managed.session
            placement = getattr(node, "placement", None)
            progress = node.managed.return_progress
            released = bool(placement is not None and placement.release_confirmed
                            or progress is not None and progress.phase == "RELEASED")
            trusted = bool(session is not None and session.held_index is not None
                           and session.attempts[session.held_index - 1].state == "HELD"
                           and not released)
            if placement is not None:
                placement.observing = False
            node.placement = None
            node.managed.return_progress = None
            node.managed.held_loss_pending = False
            node.managed._clear_request()
            if session is not None:
                session.cancel_remaining()
            node.events.record(
                "WARNING", "recovery_action_cancelled",
                "Interrupted action cancelled; preserve grip to Home, then test suction",
                placement_release_confirmed=released, trusted_held_source=trusted)
            return cls(trusted, released)

    def capture(self, node, sample):
        """Adopt fresh I/O after accepted Stop; prove standstill after enabling."""
        bits = sample.feed["digital_outputs"]
        outputs = {ch: bool(bits & (1 << (ch - 1))) for ch in GRIP_CHANNELS}
        if outputs[1] and outputs[13] or outputs[2] and outputs[14]:
            raise FeedbackFailure("Recovery blocked: opposing gripper outputs are ON")
        detected = bool(sample.feed["digital_input_bits"] & 1)
        if detected:
            if self.release_confirmed:
                raise HeldUnknown("DI1 HIGH after confirmed release; clear item or obstruction")
            if not self.trusted_held or not outputs[13]:
                raise HeldUnknown(UNKNOWN_ITEM_GUIDANCE)
        elif sample.suction_present:
            raise FeedbackFailure("Recovery blocked: DI1 LOW is not yet stable; retry Recover")
        self.holding = detected
        self.relaxing = False
        self.testing = self.awaiting_return = False
        self.pending_outputs.clear()
        self.outputs = outputs
        node.expected_outputs.update(outputs)
        node.holding_item = detected
        if not detected:
            self.trusted_held = False
            session = node.managed.session
            if session is not None and session.held_index is not None:
                index = session.held_index
                if session.attempts[index - 1].state == "HELD":
                    # Retain the uncertain source until the active suction test finishes.
                    session.set_state(index, "CANCELED" if self.release_confirmed else "DROPPED")
                if self.release_confirmed:
                    session.held_index = None
        node.events.record(
            "INFO", "recovery_grip_preserved", "Fresh gripper state adopted after accepted Stop",
            digital_outputs=bits, digital_input_bits=sample.feed["digital_input_bits"],
            holding_item=detected)

    def check(self, sample):
        if not self.outputs:
            raise FeedbackFailure("Recovery gripper state has not been confirmed after Stop")
        bits = sample.feed["digital_outputs"]
        for channel, expected in self.outputs.items():
            actual = bool(bits & (1 << (channel - 1)))
            allowed = (expected, self.pending_outputs.get(channel, expected))
            if actual not in allowed:
                raise FeedbackFailure(f"Recovery blocked: preserved DO{channel} changed")
        self.observe_suction(sample)
        if self.relaxing or self.testing or self.awaiting_return:
            return sample  # Test/clearance/reset owns the expected DI1 changes at Home.
        if self.holding and not sample.suction_present:
            self.trusted_held = False
            raise HeldSuctionLost("Recovery stopped: held-item suction was lost")
        if not self.holding and sample.feed["digital_input_bits"] & 1:
            raise HeldUnknown("Recovery stopped: unexpected DI1 HIGH; check the gripper")
        return sample

    def run(self, node):
        node.raise_if_cancelled()
        config = node.configuration
        config.validate_sources(node.root)
        self.check(node.monitor.snapshot(require_enabled=True))

        current = node.hardware.current_pose()
        speed = config.profile["speed"]["travel_percent"]
        acceleration = config.profile["acceleration"]["travel_percent"]
        policy = dict(require_suction=self.holding, forbid_suction=not self.holding,
                      preserve_outputs=True)
        if current[2, 3] < config.home_matrix[2, 3] - 1e-9:
            rise = current.copy()
            rise[2, 3] = config.home_matrix[2, 3]
            target = Target("recovery_lift", rise, speed, acceleration, relative_z=True)
            node.operation_progress("RECOVERY_LIFT", "Lifting vertically to Home height",
                                    waypoint=target.name)
            self.motion_started = True
            node.hardware.move_batch((target,), batch_name="recovery_lift",
                                     confirmed_start_pose=current, **policy)
            # Physically confirm the lift before any lateral/rotational travel.
            current = None
        node.raise_if_cancelled()
        config.validate_sources(node.root)
        self.check(node.monitor.snapshot(require_enabled=True))
        if not node.hardware.home_already_reached(config.home_joints):
            home = Target("recovery_home", config.home_matrix.copy(), speed, acceleration,
                          tuple(config.home_joints), joint_motion=True)
            node.operation_progress("RECOVERY_HOME", "Moving to taught Home; grip preserved",
                                    waypoint=home.name)
            self.motion_started = True
            node.hardware.move_batch((home,), batch_name="recovery_home",
                                     confirmed_start_pose=current, **policy)
        self.check(node.monitor.snapshot(require_enabled=True))

    def has_return_source(self, node):
        session = node.managed.session
        return bool(not self.release_confirmed and session is not None
                    and session.held_index is not None
                    and session.attempts[session.held_index - 1].state in ("HELD", "DROPPED"))

    def observe_suction(self, sample):
        # Feed callbacks latch even a HIGH that clears before the operation wakes.
        if self.testing and sample.feed["digital_input_bits"] & 1:
            self.suction_seen = True

    def check_home(self, node, sample):
        from .hardware import HOME_JOINT_TOLERANCE_RAD

        self.check(sample)
        if (not sample.robot_enabled or sample.feed["isRunQueuedCmd"]
                or sample.feed["RunningStatus"]
                or max(abs(a - b) for a, b in zip(sample.joints, node.configuration.home_joints))
                > HOME_JOINT_TOLERANCE_RAD):
            raise FeedbackFailure("Recovery suction check requires stationary taught Home")
        return sample

    def test_suction(self, node):
        """A positive raw DI1 is an obstruction/item indication, never a new pickup."""
        from .hardware import OUTPUT_FEEDBACK_TIMEOUT_SEC

        node.raise_if_cancelled()
        node.configuration.validate_sources(node.root)
        self.check_home(node, node.monitor.snapshot(require_enabled=True))
        self.suction_seen = self.awaiting_return = False
        self.testing = True
        self.test_started = True
        node.operation_progress("RECOVERY_SUCTION_TEST", "At Home; testing suction for 1 second")
        node.events.record("INFO", "recovery_suction_test_started",
                           "Testing suction at Home; finger outputs preserved",
                           duration_sec=SUCTION_TEST_SEC)
        try:
            # Opposite OFF before suction ON; do not cycle an already-active vacuum.
            for channel, active in ((1, False), (13, True)):
                if self.outputs[channel] != active:
                    node.raise_if_cancelled()
                    self.pending_outputs[channel] = active
                    node.hardware.output(channel, active)
                    self.outputs[channel] = active
                    self.pending_outputs.pop(channel)
                    self.check(node.monitor.snapshot(require_enabled=True))
            initial = self.check_home(node, node.hardware.confirm_home(
                node.configuration.home_joints))
            started = time.monotonic()

            def observed(sample):
                self.check_home(node, sample)
                return self.suction_seen or (sample.sequence > initial.sequence
                                             and time.monotonic() - started >= SUCTION_TEST_SEC)

            node.monitor.wait(observed, SUCTION_TEST_SEC + OUTPUT_FEEDBACK_TIMEOUT_SEC,
                              cancel=node.cancel_requested, require_enabled=True,
                              description="Recovery suction test at stationary Home")
            node.raise_if_cancelled()
            node.configuration.validate_sources(node.root)
        finally:
            self.testing = False
        self.awaiting_return = self.suction_seen
        if not self.suction_seen:
            self.holding = self.trusted_held = node.holding_item = False
        node.events.record("WARNING" if self.suction_seen else "INFO",
                           "recovery_suction_test_result",
                           "Suction detected: item or obstruction" if self.suction_seen else
                           "Suction test clear", detected=self.suction_seen,
                           return_source_available=self.has_return_source(node))
        return self.suction_seen

    def park(self, node):
        with node.managed.lock:
            node.managed.kind = "pause"
            node.managed.executing = True
            node.managed.previous = "RECOVERING"
            node.managed.resume.clear()
            node.pause_event.set()
            node.startup_complete = True
            node.operation_progress("RECOVERY_SUCTION_BLOCKED", "Suction detected at Home")
            choice = ("Return Item to its saved bin position, or clear the obstruction and "
                      if self.has_return_source(node) else
                      "No saved return position; clear the item/obstruction and ")
            node._transition("PAUSED", "Suction detected: item or obstruction. " + choice
                             + "Continue to retest. The previous operation is cancelled.")

    def wait_for_choice(self, node):
        """Use the existing managed-operation worker; no service waits for an operator."""
        managed = node.managed
        try:
            while True:
                node.raise_if_cancelled()
                sample = self.check_home(node, node.monitor.snapshot(require_enabled=True))
                managed.note_suction_loss(sample)
                with managed.lock:
                    returning, retest = managed.kind == "return", managed.resume.is_set()
                if returning:
                    if not self.has_return_source(node):
                        raise HeldUnknown("Return Item requires a saved unreleased pickup source")
                    node.configuration.validate_sources(node.root)
                    node.recovery_home = None
                    managed._put_back(dropped=managed._candidate().state == "DROPPED")
                    node._transition("READY", "Item return completed at Home; recovery finished; "
                                     "previous operation cancelled")
                    return
                if retest:
                    managed.resume.clear()
                    if sample.feed["digital_input_bits"] & 1:
                        raise HeldUnknown("Suction remains HIGH; clear it before retesting")
                    self.holding = self.trusted_held = node.holding_item = False
                    node._transition("RECOVERING", "Retesting suction at Home")
                    if self.test_suction(node):
                        self.park(node)
                    else:
                        self.relax(node)
                        node.recovery_home = None
                        node._transition("READY", "Recovery completed at Home; suction test clear; "
                                         "gripper relaxed; previous operation cancelled")
                        return
                node.wait_control(.02)
        finally:
            with managed.lock:
                managed._clear_request()

    def relax(self, node):
        """Only neutralize after confirmed Home; never open fingers or pulse exhaust."""
        from .hardware import OUTPUT_FEEDBACK_TIMEOUT_SEC

        node.raise_if_cancelled()
        self.check(node.monitor.snapshot(require_enabled=True))
        if not node.hardware.home_already_reached(node.configuration.home_joints):
            raise FeedbackFailure("Gripper reset requires confirmed stationary Home")
        node.operation_progress("RECOVERY_RELAX", "At Home; relaxing gripper and clearing vacuum")
        self.relaxing = True
        self.trusted_held = False
        node.holding_item = False
        session = node.managed.session
        if session is not None and session.held_index is not None:
            index = session.held_index
            if session.attempts[index - 1].state == "HELD":
                session.set_state(index, "CANCELED")
            session.held_index = None
        for channel in GRIP_CHANNELS:
            node.raise_if_cancelled()
            self.pending_outputs[channel] = False
            node.hardware.output(channel, False)
            self.outputs[channel] = False
            self.pending_outputs.pop(channel)
            self.check(node.monitor.snapshot(require_enabled=True))
        if not node.hardware.sensor(False, OUTPUT_FEEDBACK_TIMEOUT_SEC):
            raise HeldUnknown("Gripper outputs relaxed at Home, but DI1 suction is still HIGH; "
                              "clear the item or sensor obstruction, then Recover again")
        sample = self.check(node.hardware.confirm_home(node.configuration.home_joints))
        if sample.feed["digital_input_bits"] & 1:
            raise HeldUnknown("Gripper outputs relaxed at Home, but DI1 suction is still HIGH")
        self.holding = False
        node.events.record("INFO", "recovery_gripper_relaxed", "Gripper reset confirmed at Home",
                           digital_outputs=sample.feed["digital_outputs"],
                           digital_input_bits=sample.feed["digital_input_bits"])
