"""Cancel-and-Home recovery, followed by an explicit gripper reset at Home."""

from dataclasses import dataclass, field

from .errors import FeedbackFailure, HeldSuctionLost, HeldUnknown, UNKNOWN_ITEM_GUIDANCE
from .motion import Target


GRIP_CHANNELS = (1, 2, 13, 14)
GRIP_MASK = sum(1 << (channel - 1) for channel in GRIP_CHANNELS)


@dataclass
class HomeRecovery:
    trusted_held: bool
    release_confirmed: bool
    outputs: dict = field(default_factory=dict)
    holding: bool = False
    motion_started: bool = False
    relaxing: bool = False
    pending_outputs: dict = field(default_factory=dict)

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
            node.managed._clear_request()
            if session is not None:
                session.cancel_remaining()
            node.events.record(
                "WARNING", "recovery_action_cancelled",
                "Interrupted action cancelled; preserve grip to Home, then relax gripper",
                placement_release_confirmed=released, trusted_held_source=trusted)
            return cls(trusted, released)

    def capture(self, node, sample):
        """Only called after Stop proves a stationary empty queue and stable I/O."""
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
                    # No release proof: cancel it, never claim PLACED/RETURNED.
                    session.set_state(index, "CANCELED")
                session.held_index = None
        node.events.record(
            "INFO", "recovery_grip_preserved", "Fresh stopped gripper state adopted",
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
        if self.relaxing:
            return sample  # Suction may decay while the explicitly requested reset runs.
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
                          tuple(config.home_joints))
            node.operation_progress("RECOVERY_HOME", "Moving to taught Home; grip preserved",
                                    waypoint=home.name)
            self.motion_started = True
            node.hardware.move_batch((home,), batch_name="recovery_home",
                                     confirmed_start_pose=current, **policy)
        self.check(node.monitor.snapshot(require_enabled=True))

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
        sample = self.check(node.monitor.snapshot(require_enabled=True))
        if sample.feed["digital_input_bits"] & 1:
            raise HeldUnknown("Gripper outputs relaxed at Home, but DI1 suction is still HIGH")
        if not node.hardware.home_already_reached(node.configuration.home_joints):
            raise FeedbackFailure("Home position lost while resetting gripper")
        self.holding = False
        node.events.record("INFO", "recovery_gripper_relaxed", "Gripper reset confirmed at Home",
                           digital_outputs=sample.feed["digital_outputs"],
                           digital_input_bits=sample.feed["digital_input_bits"])
