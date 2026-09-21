"""Single-owner Pause parking, Continue replanning and intentional item return."""

import threading

from .errors import (CommandRejected, FeedbackFailure, HeldUnknown,
                     ManagedInterruption, PausedItemDropped, ReturnedToHome)
from .kinematics import pose_matrix
from .motion import PickExecutor, pose_reached
from .pick_session import (approach_from_safety, return_targets, safety_target,
                           transit_from_safety)


class ManagedControl:
    def __init__(self, node):
        self.node = node
        self.lock = threading.RLock()
        self.kind = None
        self.previous = None
        self.executing = False
        self.parking_held = False
        self.drop_pending = False
        self.thread = None
        self.resume = threading.Event()
        self.session = None
        self.parked_pose = None

    def checkpoint(self):
        self.node.raise_if_cancelled()
        if self.executing:
            if self.drop_pending:
                raise PausedItemDropped("Suction lost during Pause parking")
        elif self.kind is not None:
            raise ManagedInterruption("Managed interruption requested")

    def request(self, kind):
        node = self.node
        with self.lock:
            if not node.startup_complete or node.machine.state not in (
                    "READY", "HOLDING", "HOMING", "PICKING", "PAUSED"):
                raise CommandRejected(
                    "Pause/return requires a started idle, Home or Pick controller")
            if kind == "return" and (self.session is None
                                     or self.session.held_index is None
                                     or not node.holding_item):
                raise CommandRejected("Return requires the trusted held candidate's source pose")
            if self.kind is not None:
                if kind == "return" and node.machine.state == "PAUSED":
                    self.kind = kind
                    return
                raise CommandRejected("Pause/return is already pending")
            if node.operation_lock.locked() and node.active_action not in ("home", "pick"):
                raise CommandRejected("Cannot interrupt lifecycle/settings operation")
            idle = not node.operation_lock.locked()
            previous = node.machine.state
            if idle:
                node._begin_operation("pause")
            self.previous = previous
            self.kind = kind
            self.resume.clear()
            node.pause_event.set()
            node._transition("PAUSING", "Stopping queue before " + kind)
            try:
                node.hardware.request_stop("Managed " + kind + " requested")
            except Exception:
                node.cancel_event.set()
                node.startup_complete = False
                self.kind = None
                node.pause_event.clear()
                node._transition("FAULT", "Managed Stop dispatch failed")
                if idle:
                    node._end_operation()
                raise
            if idle:
                self.start_idle_worker()

    def start_idle_worker(self):
        self.thread = threading.Thread(target=self._idle_worker, daemon=True)
        self.thread.start()

    def continue_operation(self):
        with self.lock:
            if self.node.machine.state != "PAUSED" or self.kind != "pause":
                raise CommandRejected("Continue requires completed Pause parking")
            self.node.configuration.validate_sources(self.node.root)
            sample = self.node.monitor.snapshot(require_enabled=True)
            self._check_parked(sample)
            if self.drop_pending or (
                    self.node.holding_item and not sample.feed["digital_input_bits"] & 1):
                raise CommandRejected("Suction lost; item return must finish before Continue")
            self.resume.set()

    def observe(self, sample):
        """Latch drop without abandoning an outstanding service response."""
        with self.lock:
            if ((self.parking_held or self.kind == "pause" and (
                    not self.executing or self.node.machine.state == "PAUSED"))
                    and self.node.holding_item and not sample.feed["digital_input_bits"] & 1):
                if not self.drop_pending:
                    self.drop_pending = True
                    self.node.hardware.request_stop("Suction lost during managed Pause rise")
                return True
        return False

    def _candidate(self):
        if self.session is None or self.session.held_index is None:
            raise HeldUnknown("No retained source pose for held item")
        return self.session.attempts[self.session.held_index - 1]

    def _mark_drop(self):
        attempt = self._candidate()
        if attempt.state == "HELD":
            self.session.set_state(self.session.held_index, "DROPPED")
        self.node.holding_item = False
        self.drop_pending = False
        self.parking_held = False

    def _confirm_interruption(self):
        node = self.node
        node.hardware.ensure_no_pending_response()
        node.configuration.validate_sources(node.root)
        # Every admitted command has replied before this final containment Stop.
        future = node.hardware.request_stop("Confirm discarded queue before managed motion")
        node.hardware.confirm_stop(future, allow_suction_loss=node.holding_item)
        node.raise_if_cancelled()
        sample = node.monitor.snapshot(require_enabled=True)
        bits = sample.feed["digital_outputs"]
        if (bits & 1 and bits & (1 << 12)) or (bits & 2 and bits & (1 << 13)):
            raise FeedbackFailure("Opposing outputs active at managed Stop")
        acquired = bool(sample.feed["digital_input_bits"] & 1)
        if not node.holding_item and acquired:
            eligible = getattr(node.hardware, "acquisition_eligible", False)
            active = [index for index, attempt in enumerate(
                self.session.attempts if self.session else (), 1) if attempt.state == "ACTIVE"]
            if eligible and bits & (1 << 12) and len(active) == 1:
                self.session.set_state(active[0], "HELD")
                node.holding_item = True
            elif not getattr(node.hardware, "late_miss_suction", False):
                raise HeldUnknown("DI1 active without an eligible acquisition during Pause")
            # A latched failed attempt's late DI1 is never a new acquisition.
        if node.holding_item and (not acquired or self.drop_pending):
            self._mark_drop()
        if not node.holding_item:
            node.expected_outputs.update({channel: bool(bits & (1 << (channel - 1)))
                                          for channel in (1, 2, 13, 14)})
        node.hardware.acquisition_eligible = False
        return pose_matrix(sample.feed["tool_vector_actual"])

    def _rise(self, current, *, holding, preserve_outputs=False):
        node = self.node
        target = safety_target(current, node.configuration.home_matrix,
                               node.configuration.profile)
        if target.matrix[2, 3] > current[2, 3] + 1e-9:
            node.operation_progress("PAUSE_SAFETY", "Rising vertically to Home Z")
            node.hardware.move_batch((target,), batch_name="pause_safety",
                                     require_suction=holding,
                                     preserve_outputs=preserve_outputs,
                                     confirmed_start_pose=current)
            return target.matrix.copy()
        return current

    def _neutral(self):
        for channel in (2, 14, 13, 1):
            self.node.hardware.output(channel, False)

    def _park(self, current):
        node = self.node
        self.parking_held = node.holding_item
        if self.session:
            self.session.interrupt_active()
        if not node.holding_item:
            self._neutral()
        try:
            current = self._rise(current, holding=node.holding_item)
        finally:
            self.parking_held = False
        if not node.holding_item and self.session and node.active_action == "pick":
            index = self.session.next_pending
            self.session.parked_index = index
            if index is not None:
                plan = self.session.attempts[index - 1].plan
                node.operation_progress("PAUSE_TRANSIT",
                                        f"Parking above candidate {index} at safety Z",
                                        candidate_index=index)
                node.hardware.move_batch((transit_from_safety(current, plan),),
                                         batch_name="pause_to_next_transit",
                                         confirmed_start_pose=current)

    def _put_back(self, *, dropped):
        node = self.node
        attempt = self._candidate()
        plan = attempt.plan
        release, retreat = return_targets(plan)
        node._transition("RETURNING_ITEM", "Returning last held candidate to its release pose")
        node.configuration.validate_sources(node.root)
        current = node.hardware.current_pose()
        current = self._rise(current, holding=not dropped, preserve_outputs=dropped)
        approach = approach_from_safety(current, plan)
        if abs(release.matrix[2, 3] - plan[2].matrix[2, 3]) > 1e-9:
            approach += (release,)
        else:
            approach = approach[:-1] + (release,)
        node.hardware.move_batch(approach, batch_name="return_item_to_release",
                                 require_suction=not dropped,
                                 preserve_outputs=dropped,
                                 confirmed_start_pose=current)
        node.operation_progress("RELEASE", "Opening fingers and pulsing exhaust for 50 ms")
        # Intentional release owns the upcoming DI1 loss. Keep source context
        # until the entire return completes, but no longer assert held suction.
        node.holding_item = False
        node.hardware.output(2, False)
        node.hardware.output(13, False)
        node.hardware.output(14, True)
        node.hardware.output(1, False)
        node.hardware.exhaust_pulse()
        node._execute_home(preceding=retreat, require_suction=False, forbid_suction=True,
                           confirmed_start_pose=release.matrix,
                           queue_through_home=True, batch_name="return_item_to_home")
        if not dropped:
            self.session.set_state(self.session.held_index, "RETURNED")
        self.session.held_index = None
        self.session.parked_index = None
        node.events.record("INFO", "item_return_completed",
                           "Release sequence and Home confirmed; physical placement not measured",
                           candidate_id=attempt.identifier, dropped=dropped)

    def _check_parked(self, sample):
        node = self.node
        if sample.feed["isRunQueuedCmd"] or sample.feed["RunningStatus"]:
            raise FeedbackFailure("Unexpected motion while parked")
        if self.parked_pose is not None and not pose_reached(
                pose_matrix(sample.feed["tool_vector_actual"]), self.parked_pose,
                translation_m=0.001, rotation_deg=0.5):
            raise FeedbackFailure("Robot pose changed while parked")
        for channel, expected in node.expected_outputs.items():
            if bool(sample.feed["digital_outputs"] & (1 << (channel - 1))) != expected:
                raise FeedbackFailure(f"Parked output DO{channel} changed unexpectedly")
        if not node.holding_item and sample.feed["digital_input_bits"] & 1:
            raise HeldUnknown("Unexpected DI1 while parked without an item")

    def handle(self):
        node = self.node
        self.executing = True
        try:
            current = self._confirm_interruption()
            dropped = bool(self.session and self.session.held_index is not None
                           and self._candidate().state == "DROPPED")
            if self.kind == "return" or dropped:
                self._put_back(dropped=dropped)
            elif self.previous != "READY":
                try:
                    self._park(current)
                except PausedItemDropped:
                    self.parking_held = False
                    self.drop_pending = False
                    self._confirm_interruption()
                    # DI1 can bounce back after the latched loss; the return is
                    # still mandatory and never converted back into success.
                    self._mark_drop()
                    self._put_back(dropped=True)
            if self.kind == "return":
                node._transition("READY", "Item returned; stopped operation completed at Home")
                raise ReturnedToHome("Item returned and robot Home")
            while True:
                self.parked_pose = pose_matrix(
                    node.monitor.snapshot(require_enabled=True).feed["tool_vector_actual"])
                node._transition("PAUSED", "Parked; Continue resumes remaining operation")
                while True:
                    node.raise_if_cancelled()
                    sample = node.monitor.snapshot(require_enabled=True)
                    self._check_parked(sample)
                    if self.drop_pending or (
                            node.holding_item and not sample.feed["digital_input_bits"] & 1):
                        self.resume.clear()
                        self._confirm_interruption()
                        self._mark_drop()
                        self._put_back(dropped=True)
                        if self.kind == "return":
                            node._transition("READY", "Dropped-item return completed at Home")
                            raise ReturnedToHome("Item return routine completed at Home")
                        break
                    with self.lock:
                        returning = self.kind == "return"
                        if returning:
                            node._transition("RETURNING_ITEM", "Controlled return accepted")
                        elif self.resume.is_set():
                            node.raise_if_cancelled()
                            if self.session:
                                self.session.resuming = True
                            restored = ("PICKING" if node.active_action == "pick" else
                                        "HOMING" if node.active_action == "home" else
                                        "HOLDING" if node.holding_item else "READY")
                            # Clear before exposing the resumed state. A new
                            # request after this lock releases belongs to the
                            # resumed owner and must survive this handle's finally.
                            self._clear_request()
                            node._transition(
                                restored, "Continue accepted; replanning from parked pose")
                            return
                    if returning:
                        self._put_back(dropped=False)
                        node._transition(
                            "READY", "Item returned; stopped operation completed at Home")
                        raise ReturnedToHome("Item returned and robot Home")
                    node.wait_control(0.02)
        finally:
            with self.lock:
                if self.executing:
                    self._clear_request()

    def _clear_request(self):
        self.executing = False
        self.parking_held = False
        self.drop_pending = False
        self.parked_pose = None
        self.kind = None
        self.resume.clear()
        self.node.pause_event.clear()

    def _idle_worker(self):
        try:
            self.handle()
            if (self.previous == "HOLDING" and not self.node.holding_item
                    and self.session is not None and self.session.resuming):
                # Explicit Continue after a completed Pick's paused-drop return
                # owns the remaining batch and never requests replacement poses.
                self.node.active_action = "pick"
                self.node._transition(
                    "PICKING", "Continuing retained candidates after item return")
                while True:
                    try:
                        result = PickExecutor(self.node.hardware, finish_home=True).run(
                            [attempt.plan for attempt in self.session.attempts],
                            self.node.configuration.profile, session=self.session,
                            check=lambda _index: self.node.configuration.validate_sources(
                                self.node.root), return_home=self.node._execute_home,
                            progress=self.node._candidate_progress,
                            holding_changed=lambda value: setattr(
                                self.node, "holding_item", value))
                        with self.lock:
                            self.checkpoint()
                            self.node._transition(
                                "HOLDING" if result["picked"] else "READY",
                                "Retained candidate operation completed at Home")
                        break
                    except ManagedInterruption:
                        self.handle()
        except ReturnedToHome:
            pass
        except Exception as exc:
            self.node._contain_queue_control_failure("Managed pause/return", exc)
        finally:
            self.node._end_operation()
