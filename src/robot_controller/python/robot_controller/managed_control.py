"""Single-owner Pause parking, Continue replanning and intentional item return."""

from dataclasses import replace
import threading

from .errors import (CommandRejected, FeedbackFailure, HeldSuctionLost, HeldUnknown,
                     ManagedInterruption, PausedItemDropped, ReturnedToHome)
from .motion import CARTESIAN_POSITION_TOLERANCE_M, PickExecutor, pick_tray_target, pose_reached
from .item_return import ItemReturnOperation, ReturnPickBridge
from .pick_session import safety_target, transit_from_safety


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
        self.return_progress = None
        self.held_loss_pending = False
        self.containing_loss = False

    def checkpoint(self):
        self.node.raise_if_cancelled()
        if self.held_loss_pending and not self.containing_loss:
            raise HeldSuctionLost("Confirmed held suction loss interrupted the operation")
        if self.executing:
            if self.drop_pending:
                raise PausedItemDropped("Suction lost during Pause parking")
        elif self.kind is not None:
            raise ManagedInterruption("Managed interruption requested")

    def can_return_item(self):
        """Shared status/admission guard for trusted-source return, including acquisition Pause."""
        node = self.node
        if not node.holding_item or self.session is None or self.session.held_index is None:
            return False
        if node.active_action != "place":
            return True
        placement = getattr(node, "placement", None)
        return bool(placement is not None and placement.acquisition_paused
                    and node.machine.state == "PAUSED" and self.kind == "pause"
                    and self._candidate().state == "HELD" and not self.drop_pending)

    def request(self, kind):
        node = self.node
        with self.lock:
            if not node.startup_complete or node.machine.state not in (
                    "READY", "HOLDING", "HOMING", "PICKING", "TRAY_POSITIONING",
                    "PLACING", "PAUSED"):
                raise CommandRejected(
                    "Pause/return requires a started idle or motion controller")
            if kind == "return" and node.active_action == "place" and not self.can_return_item():
                raise CommandRejected(
                    "Bin return during Place requires paused failed tray acquisition "
                    "and a trusted held item; otherwise Stop and Recover")
            if kind == "return" and (self.session is None
                                     or self.session.held_index is None
                                     or not node.holding_item):
                raise CommandRejected("Return requires the trusted held candidate's source pose")
            if self.kind is not None:
                if kind == "return" and node.machine.state == "PAUSED":
                    self.kind = kind
                    return
                raise CommandRejected("Pause/return is already pending")
            if node.operation_lock.locked() and node.active_action not in (
                    "home", "pick", "tray_position", "place"):
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
            reason = self.continue_block_reason(sample)
            if reason:
                raise CommandRejected(reason)
            if node_placement := getattr(self.node, "placement", None):
                node_placement.check_paused(self.node, sample)
            else:
                self._check_parked(sample)
            if self.drop_pending or (
                    self.node.holding_item and not sample.suction_present):
                raise CommandRejected("Suction lost; item return must finish before Continue")
            self.resume.set()

    def continue_block_reason(self, sample):
        """Read-only eligibility for status and Continue admission; never requests motion."""
        with self.lock:
            node = self.node
            if node.machine.state != "PAUSED" or self.kind != "pause":
                return "Continue requires completed Pause parking"
            if self.resume.is_set():
                return "Continue is already pending"
            if self.drop_pending or (node.holding_item and not sample.suction_present):
                return "Suction lost; item return must finish before Continue"
            placement = getattr(node, "placement", None)
            if placement is not None:
                if placement.release_issued and not placement.release_confirmed:
                    return "Placement release unconfirmed; use Stop, then Recover"
                if placement.phase in ("OBSERVE", "APPROACH") and not node._perception_ready(
                        "place"):
                    return "Arm Tray Teach or start Tray Detect with exactly one provider"
            try:
                if placement is not None:
                    placement.check_parked_feedback(node, sample)
                else:
                    self._check_parked(sample)
            except (FeedbackFailure, HeldUnknown) as exc:
                return str(exc)
            return ""

    def observe(self, sample):
        """Latch drop without abandoning an outstanding service response."""
        with self.lock:
            self.note_suction_loss(sample)
            if ((self.parking_held or self.kind == "pause" and (
                    not self.executing or self.node.machine.state == "PAUSED"))
                    and self.node.holding_item and not sample.suction_present):
                if not self.drop_pending:
                    self.drop_pending = True
                    self.node.hardware.request_stop("Suction lost during managed Pause rise")
                return True
        return False

    def interrupt_held_loss(self, sample):
        """Feedback-side latch/Stop; the existing operation owner performs recovery."""
        with self.lock:
            if (self.containing_loss or self.kind is not None or self.executing
                    or self.node.machine.state in ("FAULT", "STOPPING", "INACTIVE")
                    or self.node.active_action == "recover"
                    or not self.node.holding_item or sample.suction_present
                    or self.session is None or self.session.held_index is None):
                return
            self.note_suction_loss(sample)
            if not self.held_loss_pending:
                self.held_loss_pending = True
                self.node.hardware.request_stop("Confirmed held suction loss")

    def observe_continuous(self, sample):
        placement = getattr(self.node, "placement", None)
        if placement is not None and placement.observing:
            # FeedInfo may latch/send Stop, but operation exceptions belong to
            # the command owner, including after an explicit cancellation.
            placement.observe(self.node, sample, raise_on_loss=False)
        self.interrupt_held_loss(sample)

    def note_suction_loss(self, sample):
        """Retain the source and loss even if DI1 returns before put-back."""
        with self.lock:
            if (not self.node.holding_item or sample.suction_present
                    or self.session is None or self.session.held_index is None):
                return
            attempt = self._candidate()
            if attempt.state == "HELD":
                self.session.set_state(self.session.held_index, "DROPPED")
                self.node.events.record(
                    "WARNING", "held_suction_lost",
                    "DI1 loss confirmed; physical holding uncertain, source and outputs retained",
                    candidate_id=attempt.identifier, sequence=sample.sequence,
                    controller_timer=sample.feed.get("controller_timer"),
                    digital_input_bits=sample.feed["digital_input_bits"],
                    digital_outputs=sample.feed["digital_outputs"])

    def recovery_return_needed(self):
        placement = getattr(self.node, "placement", None)
        if placement is not None and placement.needs_recovery:
            return True
        self.note_suction_loss(self.node.monitor.snapshot(require_enabled=False))
        with self.lock:
            return bool(self.return_progress is not None or (
                self.session is not None and self.session.held_index is not None
                and self._candidate().state == "DROPPED"))

    def motion_admitted(self, target):
        self.session.admitted(target)

    def run_pick(self, plans, *, check, departure=(), departure_pose=None,
                 placement_bridge=None):
        """Keep confirmed held loss inside the owning Pick and its saved batch."""
        node = self.node
        while True:
            if departure and isinstance(self.return_progress, ItemReturnOperation):
                placement_bridge = ReturnPickBridge(node, self.return_progress, departure_pose)
            try:
                return PickExecutor(node.hardware, finish_home=True).run(
                    plans, node.configuration.profile,
                    session=(placement_bridge.next_session if placement_bridge is not None
                             else self.session), check=check,
                    tray_target=pick_tray_target(
                        node.configuration.tray, node.configuration.profile),
                    return_home=node._execute_home, progress=node._candidate_progress,
                    holding_changed=lambda value: setattr(node, "holding_item", value),
                    departure=departure, departure_pose=departure_pose,
                    placement_bridge=placement_bridge)
            except HeldSuctionLost:
                if placement_bridge is not None and not placement_bridge.completed:
                    # The queued next Pick does not own the old held item/batch.
                    # Auto Run must discard that speculative session first.
                    raise
                placement_bridge = None
                returned = self._return_after_suction_loss()
                if returned is None:
                    return {"picked": False, "candidate": None, "holding_item": False}
                departure, departure_pose = returned
                self.session.resuming = False
                node._transition("RETURNING_ITEM",
                                 "Queueing return followed by next retained candidate")

    def _return_after_suction_loss(self):
        self.node.raise_if_cancelled()
        self.containing_loss = True
        try:
            return self._contain_and_return_loss()
        finally:
            self.containing_loss = False

    def _contain_and_return_loss(self):
        node = self.node
        node.raise_if_cancelled()
        attempt = self._candidate()
        if not node.holding_item or attempt.state not in ("HELD", "DROPPED"):
            raise HeldUnknown("Automatic put-back requires a trusted held source")
        self.session.set_state(self.session.held_index, "DROPPED")
        self.held_loss_pending = True
        future = node.hardware.request_stop("Held suction lost; discard return queue for put-back")
        node.hardware._acknowledge_stop(future, check_cancel=True)
        node.hardware.resolve_interrupted_commands()
        # A command submitted concurrently with the first Stop must also be
        # discarded. All replies precede this final Stop/empty-queue barrier.
        future = node.hardware.request_stop("Discard resolved interrupted queue", fresh=True)
        node.configuration.validate_sources(node.root)
        node.hardware.confirm_stop(future, allow_suction_loss=True)
        self.checkpoint()
        sample = node.monitor.snapshot(require_enabled=True)
        # Stop may reconcile an admitted timed transition. No unrelated output
        # change may be adopted when automatic put-back takes ownership.
        bits = sample.feed["digital_outputs"]
        for channel, active in node.expected_outputs.items():
            if bool(bits & (1 << (channel - 1))) != active:
                raise FeedbackFailure(f"Held-item output DO{channel} changed before put-back")
        if (bits & 1 and bits & (1 << 12)) or (bits & 2 and bits & (1 << 13)):
            raise FeedbackFailure("Opposing outputs active before automatic put-back")
        with self.lock:
            self.checkpoint()
            node._transition("RETURNING_ITEM", "Suction lost; automatically returning saved item")
            self.executing = True
        try:
            node.events.record(
                "WARNING", "automatic_item_return_started",
                "Stop confirmed; putting back item before continuing saved batch",
                candidate_id=attempt.identifier)
            placement = getattr(node, "placement", None)
            if placement is not None:
                placement.close_pending()
                placement.observing = False
                node.placement = None
            self._mark_drop()
            self.held_loss_pending = False
            return self._put_back(dropped=True, continue_candidates=True, home_if_exhausted=False)
        finally:
            with self.lock:
                self.executing = False

    def continue_after_loss(self):
        """Return to the saved source and resume only its ordered eligible poses."""
        departure = self._return_after_suction_loss()
        if departure is None:
            self.node._transition("READY", "Dropped item returned; saved batch exhausted")
            return False
        self.session.resuming = False
        self.node.active_action = "pick"
        self.node._transition("RETURNING_ITEM",
                              "Queueing return followed by next retained candidate")

        def check(_index):
            self.node.raise_if_cancelled()
            self.node.configuration.validate_sources(self.node.root)

        result = self.run_pick([attempt.plan for attempt in self.session.attempts],
                               check=check, departure=departure[0],
                               departure_pose=departure[1])
        self.node._transition("HOLDING" if result["picked"] else "READY",
                              "Retained batch Pick complete after dropped-item return")
        return result["picked"]

    def recover_item_and_continue(self):
        """Explicit recovery owns put-back, then the remaining retained batch."""
        self.node.raise_if_cancelled()
        self.executing = True
        try:
            if self.return_progress is None:
                self._mark_drop()
            departure = self._put_back(dropped=True, continue_candidates=True)
        finally:
            self._clear_request()
        if departure is None:
            self.node.raise_if_cancelled()
            self.node._transition(
                "READY", "Item returned; no eligible candidates remain; robot Home")
            return
        retreat, origin = departure
        self.session.resuming = False
        self.node.active_action = "pick"
        self.node._transition("PICKING", "Item released; continuing retained candidates")
        while True:
            try:
                result = self.run_pick(
                    [attempt.plan for attempt in self.session.attempts],
                    check=lambda _index: self.node.configuration.validate_sources(self.node.root),
                    departure=retreat, departure_pose=origin)
                with self.lock:
                    self.checkpoint()
                    self.node.events.record(
                        "INFO", "recovery_pick_completed",
                        "Retained candidate operation completed",
                        picked=result["picked"], candidate=result["candidate"])
                    self.node._transition(
                        "HOLDING" if result["picked"] else "READY",
                        "Retained candidate operation finished at "
                        + ("Tray Detect" if result["picked"] else "Home"))
                    return
            except ManagedInterruption:
                retreat, origin = (), None
                try:
                    self.handle()
                except ReturnedToHome:
                    return

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
                plan = self.session.attempts[active[0] - 1].plan
                node.hardware.defer_pickup_drop(plan[4].matrix, sample)
                self.session.set_state(active[0], "HELD")
                node.holding_item = True
            elif not getattr(node.hardware, "late_miss_suction", False):
                raise HeldUnknown("DI1 active without an eligible acquisition during Pause")
            # A latched failed attempt's late DI1 is never a new acquisition.
        if node.holding_item and (not sample.suction_present or self.drop_pending):
            self._mark_drop()
        if not node.holding_item:
            node.expected_outputs.update({channel: bool(bits & (1 << (channel - 1)))
                                          for channel in (1, 2, 13, 14)})
        node.hardware.acquisition_eligible = False
        return node.hardware.pose_from_snapshot(sample)

    def _rise(self, current, *, holding, preserve_outputs=False, full_speed=False):
        node = self.node
        target = safety_target(current, node.configuration.home_matrix,
                               node.configuration.profile)
        if full_speed:
            target = replace(target, speed_percent=100)
        if target.matrix[2, 3] > current[2, 3] + CARTESIAN_POSITION_TOLERANCE_M:
            node.operation_progress("PAUSE_SAFETY", "Rising vertically to Home Z")
            node.hardware.move_batch((target,), batch_name="pause_safety",
                                     require_suction=holding,
                                     preserve_outputs=preserve_outputs,
                                     confirmed_start_pose=current)
            return target.matrix.copy()
        return current

    def _neutral(self, *, require_clear=False):
        for channel in (2, 14, 13, 1):
            self.node.hardware.output(channel, False, require_clear=require_clear)

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
            index = self.session.next_eligible
            self.session.parked_index = index
            if index is not None:
                plan = self.session.attempts[index - 1].plan
                node.operation_progress("PAUSE_TRANSIT",
                                        f"Parking above candidate {index} at safety Z",
                                        candidate_index=index)
                node.hardware.move_batch((transit_from_safety(current, plan),),
                                         batch_name="pause_to_next_transit",
                                         confirmed_start_pose=current)

    def _put_back(self, *, dropped, continue_candidates=False, home_if_exhausted=True):
        node = self.node
        attempt = self._candidate()
        if self.return_progress is None:
            self.return_progress = ItemReturnOperation(
                index=self.session.held_index, phase="APPROACH",
                dropped=dropped or attempt.state == "DROPPED",
                continue_candidates=continue_candidates)
        operation = self.return_progress
        continuing = (operation.continue_candidates
                      and self.session.next_eligible is not None)
        if continuing:
            current = operation.prepare(node, finish_home=False)
            # The shared return prefix and next candidate are dispatched in one
            # ordered group. No release/retract arrival wait separates them.
            return operation.plan, current
        operation.run(node, finish_home=home_if_exhausted)
        return None

    def _check_parked(self, sample):
        node = self.node
        if sample.feed["isRunQueuedCmd"] or sample.feed["RunningStatus"]:
            raise FeedbackFailure("Unexpected motion while parked")
        if self.parked_pose is not None and not pose_reached(
                node.hardware.pose_from_snapshot(sample), self.parked_pose,
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
            if self.return_progress is not None:
                # Pause during a released item's retreat finishes at Home and
                # stays paused; Continue can then own the remaining candidates.
                self.return_progress.continue_candidates = False
            if self.kind == "return" or dropped or self.return_progress is not None:
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
                self.parked_pose = node.hardware.pose_from_snapshot(
                    node.monitor.snapshot(require_enabled=True))
                node._transition("PAUSED", "Parked; Continue resumes remaining operation")
                while True:
                    node.raise_if_cancelled()
                    sample = node.monitor.snapshot(require_enabled=True)
                    self._check_parked(sample)
                    if self.drop_pending or (
                            node.holding_item and not sample.suction_present):
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
                                        "TRAY_POSITIONING"
                                        if node.active_action == "tray_position" else
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
                        result = self.run_pick(
                            [attempt.plan for attempt in self.session.attempts],
                            check=lambda _index: self.node.configuration.validate_sources(
                                self.node.root))
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

    def start_loss_worker(self):
        def recover():
            try:
                self.continue_after_loss()
            except Exception as exc:
                self.node._contain_queue_control_failure("Held-item drop recovery", exc)
            finally:
                self.node._end_operation()
        self.thread = threading.Thread(target=recover, daemon=True)
        self.thread.start()
