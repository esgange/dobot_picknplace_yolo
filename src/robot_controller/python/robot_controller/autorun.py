"""One controller owner for counted cycles and ordered placement-to-pick queues."""

from concurrent.futures import Future
import threading

from .errors import CommandRejected, FeedbackFailure, OperationCanceled
from .pick_session import PickSession, return_targets
from .placement import PlacementOperation, validate_target
from .recovery import GRIP_MASK


def validate_quantity(value):
    if type(value) is not int or not 1 <= value <= 10000:
        raise ValueError("Auto Run quantity must be an integer from 1 to 10000")
    return value


class CandidatePrefetch:
    """At most one read-only request; never runs a hardware command."""

    def __init__(self, node, configuration, save_debug_images):
        self.cancel = threading.Event()
        self.future = Future()

        def acquire():
            try:
                batch = node.candidates.request(
                    configuration, save_debug_images=save_debug_images,
                    cancel=lambda: self.cancel.is_set() or node.cancel_requested())
                if self.cancel.is_set() or node.cancel_requested():
                    raise OperationCanceled("Auto Run prefetch canceled")
                self.future.set_result(batch)
            except Exception as exc:
                self.future.set_exception(exc)

        self.thread = threading.Thread(target=acquire, name="auto_run_item_poses", daemon=True)
        self.thread.start()

    def close(self):
        self.cancel.set()
        self.thread.join(timeout=5.)
        if self.thread.is_alive():
            raise FeedbackFailure("Auto Run detector request did not finish cancellation")


class PlacementBridge:
    """Retain the old source until ordered execution crosses into the appended Home."""

    def __init__(self, run, placement):
        self.run = run
        self.node = run.node
        self.placement = placement
        self.old_session = self.node.managed.session
        self.next_session = None
        self.origin = placement.plan[-1].matrix.copy()
        self.home_id = None
        self.accepted_sequence = None
        self.neutral_seen = False
        self.completed = False
        self.targets = []

    def accepted(self, index, response):
        if index == 0:
            self.home_id = self.node.hardware._motion_command_id(response)
            self.accepted_sequence = self.node.monitor.sequence

    def admitted(self, target):
        self.targets.append(target)
        if self.completed and self.next_session is not None:
            self.next_session.admitted(target)

    def observe(self, sample):
        if self.completed:
            return
        self.placement.observe(self.node, sample)
        for sequence, _timer, outputs, inputs in self.node.monitor.output_history(
                self.placement.queue_start_sequence):
            if sequence <= sample.sequence and not outputs & GRIP_MASK and not inputs & 1:
                self.neutral_seen = True
        if (self.home_id is not None and sample.sequence > self.accepted_sequence
                and sample.feed["currentCommandId"] >= self.home_id):
            if not self.neutral_seen:
                raise FeedbackFailure(
                    "Auto Run placement crossed into Home without neutral outputs and DI1 LOW")
            self._complete()

    def complete_idle(self):
        """The existing placement completion loop already validated final retract."""
        if self.placement.phase != "DONE":
            raise FeedbackFailure("Auto Run placement has not completed")
        self._complete()

    def _complete(self):
        if self.completed:
            return
        node = self.node
        if self.old_session is not None and self.old_session.held_index is not None:
            index = self.old_session.held_index
            if self.old_session.attempts[index - 1].state == "HELD":
                self.old_session.set_state(index, "PLACED")
            self.old_session.held_index = None
        self.placement.phase = "DONE"
        self.placement.observing = False
        node.holding_item = False
        node.placement = None
        self.completed = True
        if self.next_session is not None:
            node.managed.session = self.next_session
            for target in self.targets:
                self.next_session.admitted(target)
            node._transition("PICKING", "Placement passed; next queued Pick is executing")
        elif self.home_id is not None:
            node._transition("HOMING", "Placement passed; queued Home is executing")
        else:
            node._transition("READY", "Placement complete; waiting for next item poses")
        self.run.placement_completed()


class AutoRunOperation:
    def __init__(self, node, request):
        self.node = node
        self.quantity = validate_quantity(request.quantity)
        self.target = validate_target(request.x_mm, request.y_mm, request.rotation_deg)
        self.save_debug_images = bool(request.save_debug_images)
        self.configuration = node.configuration
        self.completed = 0
        self.prefetch = None
        self.bridge = None
        self.seen_batches = set()

    def placement_completed(self):
        self.completed += 1
        if self.completed > self.quantity:
            raise FeedbackFailure("Auto Run placement count exceeded its requested quantity")
        self.node.events.record(
            "INFO", "auto_run_placement_completed", "Placement sequence completed",
            completed=self.completed, requested=self.quantity,
            home_command_id=self.bridge.home_id if self.bridge is not None else None)
        self.node.operation_progress(
            "AUTO_COUNT", f"Auto Run: {self.completed}/{self.quantity} placements completed")

    def queued_home(self, bridge):
        targets = self.node._home_plan(bridge.origin)
        # Placement retract is already at Home Z. The first appended MovL must
        # expose a queue ID that proves execution passed every placement command.
        if len(targets) != 1 or targets[0].joints_rad is None or targets[0].motion_io:
            raise CommandRejected("Auto Run requires a direct joint Home after placement retract")
        return targets

    def finish_home(self, bridge):
        node = self.node
        node.active_action = "home"
        bridge.placement.close_pending()
        node.hardware.move_batch(
            self.queued_home(bridge), confirmed_start_pose=bridge.origin,
            placement_bridge=bridge, batch_name="auto_place_to_home")
        node._preflight_item_state(False)
        node._transition("READY", "Auto Run Home confirmed")
        self.bridge = None

    def _pick(self, batch=None, bridge=None):
        node, config = self.node, self.configuration
        node.active_action = "pick"
        attempted = 0
        empty_retry_used = False
        for attempt in range(1, 4):
            node.raise_if_cancelled()
            config.validate_sources(node.root)
            if bridge is None:
                node._transition("PICKING", "Auto Run: starting Pick")
                node._preflight_item_state(False)
            while True:
                node.raise_if_cancelled()
                if batch is None:
                    node.operation_progress("DETECT", f"Auto Run Pick attempt {attempt}/3")
                    batch = node.candidates.request(
                        config, save_debug_images=self.save_debug_images,
                        cancel=node.cancel_requested)
                node.raise_if_cancelled()
                if batch.identifier in self.seen_batches:
                    raise FeedbackFailure("Detector reused an earlier Auto Run batch ID")
                self.seen_batches.add(batch.identifier)
                if batch.candidates:
                    break
                if empty_retry_used:
                    node._transition(
                        "READY", "Auto Run stopped: no item poses; the one Home "
                        "acquisition retry has been used")
                    node.events.record(
                        "INFO", "pick_pose_acquisition_exhausted", node.machine.message,
                        batch_id=batch.identifier, attempted=attempted)
                    return False
                empty_retry_used = True
                node.events.record(
                    "INFO", "pick_empty_pose_retry",
                    "No item poses; confirm Home and retry acquisition once",
                    batch_id=batch.identifier)
                if bridge is not None:
                    self.finish_home(bridge)
                    bridge = None
                    node.active_action = "pick"
                    node._transition("PICKING", "Auto Run: retrying item poses at Home")
                else:
                    node._execute_home()
                batch = None
            if bridge is None:
                node._execute_home()
            node.candidate_total = len(batch.candidates)
            plans = node._plan_candidate_batch(batch)
            session = PickSession(
                [candidate.identifier for candidate in batch.candidates], plans,
                node._attempt_changed, previous_attempted=attempted)
            for plan in plans:
                return_targets(plan)
            extra = {}
            if bridge is not None:
                bridge.next_session = session
                extra = dict(queued_home=self.queued_home(bridge), placement_bridge=bridge)
            else:
                node.managed.session = session

            def check(_index):
                node.raise_if_cancelled()
                config.validate_sources(node.root)

            outcome = node.managed.run_pick(plans, check=check, **extra)
            attempted = session.attempted_count
            bridge = self.bridge = None
            if outcome["picked"]:
                node.holding_item = True
                node._transition("HOLDING", "Auto Run: item held at Tray Detect")
                return True
            node.events.record("INFO", "pick_batch_exhausted", "Auto Run Pick batch exhausted",
                               attempt=attempt, max_attempts=3, batch_id=batch.identifier,
                               attempted=attempted)
            batch = None
        node._transition("READY", "Auto Run stopped: all three Pick attempts exhausted")
        return False

    def run(self):
        node = self.node
        batch, bridge = None, None
        while self.completed < self.quantity:
            if not self._pick(batch, bridge):
                return False
            node.raise_if_cancelled()
            node.active_action = "place"
            node._transition("PLACING", "Auto Run: waiting for stable Tray Detect and tray pose")
            placement = PlacementOperation(*self.target, require_held_item=True)
            node.placement = placement
            if self.completed + 1 < self.quantity:
                # Pick has physically confirmed saved Tray Detect joints/idle.
                # Observe the bin while acquiring the tray, retaining the current
                # held source until the normal placement execution boundary.
                node.operation_progress(
                    "AUTO_PREFETCH", "Tray Detect confirmed; acquiring next bin candidate batch")
                self.prefetch = CandidatePrefetch(
                    node, self.configuration, self.save_debug_images)
            placement.run(node)
            self.bridge = bridge = PlacementBridge(self, placement)
            if self.completed + 1 == self.quantity:
                self.finish_home(bridge)
                return True
            # A ready result cannot append motion until placement is fully
            # admitted. This same owner retains accepted dashboard ordering.
            pending, placement.pending_motion = placement.pending_motion, None
            node.hardware.finish_batch(pending, handoff=self.prefetch.future.done)
            if placement.phase == "DONE":
                bridge.complete_idle()
                self.bridge = bridge = None
            while not self.prefetch.future.done():
                node.raise_if_cancelled()
                node._preflight_item_state(False)
                sample = node.monitor.snapshot(require_enabled=True)
                if not node.hardware._idle(sample):
                    raise FeedbackFailure("Unexpected motion while Auto Run waits for item poses")
                node.wait_control(.02)
            node.raise_if_cancelled()
            batch = self.prefetch.future.result()
            self.prefetch.close()
            self.prefetch = None
        return True

    def close(self):
        if self.prefetch is not None:
            self.prefetch.close()
            self.prefetch = None
        if self.node.placement is not None:
            self.node.placement.close_pending()
        if self.bridge is not None and self.bridge.next_session is not None:
            self.bridge.next_session.cancel_remaining()
