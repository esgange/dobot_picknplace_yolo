"""Exercise the complete managed pause/return flow without a ROS or robot process."""

from types import SimpleNamespace
import threading

import numpy as np
import pytest

from robot_controller.errors import (CommandRejected, FeedbackFailure, HeldUnknown,
                                     ManagedInterruption, OperationCanceled, ReturnedToHome)
from robot_controller.kinematics import pose_matrix, pose_values
from robot_controller.managed_control import ManagedControl
from robot_controller.motion import PickExecutor, Target, pick_targets
from robot_controller.pick_session import PickSession, return_targets, safety_target
from robot_controller.state_machine import ControllerStateMachine


def profile(prepick=80.0):
    return {
        "pick_rotation": 0.0,
        "motion": {"standoff_height": 0.0, "prepick_height": prepick,
                   "retract_height": 50.0},
        "speed": {"travel_percent": 80, "approach_percent": 6, "retract_percent": 10},
        "acceleration": {"travel_percent": 80, "approach_percent": 50,
                         "retract_percent": 50},
        "gripper": {"use_grip": True, "grip_onpick": True},
        "timing": {"pick_settling": 0.1},
    }


def matrix(x=0., z=0.3):
    value = np.eye(4)
    value[0, 3], value[2, 3] = x, z
    return value


class Rig:
    def __init__(self, *, held=False, prepick=80., count=3):
        self.root = "."
        self.log = []
        self.machine = ControllerStateMachine(initial="PICKING")
        self.startup_complete = True
        self.holding_item = held
        self.expected_outputs = {1: False, 2: held, 13: held, 14: not held}
        self.feed = {"digital_input_bits": int(held),
                     "digital_outputs": sum(1 << (key - 1) for key, on
                                            in self.expected_outputs.items() if on),
                     "tool_vector_actual": pose_values(matrix()),
                     "isRunQueuedCmd": 0, "RunningStatus": 0}
        self.operation_lock = threading.Lock()
        self.operation_lock.acquire()
        self.active_action = "pick"
        self.pause_event = threading.Event()
        self.cancel_event = threading.Event()
        self.configuration = SimpleNamespace(
            home_matrix=matrix(z=0.8), profile=profile(prepick),
            validate_sources=lambda _root: None)
        self.events = SimpleNamespace(record=lambda *args, **kwargs: self.log.append(
            ("event", args, kwargs)))
        self.monitor = SimpleNamespace(snapshot=self.snapshot)
        self.hardware = FakeTransport(self)
        self.managed = ManagedControl(self)
        plans = [pick_targets(self.configuration.home_matrix, matrix(x=i * .1),
                              self.configuration.profile, i + 1) for i in range(count)]
        self.managed.session = PickSession([f"item{i}" for i in range(count)], plans)
        if held:
            self.managed.session.set_state(1, "ACTIVE")
            self.managed.session.set_state(1, "HELD")
        self.on_wait = lambda: self.managed.continue_operation()

    def snapshot(self, **_kwargs):
        return SimpleNamespace(feed=self.feed.copy(), sequence=1)

    def raise_if_cancelled(self):
        if self.cancel_event.is_set():
            raise OperationCanceled("direct Stop")

    def _transition(self, state, message):
        self.machine.transition(state, message)
        self.log.append(("state", state))

    def operation_progress(self, *args, **kwargs):
        self.managed.checkpoint()
        self.log.append(("phase", args, kwargs))

    def wait_control(self, _seconds):
        self.on_wait()

    def _execute_home(self, **kwargs):
        self.log.append(("home", kwargs))
        targets = kwargs.get("preceding", ()) + (
            Target("home", self.configuration.home_matrix, 80, 80),)
        forwarded = {key: value for key, value in kwargs.items()
                     if key not in ("preceding", "queue_through_home")}
        self.hardware.move_batch(targets, **forwarded)


class FakeTransport:
    def __init__(self, node):
        self.node = node
        self.acquisition_eligible = False
        self.late_miss_suction = False
        self.on_move = lambda _targets, _kwargs: None
        self.acquisitions = iter((False, False, False))

    def request_stop(self, reason):
        self.node.log.append(("stop", reason))
        return object()

    def confirm_stop(self, _future, **kwargs):
        self.node.log.append(("stop_confirmed", kwargs))

    def ensure_no_pending_response(self):
        pass

    def current_pose(self):
        return pose_matrix(self.node.feed["tool_vector_actual"])

    def sensor(self, active, _timeout):
        return bool(self.node.feed["digital_input_bits"] & 1) == active

    def output(self, channel, active):
        self.node.managed.checkpoint()
        self.node.log.append(("output", channel, active))
        mask = 1 << (channel - 1)
        self.node.feed["digital_outputs"] &= ~mask
        if active:
            self.node.feed["digital_outputs"] |= mask
        self.node.expected_outputs[channel] = active

    def exhaust_pulse(self):
        self.node.log.append(("pulse", 50))
        self.node.feed["digital_input_bits"] = 0

    def move_batch(self, targets, **kwargs):
        targets = tuple(targets)
        node = self.node
        node.managed.checkpoint()
        self.on_move(targets, kwargs)
        node.log.append(("move", tuple(target.name for target in targets), kwargs))
        for target in targets:
            if not node.managed.executing:
                node.managed.session.admitted(target)
            for event in target.motion_io:
                self.output(event.channel, event.active)
        node.feed["tool_vector_actual"] = pose_values(targets[-1].matrix)
        acquired = next(self.acquisitions) if kwargs.get("stop_on_suction") else False
        if acquired:
            node.feed["digital_input_bits"] = 1
        return (acquired, targets[-1].matrix.copy()) if kwargs.get(
            "return_terminal_pose") else acquired


def states(rig):
    return [attempt.state for attempt in rig.managed.session.attempts]


@pytest.mark.parametrize("stopped_z", [.3, 1.0])
@pytest.mark.parametrize("started", [False, True])
def test_pause_parks_above_same_candidate_at_safety_z(stopped_z, started):
    rig = Rig()
    rig.feed["tool_vector_actual"] = pose_values(matrix(z=stopped_z))
    if started:
        rig.managed.session.set_state(1, "ACTIVE")
    expected_states = ["INTERRUPTED" if started else "PENDING", "PENDING", "PENDING"]
    expected = rig.managed.session.attempts[0].plan[0].matrix.copy()
    expected[2, 3] = max(stopped_z, rig.configuration.home_matrix[2, 3])

    def check_parked_before_continue():
        assert rig.machine.state == "PAUSED"
        assert np.allclose(rig.hardware.current_pose(), expected)
        assert not any(rig.expected_outputs.values())
        assert states(rig) == expected_states
        rig.managed.continue_operation()
    rig.on_wait = check_parked_before_continue
    rig.managed.request("pause")
    with pytest.raises(ManagedInterruption):
        rig.managed.checkpoint()
    rig.managed.handle()
    assert states(rig) == expected_states
    assert rig.managed.session.parked_index == 1
    assert rig.machine.state == "PICKING"
    assert np.allclose(rig.hardware.current_pose(), expected)
    assert not any(rig.expected_outputs.values())
    expected_moves = [("pause_safety",)] if stopped_z < expected[2, 3] else []
    assert [entry[1] for entry in rig.log if entry[0] == "move"] == (
        expected_moves + [("park_transit",)])


def test_repeated_pause_of_parked_candidate_does_not_consume_it():
    rig = Rig()
    rig.managed.session.set_state(1, "ACTIVE")
    for _ in range(2):
        rig.managed.request("pause")
        rig.managed.handle()
    assert states(rig) == ["INTERRUPTED", "PENDING", "PENDING"]
    assert rig.managed.session.next_eligible == 1
    assert rig.managed.session.attempted_count == 1


def test_pause_held_preserves_outputs_and_continue_does_not_open_fingers():
    rig = Rig(held=True)
    rig.managed.request("pause")
    rig.managed.handle()
    assert rig.holding_item
    assert rig.managed.session.held_index == 1
    assert rig.hardware.current_pose()[2, 3] == pytest.approx(.8)
    assert not any(entry[0] in ("output", "pulse", "home") for entry in rig.log)
    session = rig.managed.session
    result = PickExecutor(rig.hardware, finish_home=True).run(
        [attempt.plan for attempt in session.attempts], rig.configuration.profile,
        session=session, check=lambda _i: None, return_home=rig._execute_home)
    assert result["picked"]
    assert ("output", 14, True) not in rig.log


@pytest.mark.parametrize("prepick", [50., 80.])
@pytest.mark.parametrize("dropped", [False, True])
def test_held_return_and_paused_drop_use_original_candidate_release_and_home(prepick, dropped):
    rig = Rig(held=True, prepick=prepick)
    if dropped:
        waits = [0]

        def on_wait():
            waits[0] += 1
            if waits[0] == 1:
                rig.feed["digital_input_bits"] = 0
            else:
                rig.managed.continue_operation()
        rig.on_wait = on_wait
    rig.managed.request("pause" if dropped else "return")
    if dropped:
        rig.managed.handle()
    else:
        with pytest.raises(ReturnedToHome):
            rig.managed.handle()
    assert states(rig)[0] == ("DROPPED" if dropped else "RETURNED")
    assert states(rig)[1:] == ["PENDING", "PENDING"]
    assert not rig.holding_item
    assert rig.managed.session.held_index is None
    moves = [entry for entry in rig.log if entry[0] == "move"]
    assert any("return_release" in entry[1] for entry in moves)
    assert ("pulse", 50) in rig.log
    pulse = rig.log.index(("pulse", 50))
    assert rig.log.index(("output", 2, False)) < pulse
    assert rig.log.index(("output", 13, False)) < pulse
    assert rig.log.index(("output", 14, True)) < pulse
    assert not any(rig.expected_outputs.values())
    assert np.allclose(rig.hardware.current_pose(), rig.configuration.home_matrix)
    returned = next(entry[1] for entry in rig.log if entry[0] == "home")
    assert returned["queue_through_home"]
    retreat = returned["preceding"]
    assert len(retreat) == (1 if prepick == 50. else 2)
    assert len(retreat[0].motion_io) == 4


def test_drop_during_pause_rise_stops_and_puts_back_even_if_di1_bounces_high():
    rig = Rig(held=True)
    dropped = [False]

    def on_move(_targets, kwargs):
        if kwargs.get("batch_name") == "pause_safety" and not dropped[0]:
            dropped[0] = True
            rig.feed["digital_input_bits"] = 0
            rig.managed.observe(rig.snapshot())
            rig.feed["digital_input_bits"] = 1
            rig.managed.checkpoint()
    rig.hardware.on_move = on_move
    rig.managed.request("pause")
    rig.managed.handle()
    assert states(rig)[0] == "DROPPED"
    assert ("pulse", 50) in rig.log
    assert not rig.holding_item
    assert any(entry[0] == "stop" and "Suction lost" in entry[1] for entry in rig.log)


def test_direct_stop_during_putback_prevents_release_and_home():
    rig = Rig(held=True)

    def stop_during_motion(_targets, kwargs):
        if kwargs.get("batch_name") == "return_item_to_release":
            rig.cancel_event.set()
            rig.managed.checkpoint()
    rig.hardware.on_move = stop_during_motion
    rig.managed.request("return")
    with pytest.raises(OperationCanceled):
        rig.managed.handle()
    assert not any(entry[0] in ("pulse", "home", "output") for entry in rig.log)
    assert rig.holding_item


def test_no_remaining_candidates_pause_at_safety_then_continue_returns_home():
    rig = Rig(count=1)
    rig.managed.session.set_state(1, "ACTIVE")
    rig.managed.session.set_state(1, "FAILED")
    rig.managed.request("pause")
    rig.managed.handle()
    assert rig.managed.session.parked_index is None
    outcome = PickExecutor(rig.hardware, finish_home=True).run(
        [attempt.plan for attempt in rig.managed.session.attempts], rig.configuration.profile,
        session=rig.managed.session, check=lambda _index: None, return_home=rig._execute_home)
    assert not outcome["picked"]
    assert states(rig) == ["FAILED"]
    assert any(entry[0] == "home" for entry in rig.log)
    assert not any(entry[0] == "move" and "p1_pick" in entry[1] for entry in rig.log)


def test_resume_opens_fingers_then_descends_through_prepick_to_final_pick():
    rig = Rig(count=2)
    rig.managed.session.set_state(1, "ACTIVE")
    rig.managed.request("pause")
    rig.managed.handle()
    rig.log.clear()
    session = rig.managed.session
    transitions = []
    session.changed = lambda index, attempt: transitions.append((index, attempt.state))
    PickExecutor(rig.hardware, finish_home=True).run(
        [attempt.plan for attempt in session.attempts], rig.configuration.profile,
        session=session, check=lambda _index: None, return_home=rig._execute_home)
    first_motion = next(entry for entry in rig.log if entry[0] == "move")
    assert first_motion[1] == ("p1_prepick", "p1_pick")
    assert first_motion[2]["pick_settling_sec"] == rig.configuration.profile[
        "timing"]["pick_settling"]
    assert rig.log.index(("output", 2, False)) < rig.log.index(first_motion)
    assert rig.log.index(("output", 14, True)) < rig.log.index(first_motion)
    assert states(rig) == ["FAILED", "FAILED"]
    assert transitions == [(1, "ACTIVE"), (1, "FAILED"), (2, "ACTIVE"), (2, "FAILED")]


def test_suction_acquired_during_pause_stop_is_retained_as_trusted_held_context():
    rig = Rig()
    rig.managed.session.set_state(1, "ACTIVE")
    rig.feed["digital_input_bits"] = 1
    rig.hardware.output(13, True)
    rig.hardware.acquisition_eligible = True
    rig.managed.request("pause")
    rig.managed.handle()
    assert rig.holding_item
    assert states(rig)[0] == "HELD"
    assert not any(entry[0] == "pulse" for entry in rig.log)


def test_untrusted_di1_blocks_managed_motion():
    rig = Rig()
    rig.feed["digital_input_bits"] = 1
    rig.managed.request("pause")
    with pytest.raises(HeldUnknown):
        rig.managed.handle()
    assert not any(entry[0] in ("move", "output", "pulse") for entry in rig.log)


def test_stale_paused_feedback_never_triggers_release():
    rig = Rig(held=True)

    def stale():
        def unavailable(**_kwargs):
            raise FeedbackFailure("stale")
        rig.monitor.snapshot = unavailable
    rig.on_wait = stale
    rig.managed.request("pause")
    with pytest.raises(FeedbackFailure, match="stale"):
        rig.managed.handle()
    assert not any(entry[0] == "pulse" for entry in rig.log)


def test_continue_rejected_before_parking_finishes():
    rig = Rig()
    rig.managed.request("pause")
    with pytest.raises(CommandRejected, match="completed Pause"):
        rig.managed.continue_operation()


def test_return_geometry_rejects_drop_above_prepick_and_preserves_original_pose():
    rig = Rig(prepick=40.)
    with pytest.raises(ValueError, match="at least 50 mm"):
        return_targets(rig.managed.session.attempts[0].plan)
    rig = Rig(prepick=80.)
    plan = rig.managed.session.attempts[0].plan
    release, _retreat = return_targets(plan)
    assert release.matrix[2, 3] == pytest.approx(plan[3].matrix[2, 3] + .05)
    assert np.allclose(release.matrix[:2, 3], plan[3].matrix[:2, 3])
    assert np.allclose(release.matrix[:3, :3], plan[3].matrix[:3, :3])
    above = matrix(x=.5, z=1.0)
    assert np.array_equal(safety_target(above, matrix(z=.8), profile()).matrix, above)


def test_queued_output_difference_at_pause_is_reconciled_before_neutralizing():
    rig = Rig()
    rig.expected_outputs = {1: False, 2: False, 13: True, 14: True}
    rig.feed["digital_outputs"] = 0
    rig.managed.session.set_state(1, "ACTIVE")
    rig.managed.session.set_state(1, "FAILED")
    rig.managed.session.set_state(2, "ACTIVE")
    rig.managed.request("pause")
    rig.managed.handle()
    assert states(rig) == ["FAILED", "INTERRUPTED", "PENDING"]
    assert rig.managed.session.parked_index == 2
    assert not any(rig.expected_outputs.values())
    rig.log.clear()
    session = rig.managed.session
    PickExecutor(rig.hardware, finish_home=True).run(
        [attempt.plan for attempt in session.attempts], rig.configuration.profile,
        session=session, check=lambda _index: None, return_home=rig._execute_home)
    assert next(entry[1] for entry in rig.log if entry[0] == "move") == (
        "p2_prepick", "p2_pick")
    assert states(rig) == ["FAILED", "FAILED", "FAILED"]
    assert not any(entry[0] == "move" and "p1_pick" in entry[1] for entry in rig.log)


def test_pause_during_blended_retry_transit_keeps_that_candidate_for_continue():
    rig = Rig(count=2)
    session = rig.managed.session
    plans = [attempt.plan for attempt in session.attempts]

    def pause_on_transit(targets, kwargs):
        if kwargs.get("batch_name") == "candidate_1_pick_to_retry_2_pick":
            assert targets[2].name == "p2_transit"
            for target in targets[:3]:
                session.admitted(target)
            rig.managed.request("pause")
            rig.managed.checkpoint()
    rig.hardware.on_move = pause_on_transit
    with pytest.raises(ManagedInterruption):
        PickExecutor(rig.hardware, finish_home=True).run(
            plans, rig.configuration.profile, session=session,
            check=lambda _index: None, return_home=rig._execute_home)

    assert states(rig) == ["FAILED", "ACTIVE"]
    rig.managed.handle()
    assert states(rig) == ["FAILED", "INTERRUPTED"]
    assert session.parked_index == 2
    assert np.allclose(rig.hardware.current_pose(), plans[1][0].matrix)
    rig.hardware.on_move = lambda *_args: None
    rig.log.clear()
    PickExecutor(rig.hardware, finish_home=True).run(
        plans, rig.configuration.profile, session=session,
        check=lambda _index: None, return_home=rig._execute_home)
    assert next(entry[1] for entry in rig.log if entry[0] == "move") == (
        "p2_prepick", "p2_pick")
    assert states(rig) == ["FAILED", "FAILED"]


def test_return_requested_from_confirmed_pause_has_no_continue_or_new_pick():
    rig = Rig(held=True)
    rig.on_wait = lambda: rig.managed.request("return")
    rig.managed.request("pause")
    with pytest.raises(ReturnedToHome):
        rig.managed.handle()
    assert rig.machine.state == "READY"
    assert states(rig) == ["RETURNED", "PENDING", "PENDING"]
    assert not any(entry[0] == "move" and any(name.endswith("_pick") for name in entry[1])
                   for entry in rig.log)


def test_pulse_failure_stops_before_retract_home_and_keeps_source_context():
    rig = Rig(held=True)

    def fail():
        raise FeedbackFailure("Exhaust OFF not confirmed")
    rig.hardware.exhaust_pulse = fail
    rig.managed.request("return")
    with pytest.raises(FeedbackFailure, match="Exhaust OFF"):
        rig.managed.handle()
    assert not any(entry[0] == "home" for entry in rig.log)
    assert rig.managed.session.held_index == 1


def test_unresolved_command_response_prevents_all_managed_repositioning():
    rig = Rig()

    def unresolved():
        raise CommandRejected("Still awaiting MovL response")
    rig.hardware.ensure_no_pending_response = unresolved
    rig.managed.request("pause")
    with pytest.raises(CommandRejected, match="Still awaiting"):
        rig.managed.handle()
    assert not any(entry[0] in ("move", "output", "pulse") for entry in rig.log)


def test_source_change_blocks_return_before_repositioning():
    rig = Rig(held=True)

    def changed(_root):
        raise ValueError("Configured Item Teach/model changed")
    rig.configuration.validate_sources = changed
    rig.managed.request("return")
    with pytest.raises(ValueError, match="changed"):
        rig.managed.handle()
    assert not any(entry[0] in ("move", "output", "pulse") for entry in rig.log)


def test_completed_picks_paused_drop_continues_retained_candidates_after_home():
    rig = Rig(held=True, count=2)
    rig.active_action = "pause"
    rig.machine = ControllerStateMachine(initial="PAUSING")
    rig.managed.previous = "HOLDING"
    rig.managed.kind = "pause"
    rig.pause_event.set()
    rig._candidate_progress = lambda *_a: None
    rig._end_operation = lambda: rig.operation_lock.release()
    rig._contain_queue_control_failure = lambda *_a: pytest.fail("Unexpected managed failure")
    waits = [0]

    def on_wait():
        waits[0] += 1
        if waits[0] == 1:
            rig.feed["digital_input_bits"] = 0
        else:
            assert np.allclose(rig.hardware.current_pose(), rig.configuration.home_matrix)
            rig.managed.continue_operation()
    rig.on_wait = on_wait
    rig.managed._idle_worker()
    assert states(rig) == ["DROPPED", "FAILED"]
    assert rig.machine.state == "READY"
    assert not rig.operation_lock.locked()
    pulse = next(i for i, entry in enumerate(rig.log) if entry[0] == "pulse")
    next_pick = next(i for i, entry in enumerate(rig.log)
                     if entry[0] == "move" and "p2_pick" in entry[1])
    assert pulse < next_pick


def test_idle_ready_pause_never_routes_to_old_pending_candidates():
    rig = Rig()
    rig.machine = ControllerStateMachine(initial="PAUSING")
    rig.active_action = "pause"
    rig.managed.previous = "READY"
    rig.managed.kind = "pause"
    rig.managed.handle()
    assert rig.machine.state == "READY"
    assert not any(entry[0] in ("move", "output", "pulse") for entry in rig.log)


def test_feedback_callback_latches_short_paused_drop_even_after_di1_recovers():
    rig = Rig(held=True)
    waits = [0]

    def on_wait():
        waits[0] += 1
        if waits[0] == 1:
            rig.feed["digital_input_bits"] = 0
            assert rig.managed.observe(rig.snapshot())
            rig.feed["digital_input_bits"] = 1
        else:
            rig.managed.continue_operation()
    rig.on_wait = on_wait
    rig.managed.request("pause")
    rig.managed.handle()
    assert states(rig)[0] == "DROPPED"
    assert ("pulse", 50) in rig.log


@pytest.mark.parametrize("count", [1, 2])
def test_pick_action_retries_same_candidate_after_repeated_pause(monkeypatch, count):
    import robot_controller.controller as controller_module
    from robot_controller.controller import RobotController

    rig = Rig(count=count)
    rig.machine = ControllerStateMachine(initial="READY")
    rig.candidate_total = 0
    rig.cancel_requested = rig.cancel_event.is_set
    rig.wait_for_resume = rig.managed.checkpoint
    rig._candidate_progress = lambda *_args: rig.managed.checkpoint()
    rig._attempt_changed = lambda *_args: None
    rig._end_operation = lambda: rig.operation_lock.release()
    rig._failure_outcome = RobotController._failure_outcome
    rig._action_failure = lambda *_args: pytest.fail("Unexpected Pick failure: " + str(_args[2]))
    cfg = rig.configuration
    cfg.selection = SimpleNamespace(
        station=SimpleNamespace(platform=SimpleNamespace(base_from_platform=np.eye(4))),
        robot_camera=SimpleNamespace(reference_from_camera_link=np.eye(4), sha256="x"),
        bin=SimpleNamespace(points=[]))
    attitude = SimpleNamespace(accepted=True, rotation=np.eye(3), offset_direction="cw",
                               rotation_from_home_deg=0., mirrored=False,
                               selected_camera_platform_xy=(0., 0.))
    monkeypatch.setattr(controller_module, "select_pick_attitude", lambda *_a: attitude)
    candidates = [SimpleNamespace(identifier=f"candidate{i}", position_m=(i * .1, 0., .3),
                                  quaternion=(0., 0., 0., 1.)) for i in range(count)]
    requests = []
    rig.candidates = SimpleNamespace(
        request=lambda *_a, **_k: requests.append("detect") or SimpleNamespace(
            candidates=candidates, debug_message=""))
    initial_home = rig._execute_home

    def home(**kwargs):
        if rig.managed.session is None:
            rig.feed["tool_vector_actual"] = pose_values(cfg.home_matrix)
        else:
            initial_home(**kwargs)
    rig._execute_home = home
    approaches = []

    def interrupt(targets, kwargs):
        if kwargs.get("stop_on_suction"):
            approaches.append(tuple(target.name for target in targets))
            if len(approaches) <= 2:
                rig.managed.session.admitted(targets[0])
                rig.managed.request("pause")
                rig.managed.checkpoint()
    rig.hardware.on_move = interrupt
    finished = []
    goal = SimpleNamespace(request=SimpleNamespace(save_debug_images=False),
                           succeed=lambda: finished.append("success"),
                           abort=lambda: finished.append("abort"))
    result = RobotController._execute_pick_action(rig, goal)
    assert requests == ["detect"]
    assert result.outcome == result.NO_PICK
    assert result.attempted_candidates == count
    assert finished == ["success"]
    assert states(rig) == ["FAILED"] * count
    assert approaches[:3] == [("p1_transit", "p1_prepick", "p1_pick"),
                              ("p1_prepick", "p1_pick"), ("p1_prepick", "p1_pick")]
    assert len(approaches) == count + 2
    assert not rig.operation_lock.locked()


def test_pause_after_action_result_transfers_operation_owner_until_continue():
    from robot_controller.controller import RobotController

    rig = Rig(held=True)
    rig.machine = ControllerStateMachine(initial="HOLDING")
    rig.active_goal = object()
    rig.cancel_requested = rig.cancel_event.is_set
    rig.publish_status = lambda: None
    rig._end_operation = lambda: RobotController._end_operation(rig)
    rig._contain_queue_control_failure = lambda *_args: pytest.fail(str(_args))
    rig.on_wait = lambda: None
    parked = threading.Event()
    release = threading.Event()

    def pause_wait(_seconds):
        parked.set()
        if release.wait(1):
            rig.managed.continue_operation()
    rig.wait_control = pause_wait
    rig.managed.request("pause")
    assert rig.managed.thread is None  # Still owned by the just-completed action.
    RobotController._end_operation(rig)
    try:
        assert parked.wait(1)
        assert rig.operation_lock.locked()
        assert rig.active_action == "pause"
        assert rig.active_goal is None
        assert rig.machine.state == "PAUSED"
    finally:
        release.set()
        rig.managed.thread.join(2)
    assert not rig.managed.thread.is_alive()
    assert not rig.operation_lock.locked()
    assert rig.machine.state == "HOLDING"


def test_pause_before_home_action_executor_starts_runs_no_motion_until_continue():
    from robot_controller.controller import RobotController

    rig = Rig()
    rig.machine = ControllerStateMachine(initial="READY")
    rig.active_action = "home"
    rig.wait_for_resume = rig.managed.checkpoint
    rig._end_operation = lambda: rig.operation_lock.release()
    rig._execute_cartesian_home = lambda: rig.log.append(("action_home",))
    rig._failure_outcome = RobotController._failure_outcome
    rig._action_failure = lambda *_args: pytest.fail(str(_args[2]))

    def continue_home():
        assert rig.machine.state == "PAUSED"
        assert not any(entry[0] in ("move", "action_home") for entry in rig.log)
        rig.managed.continue_operation()
    rig.on_wait = continue_home
    rig.managed.request("pause")
    goal = SimpleNamespace(succeed=lambda: rig.log.append(("success",)))
    result = RobotController._execute_home_action(rig, goal)
    assert result.outcome == result.SUCCESS
    assert ("success",) in rig.log
    assert rig.machine.state == "READY"
    assert sum(entry[0] == "action_home" for entry in rig.log) == 1


def test_new_pause_at_continue_completion_is_not_cleared_by_old_owner():
    rig = Rig()
    transition = rig._transition
    requested = []

    def pause_again(state, message):
        transition(state, message)
        if state == "PICKING" and not requested:
            requested.append(True)
            rig.managed.request("pause")
    rig._transition = pause_again
    rig.managed.request("pause")
    rig.managed.handle()
    assert rig.managed.kind == "pause"
    assert rig.machine.state == "PAUSING"
    with pytest.raises(ManagedInterruption):
        rig.managed.checkpoint()
    rig.managed.handle()
    assert rig.machine.state == "PICKING"
    assert rig.managed.kind is None


def test_return_requested_at_paused_drop_releases_once_and_finishes_ready():
    rig = Rig(held=True)

    def stop_and_drop():
        rig.feed["digital_input_bits"] = 0
        rig.managed.request("return")
    rig.on_wait = stop_and_drop
    rig.managed.request("pause")
    with pytest.raises(ReturnedToHome):
        rig.managed.handle()
    assert rig.machine.state == "READY"
    assert states(rig)[0] == "DROPPED"
    assert sum(entry[0] == "pulse" for entry in rig.log) == 1


@pytest.mark.parametrize("prepick", [50., 80.])
def test_put_back_never_sends_release_events_on_zero_distance_retreat(prepick):
    settings = profile(prepick)
    settings["motion"]["retract_height"] = 0.
    plan = pick_targets(matrix(z=.8), matrix(z=.3), settings, 1)
    if prepick == 50.:
        with pytest.raises(ValueError, match="clearance above the release"):
            return_targets(plan)
    else:
        release, retreat = return_targets(plan)
        assert len(retreat) == 1
        assert retreat[0].matrix[2, 3] > release.matrix[2, 3]
        assert {event.channel for event in retreat[0].motion_io} == {1, 2, 13, 14}
