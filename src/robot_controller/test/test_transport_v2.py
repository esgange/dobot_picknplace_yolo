from types import SimpleNamespace
import inspect
import threading

import numpy as np
import pytest

import robot_controller.hardware as hardware_module
from robot_controller.errors import (CommandRejected, CommandResponseTimeout,
                                     FeedbackFailure, HeldUnknown)
from robot_controller.hardware import DobotTransport, HOME_JOINT_TOLERANCE_RAD
from robot_controller.kinematics import pose_values
from robot_controller.motion import MotionIO, Target, cartesian_home_targets


class EventLog:
    def __init__(self):
        self.entries = []

    def record(self, *_args, **_kwargs):
        self.entries.append((_args, _kwargs))


def test_service_console_levels_use_distinct_rclpy_call_sites():
    seen = {}

    class StrictLogger:
        def log(self, level, _message):
            caller = inspect.currentframe().f_back.f_back
            location = (caller.f_code.co_filename, caller.f_lineno)
            previous = seen.setdefault(location, level)
            assert previous == level

        def info(self, message):
            self.log("INFO", message)

        def warning(self, message):
            self.log("WARNING", message)

        def error(self, message):
            self.log("ERROR", message)

    transport = object.__new__(DobotTransport)
    transport.node = SimpleNamespace(get_logger=lambda: StrictLogger())
    for level in ("INFO", "ERROR", "WARNING", "INFO", "ERROR"):
        transport._console_service_log(level, "service result")
    assert set(seen.values()) == {"INFO", "WARNING", "ERROR"}


def test_normal_service_timeout_after_two_seconds_blocks_later_dispatch(monkeypatch):
    class PendingFuture:
        def done(self):
            return False

        def add_done_callback(self, _callback):
            pass

    dispatches = []
    client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: dispatches.append(request) or PendingFuture())
    transport = object.__new__(DobotTransport)
    transport.response_lock = threading.RLock()
    transport.pending_response = None
    transport.clients = {"MovL": client}
    transport.types = {"MovL": SimpleNamespace(Request=lambda **fields: fields)}
    transport.node = SimpleNamespace(
        check_command_owner=lambda _name: None,
        cancel_requested=lambda: False,
        wait_control=lambda _seconds: None)
    transport.monitor = SimpleNamespace(snapshot=lambda **_kwargs: None)
    outcomes = []
    transport._begin_service_audit = lambda name, fields: {"name": name}
    transport._finish_service_audit = lambda _audit, outcome, **fields: outcomes.append(
        (outcome, fields))
    ticks = iter((10.0, 10.5, 11.5, 12.0))
    monkeypatch.setattr(hardware_module, "time",
                        SimpleNamespace(monotonic=lambda: next(ticks)))

    with pytest.raises(CommandResponseTimeout, match="MovL response timeout"):
        transport.call("MovL", mode=False)
    assert len(dispatches) == 1
    assert outcomes == [("timeout", {"detail": "no response within 2 seconds",
                                     "level": "ERROR"})]
    with pytest.raises(CommandResponseTimeout, match="still awaiting MovL response"):
        transport.call("MovL", mode=False)
    assert len(dispatches) == 1


def test_move_batch_dispatches_every_target_before_only_terminal_arrival_check(
        monkeypatch):
    monkeypatch.setattr(hardware_module, "STATIONARY_SEC", 0.0)
    transport = object.__new__(DobotTransport)
    order = []
    sequence = iter(range(1, 20))

    def ready_snapshot():
        return SimpleNamespace(
            sequence=next(sequence),
            feed={"tool_vector_actual": [0.0] * 6,
                  "digital_input_bits": 0, "digital_outputs": 0})

    first = np.eye(4)
    first[0, 3] = 0.1
    second = np.eye(4)
    second[0, 3] = 0.2
    targets = (Target("first", first, 100, 100),
               Target("terminal", second, 100, 100))
    events = EventLog()
    transport.node = SimpleNamespace(
        events=events, expected_outputs={}, raise_if_cancelled=lambda: None,
        cancel_requested=lambda: False,
        operation_progress=lambda *_args, **_kwargs: None)
    transport.monitor = SimpleNamespace(sequence=99)
    transport.current_pose = lambda: np.eye(4)
    transport._target_values = lambda target: list(target.matrix[:3, 3]) + [0.] * 3
    transport._ready_snapshot = ready_snapshot
    transport._wait_for_resume = lambda: 0.0
    transport._idle = lambda _snapshot: True
    transport._target_reached = lambda target, _snapshot: (
        order.append(("reached", target.name)) or True)

    def call_group(calls, **_kwargs):
        order.extend(("call", service, fields["param_value"])
                     for service, fields in calls)
        return tuple(None for _call in calls)
    transport.call_group = call_group
    transport.suction_interrupted = False
    transport.suction_stop_future = None
    transport.moving = False

    assert transport.move_batch(targets, batch_name="forward") is False
    parameters = ["user=0", "tool=0", "v=100", "a=100"]
    assert order == [("call", "MovL", parameters),
                     ("call", "MovL", parameters),
                     ("reached", "terminal")]
    names = [entry[0][1] for entry in events.entries]
    assert names == ["motion_batch_dispatch_started", "motion_queued",
                     "motion_queued", "motion_batch_queued",
                     "motion_batch_completed"]


def test_hardware_home_waypoints_dispatch_cartesian_movl_only(monkeypatch):
    monkeypatch.setattr(hardware_module, "STATIONARY_SEC", 0.0)
    transport = object.__new__(DobotTransport)
    current = np.eye(4)
    current[:3, 3] = [0.1, 0.2, 0.1]
    home = np.eye(4)
    home[:3, 3] = [0.3, -0.4, 0.5]
    targets = cartesian_home_targets(
        current, home, speed_percent=75, acceleration_percent=60)
    sequence = iter(range(1, 20))
    transport.node = SimpleNamespace(
        events=EventLog(), expected_outputs={}, raise_if_cancelled=lambda: None,
        cancel_requested=lambda: False,
        operation_progress=lambda *_args, **_kwargs: None)
    transport.monitor = SimpleNamespace(sequence=0)
    transport.current_pose = lambda: current
    transport._ready_snapshot = lambda: SimpleNamespace(
        sequence=next(sequence),
        feed={"tool_vector_actual": [0.0] * 6,
              "digital_input_bits": 0, "digital_outputs": 0})
    transport._wait_for_resume = lambda: 0.0
    transport._idle = lambda _sample: True
    transport._target_reached = lambda _target, _sample: True
    calls = []
    transport.call_group = lambda group, **_kwargs: (
        calls.extend(group) or (None,))
    transport.suction_interrupted = False
    transport.suction_stop_future = None
    transport.moving = False

    for target in targets:
        assert transport.move_batch((target,), batch_name=target.name) is False

    assert [name for name, _fields in calls] == ["MovL", "MovL"]
    assert all(fields["mode"] is False and "mdis" not in fields
               and fields["param_value"] == ["user=0", "tool=0", "v=75", "a=60"]
               for _name, fields in calls)
    assert [fields["a"] for _name, fields in calls] == pytest.approx([100., 300.])
    assert [fields["b"] for _name, fields in calls] == pytest.approx([200., -400.])
    assert [fields["c"] for _name, fields in calls] == pytest.approx([500., 500.])


def test_home_height_accepts_small_getpose_jitter_but_never_descends(monkeypatch):
    monkeypatch.setattr(hardware_module, "STATIONARY_SEC", 0.0)
    transport = object.__new__(DobotTransport)
    origin = np.eye(4)
    origin[0, 3] = 0.0005
    origin[2, 3] = 0.1
    height = np.eye(4)
    height[2, 3] = 0.2
    sequence = iter(range(1, 20))

    def sample():
        return SimpleNamespace(
            sequence=next(sequence),
            feed={"tool_vector_actual": [0.0] * 6,
                  "digital_input_bits": 0, "digital_outputs": 0})
    transport.node = SimpleNamespace(
        events=EventLog(), expected_outputs={}, raise_if_cancelled=lambda: None,
        cancel_requested=lambda: False,
        operation_progress=lambda *_args, **_kwargs: None)
    transport.monitor = SimpleNamespace(sequence=0)
    transport.current_pose = lambda: origin
    transport._target_values = lambda _target: [0.0] * 6
    transport._ready_snapshot = sample
    transport._wait_for_resume = lambda: 0.0
    transport._idle = lambda _snapshot: True
    transport._target_reached = lambda _target, _snapshot: True
    calls = []

    def call_group(requests, **_kwargs):
        calls.extend(requests)
        return (None,)

    transport.call_group = call_group
    transport.suction_interrupted = False
    transport.suction_stop_future = None
    transport.moving = False

    target = Target("home_height", height, 100, 100, relative_z=True)
    assert transport.move_batch((target,), batch_name="home_height") is False
    assert calls[0][0] == "RelMovLUser"
    assert calls[0][1]["c"] == pytest.approx(100.0)

    lower = height.copy()
    lower[2, 3] = 0.09
    with pytest.raises(CommandRejected, match="must rise"):
        transport.move_batch(
            (Target("home_height", lower, 100, 100, relative_z=True),),
            batch_name="unsafe_home_height")
    assert len(calls) == 1


def test_motion_group_preserves_cross_service_dashboard_order():
    class DeferredFuture:
        def __init__(self):
            self.completed = False
            self.callbacks = []

        def done(self):
            return self.completed

        def result(self):
            return SimpleNamespace(res=0)

        def add_done_callback(self, callback):
            self.callbacks.append(callback)

        def complete(self):
            self.completed = True
            for callback in self.callbacks:
                callback(self)

    dispatches = []
    futures = []

    def dispatch(name, request):
        future = DeferredFuture()
        dispatches.append((name, request))
        futures.append(future)
        return future

    transport = object.__new__(DobotTransport)
    transport.response_lock = threading.RLock()
    transport.pending_response = None
    transport.pending_group = None
    transport.clients = {
        name: SimpleNamespace(
            service_is_ready=lambda: True,
            call_async=lambda request, service=name: dispatch(service, request))
        for name in ("MovL", "MovLIO")
    }
    transport.types = {
        name: SimpleNamespace(Request=lambda **fields: fields)
        for name in ("MovL", "MovLIO")
    }
    completed_wait = []

    def wait_control(_seconds):
        assert len(dispatches) == len(completed_wait) + 1
        completed_wait.append(dispatches[-1][0])
        futures[-1].complete()

    transport.node = SimpleNamespace(
        check_all_command_owners=lambda names: None,
        cancel_requested=lambda: False, wait_control=wait_control)
    transport.monitor = SimpleNamespace(snapshot=lambda **_kwargs: None)
    transport.suction_interrupted = False
    transport.request_stop = lambda _reason: pytest.fail("Stop was not expected")
    audits = []

    def begin(name, fields):
        audit = {"name": name, "fields": fields,
                 "started": hardware_module.time.monotonic()}
        audits.append(audit)
        return audit

    transport._begin_service_audit = begin
    transport._finish_service_audit = lambda audit, outcome, **_fields: audit.update(
        outcome=outcome)

    results = transport.call_group((
        ("MovL", {"mode": False}),
        ("MovLIO", {"mode": False, "mdis": ["{1,0,13,1}"]}),
    ))

    assert len(results) == 2
    assert completed_wait == ["MovL", "MovLIO"]
    assert [name for name, _request in dispatches] == ["MovL", "MovLIO"]
    assert [audit["outcome"] for audit in audits] == ["accepted", "accepted"]
    assert transport.pending_group is None


def test_motion_group_timeout_blocks_later_sends_and_contains_late_reply(monkeypatch):
    class DeferredFuture:
        def __init__(self):
            self.callbacks = []

        def done(self):
            return False

        def result(self):
            return SimpleNamespace(res=0)

        def add_done_callback(self, callback):
            self.callbacks.append(callback)

        def complete(self):
            self.done = lambda: True
            for callback in self.callbacks:
                callback(self)

    dispatches = []
    futures = []

    def dispatch(request):
        future = DeferredFuture()
        dispatches.append(request)
        futures.append(future)
        return future

    transport = object.__new__(DobotTransport)
    transport.response_lock = threading.RLock()
    transport.pending_response = None
    transport.pending_group = None
    transport.clients = {
        "MovL": SimpleNamespace(service_is_ready=lambda: True, call_async=dispatch)}
    transport.types = {"MovL": SimpleNamespace(Request=lambda **fields: fields)}
    stops = []
    late_stops = []
    events = EventLog()
    transport.node = SimpleNamespace(
        check_all_command_owners=lambda _names: None,
        cancel_requested=lambda: False, wait_control=lambda _seconds: None,
        on_late_motion_ack=late_stops.append, events=events)
    transport.monitor = SimpleNamespace(snapshot=lambda **_kwargs: None)
    transport.suction_interrupted = False
    transport.request_stop = lambda reason: stops.append(reason) or object()
    audits = []

    def begin(name, fields):
        audit = {"name": name, "fields": fields, "started": 0.0}
        audits.append(audit)
        return audit

    def finish(audit, outcome, **fields):
        if not fields.get("late"):
            audit["outcome"] = outcome

    transport._begin_service_audit = begin
    transport._finish_service_audit = finish
    monkeypatch.setattr(
        hardware_module, "time", SimpleNamespace(monotonic=lambda: 3.0))

    with pytest.raises(CommandResponseTimeout, match="MovL response timeout"):
        transport.call_group((
            ("MovL", {"a": 1.0}),
            ("MovL", {"a": 2.0}),
            ("MovL", {"a": 3.0}),
        ))

    assert len(dispatches) == 1
    assert stops == ["motion-group dispatch failure"]
    assert [audit["outcome"] for audit in audits] == ["timeout"]
    futures[0].complete()
    assert stops == ["motion-group dispatch failure", "late motion acknowledgement"]
    assert len(late_stops) == 1


def test_motion_group_rejection_stops_before_later_dispatch():
    class ImmediateFuture:
        def __init__(self, result):
            self.response = SimpleNamespace(res=result)

        def done(self):
            return True

        def result(self):
            return self.response

        def add_done_callback(self, callback):
            callback(self)

    dispatches = []
    responses = iter((0, -20000))

    def dispatch(request):
        dispatches.append(request)
        return ImmediateFuture(next(responses))

    transport = object.__new__(DobotTransport)
    transport.response_lock = threading.RLock()
    transport.pending_response = None
    transport.pending_group = None
    transport.clients = {
        "MovL": SimpleNamespace(service_is_ready=lambda: True, call_async=dispatch)}
    transport.types = {"MovL": SimpleNamespace(Request=lambda **fields: fields)}
    stops = []

    def stop(reason):
        assert len(dispatches) == 2
        stops.append(reason)
        return object()

    transport.node = SimpleNamespace(
        check_all_command_owners=lambda _names: None,
        cancel_requested=lambda: False, wait_control=lambda _seconds: None)
    transport.monitor = SimpleNamespace(snapshot=lambda **_kwargs: None)
    transport.suction_interrupted = False
    transport.request_stop = stop
    audits = []

    def begin(name, fields):
        audit = {"name": name, "fields": fields,
                 "started": hardware_module.time.monotonic()}
        audits.append(audit)
        return audit

    transport._begin_service_audit = begin
    transport._finish_service_audit = lambda audit, outcome, **_fields: audit.update(
        outcome=outcome)

    with pytest.raises(CommandRejected, match="MovL failed: -20000"):
        transport.call_group((
            ("MovL", {"a": 1.0}),
            ("MovL", {"a": 2.0}),
            ("MovL", {"a": 3.0}),
        ))

    assert len(dispatches) == 2
    assert stops == ["motion-group dispatch failure"]
    assert [audit["outcome"] for audit in audits] == ["accepted", "rejected"]
    assert transport.pending_group is None


def test_motion_group_has_no_delay_after_accepted_response(monkeypatch):
    class ImmediateFuture:
        def done(self):
            return True

        def result(self):
            return SimpleNamespace(res=0)

        def add_done_callback(self, callback):
            callback(self)

    clock = [0.0]
    sent_at = []

    def wait_control(seconds):
        clock[0] += seconds

    client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda _request: sent_at.append(clock[0]) or ImmediateFuture())
    transport = object.__new__(DobotTransport)
    transport.response_lock = threading.RLock()
    transport.pending_response = None
    transport.pending_group = None
    transport.clients = {"MovL": client}
    transport.types = {"MovL": SimpleNamespace(Request=lambda **fields: fields)}
    transport.node = SimpleNamespace(
        check_all_command_owners=lambda _names: None,
        cancel_requested=lambda: False, wait_control=wait_control)
    transport.monitor = SimpleNamespace(snapshot=lambda **_kwargs: None)
    transport.suction_interrupted = False
    transport.request_stop = lambda _reason: pytest.fail("Stop was not expected")
    transport._begin_service_audit = lambda name, fields: {
        "name": name, "fields": fields, "started": clock[0]}
    transport._finish_service_audit = lambda audit, outcome, **_fields: audit.update(
        outcome=outcome)
    monkeypatch.setattr(
        hardware_module, "time", SimpleNamespace(monotonic=lambda: clock[0]))

    transport.call_group(tuple(("MovL", {"a": float(index)}) for index in range(3)))

    assert sent_at == pytest.approx([0.0, 0.0, 0.0])


def test_suction_interrupt_during_dispatch_prevents_later_motion(monkeypatch):
    class ImmediateFuture:
        def done(self):
            return True

        def result(self):
            return SimpleNamespace(res=0)

        def add_done_callback(self, callback):
            callback(self)

    clock = [0.0]
    sent = []
    transport = object.__new__(DobotTransport)
    transport.response_lock = threading.RLock()
    transport.pending_response = None
    transport.pending_group = None
    transport.suction_interrupted = False
    transport.clients = {"MovL": SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: sent.append(request) or ImmediateFuture())}
    transport.types = {"MovL": SimpleNamespace(Request=lambda **fields: fields)}
    transport.node = SimpleNamespace(
        check_all_command_owners=lambda _names: None,
        cancel_requested=lambda: False,
        wait_control=lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    transport.monitor = SimpleNamespace(snapshot=lambda **_kwargs: None)
    transport._begin_service_audit = lambda name, fields: {
        "name": name, "fields": fields, "started": clock[0]}
    transport._finish_service_audit = lambda audit, outcome, **_fields: audit.update(
        outcome=outcome)
    monkeypatch.setattr(
        hardware_module, "time", SimpleNamespace(monotonic=lambda: clock[0]))

    def observe(_snapshot):
        if sent:
            transport.suction_interrupted = True

    results = transport.call_group(tuple(
        ("MovL", {"a": float(index)}) for index in range(3)), progress=observe)

    assert len(sent) == len(results) == 1
    assert transport.pending_group is None


def test_retry_group_uses_global_cp_and_requires_observed_vacuum_reset(monkeypatch):
    monkeypatch.setattr(hardware_module, "STATIONARY_SEC", 0.0)
    transport = object.__new__(DobotTransport)
    events = EventLog()
    transport.node = SimpleNamespace(
        events=events, expected_outputs={}, raise_if_cancelled=lambda: None,
        cancel_requested=lambda: False,
        operation_progress=lambda *_args, **_kwargs: None)
    count = [0]

    def sample(outputs):
        count[0] += 1
        return SimpleNamespace(
            sequence=count[0],
            feed={"tool_vector_actual": [0.0] * 6,
                  "digital_input_bits": 0, "digital_outputs": outputs})

    vacuum_on = (1 << 12) | (1 << 13)
    vacuum_off = 1 << 13
    transport.current_pose = lambda: np.eye(4)
    transport._target_values = lambda target: list(target.matrix[:3, 3]) + [0.] * 3
    transport._ready_snapshot = lambda: sample(vacuum_on)
    transport._wait_for_resume = lambda: 0.0
    transport._idle = lambda _snapshot: True
    transport._target_reached = lambda _target, _snapshot: True
    transport.monitor = SimpleNamespace(sequence=0, wait=lambda predicate, *_a, **_k: (
        predicate(sample(vacuum_on)) or pytest.fail("Expected timed outputs")))
    settling = []
    transport.sensor = lambda active, timeout, **kwargs: (
        settling.append((active, timeout, kwargs)) or False)
    transport.moving = False
    transport.suction_interrupted = False
    transport.suction_stop_future = None
    captured = []

    def call_group(calls, *, progress, outputs_by_call):
        captured.extend(calls)
        assert outputs_by_call == [{13: False, 1: True},
                                   {}, {1: False, 14: True}, {}, {13: True}]
        transport.pending_motion_outputs = {13: False, 1: True}
        progress(sample(vacuum_off))
        transport.pending_motion_outputs = {13: True, 1: False, 14: True}
        progress(sample(vacuum_on))
        return (None,) * len(calls)

    transport.call_group = call_group
    targets = (
        Target("old_retract", np.eye(4), 100, 100, motion_io=(
            MotionIO(20, 13, False), MotionIO(20, 1, True))),
        Target("old_clearance", np.eye(4), 100, 100),
        Target("next_clearance", np.eye(4), 100, 100, motion_io=(
            MotionIO(0, 1, False), MotionIO(0, 14, True))),
        Target("next_prepick", np.eye(4), 100, 100),
        Target("next_pick", np.eye(4), 6, 100, motion_io=(
            MotionIO(0, 13, True),)),
    )

    assert not transport.move_batch(
        targets, batch_name="retry", stop_on_suction=True,
        require_suction_reset=True, settle_suction_sec=0.2)
    assert [name for name, _fields in captured] == [
        "MovLIO", "MovL", "MovLIO", "MovL", "MovLIO"]
    assert [fields["param_value"] for _name, fields in captured] == [
        ["user=0", "tool=0", "v=100", "a=100"],
        ["user=0", "tool=0", "v=100", "a=100"],
        ["user=0", "tool=0", "v=100", "a=100"],
        ["user=0", "tool=0", "v=100", "a=100"],
        ["user=0", "tool=0", "v=6", "a=100"]]
    assert all(not any(value.startswith(("cp=", "r="))
                       for value in fields["param_value"])
               for _name, fields in captured)
    assert settling == [(True, 0.2, {"settling_sec": 0})]

    def no_reset(calls, *, progress, outputs_by_call):
        progress(sample(vacuum_on))
        return (None,) * len(calls)

    transport.call_group = no_reset
    with pytest.raises(FeedbackFailure, match="DO13 OFF was not observed"):
        transport.move_batch(
            targets, batch_name="retry_without_reset", stop_on_suction=True,
            require_suction_reset=True, settle_suction_sec=0.2)
    assert settling == [(True, 0.2, {"settling_sec": 0})]


def test_motion_output_becomes_pending_only_after_its_movlio_is_dispatched():
    class ImmediateFuture:
        def done(self):
            return True

        def result(self):
            return SimpleNamespace(res=0)

        def add_done_callback(self, callback):
            callback(self)

    observed = []
    transport = object.__new__(DobotTransport)
    transport.response_lock = threading.RLock()
    transport.pending_response = None
    transport.pending_group = None
    transport.pending_motion_outputs = {}

    def dispatch(name):
        observed.append((name, dict(transport.pending_motion_outputs)))
        return ImmediateFuture()

    transport.clients = {
        "MovLIO": SimpleNamespace(
            service_is_ready=lambda: True,
            call_async=lambda _request: dispatch("MovLIO"))}
    transport.types = {
        "MovLIO": SimpleNamespace(Request=lambda **fields: fields)}
    transport.node = SimpleNamespace(
        check_all_command_owners=lambda _names: None,
        cancel_requested=lambda: False, wait_control=lambda _seconds: None)
    transport.monitor = SimpleNamespace(snapshot=lambda **_kwargs: None)
    transport.suction_interrupted = False
    transport.request_stop = lambda _reason: pytest.fail("Stop was not expected")
    transport._begin_service_audit = lambda name, fields: {
        "name": name, "fields": fields, "started": 0.0}
    transport._finish_service_audit = lambda audit, outcome, **_fields: audit.update(
        outcome=outcome)

    transport.call_group(
        (("MovLIO", {"a": 1.0, "mdis": ["{0,50,14,1}"]}),
         ("MovLIO", {"a": 2.0, "mdis": ["{1,0,13,1}"]})),
        outputs_by_call=({14: True}, {13: True}))

    assert observed == [("MovLIO", {}), ("MovLIO", {14: True})]
    assert transport.pending_motion_outputs == {13: True, 14: True}


def snapshot(*, di1=False, outputs=0, joints=None):
    return SimpleNamespace(
        robot_enabled=True,
        joints=tuple([0.0] * 6 if joints is None else joints),
        feed={
            "digital_input_bits": int(di1), "digital_outputs": outputs,
            "robot_mode": 5, "EnableStatus": 1, "isRunQueuedCmd": 0,
            "RunningStatus": 0, "ErrorStatus": 0, "CollisionStates": 0,
            "isPauseCmdFlag": 0, "userCoordinate": 0, "toolCoordinate": 0,
            "controller_timer": 1,
            "tool_vector_actual": [1.0, 2.0, 3.0, 0.0, 0.0, 0.0],
        })


def test_cold_di1_is_held_unknown_and_preserves_output_context():
    transport = object.__new__(DobotTransport)
    transport.monitor = SimpleNamespace(snapshot=lambda **_kwargs: snapshot(di1=True))
    with pytest.raises(HeldUnknown, match="without trusted"):
        transport._check_held_context(False, {})


def test_known_holding_requires_di1_and_exact_expected_outputs():
    transport = object.__new__(DobotTransport)
    transport.monitor = SimpleNamespace(
        snapshot=lambda **_kwargs: snapshot(di1=True, outputs=1 << 12))
    transport._check_held_context(True, {13: True})
    with pytest.raises(HeldUnknown, match="DO2"):
        transport._check_held_context(True, {2: True, 13: True})
    transport.monitor = SimpleNamespace(snapshot=lambda **_kwargs: snapshot(di1=False))
    with pytest.raises(HeldUnknown, match="lost DI1"):
        transport._check_held_context(True, {13: True})


def test_startup_sequence_is_explicit_and_never_calls_home():
    transport = object.__new__(DobotTransport)
    order = []
    transport.node = SimpleNamespace(
        check_all_command_owners=lambda services: order.append(("owners", tuple(services))),
        check_feedback_owners=lambda: order.append(("feedback_owners",)),
        cancel_requested=lambda: False, events=EventLog(),
        operation_progress=lambda phase, message, **fields: order.append(
            ("phase", phase, message, fields)))
    transport.clients = {"EnableRobot": object()}
    transport.queue_clients = {"Pause": object(), "Continue": object()}
    transport.monitor = SimpleNamespace(
        snapshot=lambda **_kwargs: snapshot(),
        wait_samples=lambda *_args, **_kwargs: order.append(("wait_samples",)))
    transport.wait_services = lambda **kwargs: order.append(
        ("services", tuple(kwargs.get("optional", ()))))
    transport._call_startup = lambda name, **_kwargs: order.append(("call", name))
    transport.request_stop = lambda reason: order.append(("stop", reason)) or object()
    transport.confirm_stop = lambda _future: order.append(("stop_confirmed",))
    transport._check_held_context = lambda known, expected: order.append(
        ("held_check", known, expected))
    transport._clear_errors_if_needed = lambda: order.append(("clear_if_needed",))
    transport._wait_enabled = lambda: order.append(("enabled",))
    transport._apply_settings = lambda speed: order.append(("settings", speed))
    transport._reset_outputs_if_unheld = lambda: order.append(("output_reset",))
    transport._confirm_ready = lambda: order.append(("ready",))

    DobotTransport.startup(transport)

    calls = [entry[1] for entry in order if entry[0] == "call"]
    assert calls == ["StopMoveJog", "DisableRobot", "EnableRobot"]
    assert order.index(("stop_confirmed",)) < order.index(("call", "DisableRobot"))
    assert order.index(("call", "EnableRobot")) < order.index(("settings", 100))
    assert order.count(("enabled",)) == 1
    assert order.index(("output_reset",)) < order.index(("ready",))
    assert order[-2:] == [("output_reset",), ("ready",)]
    assert all("Home" not in str(entry) and "MovL" not in str(entry) for entry in order)


def test_final_ready_accepts_idle_pause_latch():
    transport = object.__new__(DobotTransport)
    paused = snapshot()
    paused.feed["isPauseCmdFlag"] = 1

    class Monitor:
        def wait(self, predicate, *_args, **_kwargs):
            assert predicate(paused)
            return paused

        def snapshot(self, **_kwargs):
            return paused

    transport.monitor = Monitor()
    transport.node = SimpleNamespace(
        holding_item=False, expected_outputs={}, cancel_requested=lambda: False,
        operation_progress=lambda *_args, **_kwargs: None)
    assert transport._confirm_ready() is paused


class CompletedFuture:
    def __init__(self, result=0):
        self._result = SimpleNamespace(res=result)

    def done(self):
        return True

    def result(self):
        return self._result


class StopMonitor:
    def __init__(self, sample):
        self.sample = sample

    def wait(self, predicate, *_args, **_kwargs):
        assert not predicate(self.sample)
        assert predicate(self.sample)
        return self.sample


def test_stop_confirmation_detects_held_item_loss_without_changing_outputs():
    transport = object.__new__(DobotTransport)
    transport.node = SimpleNamespace(
        holding_item=True, expected_outputs={13: True}, events=EventLog(),
        wait_control=lambda _seconds: None)
    stopped = snapshot(di1=False, outputs=1 << 12)
    stopped.feed["tool_vector_actual"] = [0.0] * 6
    transport.monitor = StopMonitor(stopped)
    transport.moving = True
    with pytest.raises(Exception, match="held-item integrity"):
        transport.confirm_stop(CompletedFuture())
    assert transport.node.expected_outputs == {13: True}


def test_stop_confirmation_accepts_a_latched_pause_after_queue_is_empty():
    transport = object.__new__(DobotTransport)
    transport.node = SimpleNamespace(
        holding_item=False, expected_outputs={}, events=EventLog(),
        wait_control=lambda _seconds: None)
    stopped = snapshot()
    stopped.feed["isPauseCmdFlag"] = 1
    stopped.feed["tool_vector_actual"] = [0.0] * 6
    transport.monitor = StopMonitor(stopped)
    transport.moving = True
    transport.confirm_stop(CompletedFuture())
    assert not transport.moving


def test_stop_confirmation_accepts_commanded_finger_transition_during_return():
    transport = object.__new__(DobotTransport)
    transport.node = SimpleNamespace(
        holding_item=True,
        expected_outputs={2: False, 13: True, 14: True},
        events=EventLog(), wait_control=lambda _seconds: None)
    transport.pending_motion_outputs = {2: True, 14: False}
    stopped = snapshot(
        di1=True, outputs=(1 << (2 - 1)) | (1 << (13 - 1)))
    stopped.feed["tool_vector_actual"] = [0.0] * 6
    transport.monitor = StopMonitor(stopped)
    transport.moving = True

    transport.confirm_stop(CompletedFuture())

    assert transport.node.expected_outputs == {2: True, 13: True, 14: False}
    assert transport.pending_motion_outputs == {}
    assert not transport.moving


def test_held_motion_monitor_adopts_only_commanded_finger_transition():
    transport = object.__new__(DobotTransport)
    transport.node = SimpleNamespace(
        expected_outputs={2: False, 13: True, 14: True})
    transport.suction_interrupted = False
    transitioned = snapshot(
        di1=True, outputs=(1 << (2 - 1)) | (1 << (13 - 1)))

    transport._monitor_motion_policy(
        transitioned, require_suction=True, forbid_suction=False,
        stop_on_suction=False, before_suction=None,
        planned_outputs={2: True, 14: False})

    assert transport.node.expected_outputs == {2: True, 13: True, 14: False}

    transport.node.expected_outputs[1] = False
    changed_without_command = snapshot(
        di1=True,
        outputs=(1 << (1 - 1)) | (1 << (2 - 1)) | (1 << (13 - 1)))
    with pytest.raises(FeedbackFailure, match="Held-item output DO1 changed"):
        transport._monitor_motion_policy(
            changed_without_command, require_suction=True, forbid_suction=False,
            stop_on_suction=False, before_suction=None,
            planned_outputs={2: True, 14: False})


class SensorMonitor:
    def __init__(self, *, fresh):
        self.fresh = fresh

    def wait(self, *_args, **_kwargs):
        raise FeedbackFailure("Timed out waiting for DI1=1")

    def snapshot(self, **_kwargs):
        if not self.fresh:
            raise FeedbackFailure("stale feedback")
        return snapshot(di1=False)


def test_only_fresh_coherent_opposite_di1_becomes_a_missed_pick():
    transport = object.__new__(DobotTransport)
    transport.node = SimpleNamespace(cancel_requested=lambda: False)
    transport.monitor = SensorMonitor(fresh=True)
    assert not transport.sensor(True, 0.2, settling_sec=0)
    transport.monitor = SensorMonitor(fresh=False)
    with pytest.raises(FeedbackFailure, match="Timed out waiting"):
        transport.sensor(True, 0.2, settling_sec=0)


def test_service_audit_records_exact_send_and_terminal_response():
    transport = object.__new__(DobotTransport)
    transport.service_audit_lock = __import__("threading").Lock()
    transport.service_audit_sequence = 0
    events = EventLog()
    console = []
    logger = SimpleNamespace(
        info=console.append, warning=console.append, error=console.append)
    transport.node = SimpleNamespace(
        events=events, get_logger=lambda: logger)

    audit = transport._begin_service_audit("SpeedFactor", {"ratio": 37})
    transport._finish_service_audit(
        audit, "accepted", result=SimpleNamespace(res=0, robot_return="{}"))

    assert "SEND /dobot_bringup_ros2/srv/SpeedFactor" in console[0]
    assert 'request={"ratio":37}' in console[0]
    assert "ACCEPTED /dobot_bringup_ros2/srv/SpeedFactor res=0" in console[1]
    sent = events.entries[0][1]
    response = events.entries[1][1]
    assert sent["request_id"] == response["request_id"] == 1
    assert sent["endpoint"] == "/dobot_bringup_ros2/srv/SpeedFactor"
    assert response["response_res"] == 0
    assert response["robot_return"] == "{}"


class HomeReachedMonitor:
    def __init__(self, sample):
        self.sample = sample
        self.wait_calls = 0

    def snapshot(self, **_kwargs):
        return self.sample

    def wait(self, predicate, _timeout, **kwargs):
        self.wait_calls += 1
        assert kwargs["stable_sec"] == pytest.approx(0.3)
        assert predicate(self.sample)
        return self.sample


def home_reached_transport(sample):
    transport = object.__new__(DobotTransport)
    transport.monitor = HomeReachedMonitor(sample)
    transport.node = SimpleNamespace(
        holding_item=False, expected_outputs={}, cancel_requested=lambda: False)
    return transport


def test_existing_home_uses_same_one_degree_stable_completion_gate():
    transport = home_reached_transport(
        snapshot(joints=[HOME_JOINT_TOLERANCE_RAD] * 6))
    assert transport.home_already_reached((0.0,) * 6)
    assert transport.monitor.wait_calls == 1

    transport = home_reached_transport(
        snapshot(joints=[1.01 * HOME_JOINT_TOLERANCE_RAD] * 6))
    assert not transport.home_already_reached((0.0,) * 6)
    assert transport.monitor.wait_calls == 0


def test_existing_home_skip_requires_idle_empty_queue():
    sample = snapshot(joints=[0.0] * 6)
    sample.feed["isRunQueuedCmd"] = 1
    transport = home_reached_transport(sample)
    assert not transport.home_already_reached((0.0,) * 6)
    assert transport.monitor.wait_calls == 0


class CurrentPoseMonitor:
    def __init__(self, sample, *, fail=False, stale=False, frozen=False):
        self.sample = sample
        self.fail = fail
        self.stale = stale
        self.frozen = frozen
        self.wait_kwargs = None

    def wait(self, predicate, timeout, **kwargs):
        self.wait_kwargs = {"timeout": timeout, **kwargs}
        if self.fail or self.stale:
            raise FeedbackFailure("synthetic stationary timeout")
        assert not predicate(self.sample)
        if not self.frozen:
            self.sample.feed["controller_timer"] += 1
        if not predicate(self.sample):
            raise FeedbackFailure("synthetic stationary timeout")
        assert predicate(self.sample)  # One repeated ROS payload is not a frozen source.
        return self.sample

    def snapshot(self, **_kwargs):
        if self.stale:
            raise FeedbackFailure("Canonical robot connection/feedback is unavailable or stale")
        return self.sample


def current_pose_transport(sample, *, fail=False, stale=False, frozen=False):
    transport = object.__new__(DobotTransport)
    transport.monitor = CurrentPoseMonitor(
        sample, fail=fail, stale=stale, frozen=frozen)
    transport.node = SimpleNamespace(
        holding_item=False, expected_outputs={}, cancel_requested=lambda: False)
    calls = []
    transport.call = lambda name, **fields: calls.append((name, fields))
    return transport, calls


def test_current_pose_uses_the_stable_feed_sample_without_dashboard_service():
    transport, calls = current_pose_transport(snapshot())

    matrix = transport.current_pose()

    assert transport.monitor.wait_kwargs["timeout"] == pytest.approx(2.0)
    assert transport.monitor.wait_kwargs["stable_sec"] == pytest.approx(0.3)
    assert transport.monitor.wait_kwargs["require_enabled"] is True
    assert calls == []
    assert tuple(matrix[:3, 3]) == pytest.approx((0.001, 0.002, 0.003))


def test_current_pose_preserves_feed_position_and_attitude_in_motion_frame():
    sample = snapshot()
    sample.feed["tool_vector_actual"] = [300.0, -430.0, 350.0, -170.0, 5.0, -135.0]
    transport, calls = current_pose_transport(sample)

    assert pose_values(transport.current_pose()) == pytest.approx(
        sample.feed["tool_vector_actual"], abs=1e-6)
    assert calls == []


def test_current_pose_timeout_reports_exact_idle_blockers_and_skips_motion_origin():
    blocked = snapshot()
    blocked.robot_enabled = False
    blocked.feed.update(robot_mode=7, isRunQueuedCmd=1, RunningStatus=1)
    transport, calls = current_pose_transport(blocked, fail=True)

    with pytest.raises(FeedbackFailure) as captured:
        transport.current_pose()

    message = str(captured.value)
    assert "RobotStatus.is_enable=False" in message
    assert "robot_mode=7" in message
    assert "isRunQueuedCmd=1" in message
    assert "RunningStatus=1" in message
    assert calls == []


def test_current_pose_rejects_nonzero_user_or_tool_before_using_feed():
    for key in ("userCoordinate", "toolCoordinate"):
        blocked = snapshot()
        blocked.feed[key] = 1
        transport, calls = current_pose_transport(blocked, fail=True)

        with pytest.raises(FeedbackFailure, match=f"{key}=1"):
            transport.current_pose()
        assert calls == []


def test_current_pose_never_reuses_stale_feed_vector():
    transport, calls = current_pose_transport(snapshot(), stale=True)

    with pytest.raises(FeedbackFailure, match="unavailable or stale"):
        transport.current_pose()
    assert calls == []


def test_current_pose_rejects_republished_feed_with_frozen_controller_timer():
    transport, calls = current_pose_transport(snapshot(), frozen=True)

    with pytest.raises(FeedbackFailure, match="did not remain coherent for 300 ms"):
        transport.current_pose()
    assert calls == []


def test_current_pose_rejects_source_that_freezes_after_one_timer_tick(monkeypatch):
    ticks = iter((0.0, 0.02, 0.20))
    monkeypatch.setattr(hardware_module, "time", SimpleNamespace(
        monotonic=lambda: next(ticks)))
    sample = snapshot()
    transport, calls = current_pose_transport(sample)

    def wait(predicate, _timeout, **_kwargs):
        assert not predicate(sample)
        sample.feed["controller_timer"] += 1
        assert predicate(sample)
        assert not predicate(sample)
        raise FeedbackFailure("synthetic stationary timeout")

    transport.monitor.wait = wait
    with pytest.raises(FeedbackFailure, match="did not remain coherent for 300 ms"):
        transport.current_pose()
    assert calls == []


def test_current_pose_rejects_invalid_stationary_feed_vector():
    invalid = snapshot()
    invalid.feed["tool_vector_actual"] = [float("nan")] * 6
    transport, calls = current_pose_transport(invalid)

    with pytest.raises(FeedbackFailure, match="Invalid stationary FeedInfo"):
        transport.current_pose()
    assert calls == []


def test_current_pose_reports_unstable_idle_without_inventing_a_blocker():
    transport, calls = current_pose_transport(snapshot(), fail=True)

    with pytest.raises(FeedbackFailure, match="did not remain coherent for 300 ms"):
        transport.current_pose()
    assert calls == []
