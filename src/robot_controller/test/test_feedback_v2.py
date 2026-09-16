from types import SimpleNamespace
import threading
import time

import pytest

from robot_controller.errors import FeedbackFailure, OperationCanceled
from robot_controller.feedback import FeedbackMonitor, enabled_blockers


def feed(**changes):
    value = {
        "robot_mode": 5, "digital_input_bits": 0, "digital_outputs": 0,
        "controller_timer": 1, "isRunQueuedCmd": 0, "RunningStatus": 0,
        "ErrorStatus": 0, "CollisionStates": 0, "isPauseCmdFlag": 0,
        "userCoordinate": 0, "toolCoordinate": 0, "EnableStatus": 1,
        "tool_vector_actual": [0.0] * 6, "q_actual": [0.0] * 6,
        "qd_actual": [0.0] * 6, "TCP_speed_actual": [0.0] * 6,
    }
    value.update(changes)
    return value


def joint_message(stamp_ns=1_000_000_000):
    return SimpleNamespace(
        name=[f"joint{i}" for i in range(1, 7)], position=[0.0] * 6,
        header=SimpleNamespace(stamp=SimpleNamespace(
            sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)))


def primed_monitor():
    monitor = FeedbackMonitor(lambda: 1_000_000_000)
    monitor.update_joints(joint_message())
    monitor.update_status(SimpleNamespace(is_connected=True, is_enable=True))
    monitor.update_feed(feed())
    return monitor


def test_valid_feedback_snapshot_and_enabled_blockers():
    monitor = primed_monitor()
    snapshot = monitor.snapshot(require_enabled=True)
    assert snapshot.sequence == 1
    assert snapshot.joints == (0.0,) * 6
    assert enabled_blockers(snapshot.feed, True) == []
    assert "EnableStatus=0" in enabled_blockers(feed(EnableStatus=0), False)[1]


def test_pause_flag_is_not_a_general_readiness_blocker():
    idle_latch = feed(isPauseCmdFlag=1)
    assert enabled_blockers(idle_latch, True) == []
    monitor = primed_monitor()
    monitor.update_feed(idle_latch)
    assert monitor.snapshot(require_enabled=True).feed["isPauseCmdFlag"] == 1


def test_mode_10_is_accepted_only_for_explicit_pause_monitoring():
    paused = feed(robot_mode=10, isPauseCmdFlag=1)
    assert "robot_mode=10" in enabled_blockers(paused, True)[0]
    assert enabled_blockers(paused, True, allow_paused=True) == []
    monitor = primed_monitor()
    monitor.update_feed(paused)
    with pytest.raises(FeedbackFailure, match="robot_mode=10"):
        monitor.snapshot(require_enabled=True)
    assert monitor.snapshot(require_enabled=True, allow_paused=True).feed[
        "isPauseCmdFlag"] == 1


@pytest.mark.parametrize("key", [
    "tool_vector_actual", "q_actual", "qd_actual", "TCP_speed_actual"])
def test_all_six_axis_feed_arrays_are_required(key):
    monitor = primed_monitor()
    bad = feed()
    bad[key] = [0.0] * 5
    with pytest.raises(FeedbackFailure, match=key):
        monitor.update_feed(bad)


def test_three_consistent_pause_error_samples():
    monitor = primed_monitor()
    for timer in (2, 3, 4):
        monitor.update_feed(feed(controller_timer=timer, isPauseCmdFlag=1))
    assert monitor.consistent_flags(3) == (1, 0, 0)
    monitor.update_feed(feed(controller_timer=5))
    assert monitor.consistent_flags(3) is None


def test_condition_wait_uses_new_feedback_instead_of_fixed_sleep():
    monitor = primed_monitor()

    def update():
        time.sleep(0.03)
        monitor.update_feed(feed(controller_timer=2, robot_mode=4, EnableStatus=0))

    thread = threading.Thread(target=update)
    thread.start()
    started = time.monotonic()
    result = monitor.wait(
        lambda sample: sample.feed["robot_mode"] == 4, 0.5,
        description="disabled mode")
    thread.join()
    assert result.feed["robot_mode"] == 4
    assert time.monotonic() - started < 0.3


def test_stable_window_requires_advancing_feed_samples():
    monitor = primed_monitor()
    with pytest.raises(FeedbackFailure, match="stable feedback"):
        monitor.wait(
            lambda _sample: True, 0.06, stable_sec=0.02,
            description="stable feedback")

    def update():
        for timer in range(2, 8):
            time.sleep(0.01)
            monitor.update_feed(feed(controller_timer=timer))

    thread = threading.Thread(target=update)
    thread.start()
    result = monitor.wait(
        lambda _sample: True, 0.3, stable_sec=0.02,
        description="stable advancing feedback")
    thread.join()
    assert result.sequence >= 3


def test_wait_is_native_cancel_aware():
    monitor = primed_monitor()
    with pytest.raises(OperationCanceled):
        monitor.wait(lambda _sample: False, 1.0, cancel=lambda: True)


def test_explicit_pause_suspends_a_feedback_wait_deadline():
    monitor = primed_monitor()
    paused = threading.Event()
    paused.set()

    def update():
        time.sleep(0.03)
        monitor.update_feed(feed(
            controller_timer=2, robot_mode=10, isPauseCmdFlag=1))
        time.sleep(0.08)
        paused.clear()
        monitor.update_feed(feed(controller_timer=3, robot_mode=4, EnableStatus=0))

    thread = threading.Thread(target=update)
    thread.start()
    result = monitor.wait(
        lambda sample: sample.feed["robot_mode"] == 4, 0.05,
        pause=paused.is_set, description="post-pause disabled mode")
    thread.join()
    assert result.feed["robot_mode"] == 4


def test_invalid_joint_source_stamp_is_rejected():
    monitor = FeedbackMonitor(lambda: 1_000_000_000)
    with pytest.raises(FeedbackFailure, match="nonzero source timestamp"):
        monitor.update_joints(joint_message(0))
