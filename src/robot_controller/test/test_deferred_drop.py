"""Drop supervision starts at measured first-retract height, preserving raw I/O."""

from types import SimpleNamespace

import numpy as np
import pytest

from robot_controller.errors import FeedbackFailure
from robot_controller.feedback import FeedbackMonitor
from robot_controller.hardware import DobotTransport
from robot_controller.managed_control import ManagedControl
from robot_controller.motion import Target
from robot_controller.pick_session import PickSession
from test_feedback_v2 import feed, joint_message


VACUUM = 1 << 12


class RetractRig:
    def __init__(self):
        self.time, self.timer = 0., 0
        self.events, self.stops = [], []
        self.monitor = FeedbackMonitor(lambda: 1_000_000_000, monotonic=lambda: self.time)
        self.node = SimpleNamespace(
            monitor=self.monitor, holding_item=True, active_action="pick", placement=None,
            machine=SimpleNamespace(state="PICKING"), expected_outputs={13: True},
            kinematics=SimpleNamespace(forward=self.pose),
            events=SimpleNamespace(record=lambda *a, **k: self.events.append((a, k))))
        self.transport = self.node.hardware = object.__new__(DobotTransport)
        self.transport.node, self.transport.monitor = self.node, self.monitor
        self.transport.request_stop = lambda reason: self.stops.append(reason)
        self.managed = self.node.managed = ManagedControl(self.node)
        plan = tuple(Target(f"p1_{name}", self.pose((z,)), 10, 100) for name, z in (
            ("transit", .4), ("initial", .3), ("prepick", .2),
            ("pick", .1), ("retract", .2), ("final", .3)))
        self.managed.session = PickSession(["source"], [plan])
        self.managed.session.set_state(1, "ACTIVE")
        self.managed.session.set_state(1, "HELD")
        self.emit(0., .1, True)
        self.transport.defer_pickup_drop(plan[4].matrix, self.monitor.snapshot())

    @staticmethod
    def pose(joints):
        result = np.eye(4)
        result[2, 3] = joints[0]
        return result

    def emit(self, at, height, detected, *, outputs=VACUUM, fresh_joints=True,
             advance=True, observe=True):
        self.time = at
        self.timer += int(advance)
        if fresh_joints:
            joints = joint_message()
            joints.position[0] = height
            self.monitor.update_joints(joints)
        self.monitor.update_status(SimpleNamespace(is_connected=True, is_enable=True))
        self.monitor.update_feed(feed(controller_timer=self.timer,
                                      digital_outputs=outputs, digital_input_bits=int(detected)))
        sample = self.monitor.snapshot(require_enabled=True)
        if observe:
            self.managed.observe_continuous(sample)
        return sample


@pytest.mark.parametrize("height", [.2, .23])
def test_loss_during_lift_is_ignored_then_full_500ms_starts_at_retract(height):
    rig = RetractRig()
    for tick in range(1, 11):
        sample = rig.emit(tick * .1, .15, False)
        assert sample.suction_present and sample.feed["digital_input_bits"] == 0
        assert not rig.stops
        # Transport's held policy sees the same deferral as the feedback interrupt.
        rig.transport._monitor_motion_policy(
            sample, require_suction=True, forbid_suction=False, stop_on_suction=False,
            before_suction=None, planned_outputs={})
    sample = rig.emit(1.1, height, False)  # CP may pass the waypoint between samples.
    assert sample.suction_present and not rig.stops
    assert rig.emit(1.599999, height, False).suction_present
    assert not rig.emit(1.6, height, False).suction_present
    assert rig.stops == ["Confirmed held suction loss"]
    assert rig.managed.session.attempts[0].state == "DROPPED"
    armed = [k for a, k in rig.events if a[1] == "drop_detection_armed"]
    assert len(armed) == 1 and armed[0]["debounce_ms"] == 500
    assert armed[0]["actual_z_m"] == height


def test_high_after_retract_resets_the_entire_loss_interval():
    rig = RetractRig()
    rig.emit(.1, .2, False)
    rig.emit(.59, .21, True)
    rig.emit(.6, .21, False)
    assert rig.emit(1.099999, .21, False).suction_present
    assert not rig.emit(1.1, .21, False).suction_present
    assert len(rig.stops) == 1


def test_stale_or_unchanged_joint_sample_cannot_activate_monitoring():
    rig = RetractRig()
    rig.emit(.2, .2, False, fresh_joints=False)
    assert rig.monitor._pickup_retract is not None
    rig.time = 1.01
    with pytest.raises(FeedbackFailure, match="stale"):
        rig.monitor.snapshot()
    assert rig.monitor._pickup_retract is not None
    assert rig.emit(1.02, .2, False).suction_present
    assert rig.emit(1.519999, .2, False).suction_present
    assert not rig.emit(1.52, .2, False).suction_present


def test_zero_remaining_lift_arms_on_new_position_feedback():
    rig = RetractRig()
    rig.transport.defer_pickup_drop(rig.pose((.1,)), rig.monitor.snapshot())
    rig.monitor.snapshot()
    assert rig.monitor._pickup_retract is not None
    assert rig.emit(.1, .1, False).suction_present
    assert rig.monitor._pickup_retract is None
    assert not rig.emit(.6, .1, False).suction_present


def test_pause_rise_activates_at_same_height_and_stop_does_not_restart_timer():
    rig = RetractRig()
    rig.node.machine.state = "PAUSING"
    rig.managed.kind, rig.managed.parking_held = "pause", True
    assert not rig.managed.observe(rig.emit(.1, .15, False))
    assert not rig.managed.observe(rig.emit(.7, .19, False))
    rig.node.machine.state = "STOPPING"
    rig.monitor.snapshot()  # A direct Stop/read cannot enable loss early or erase the target.
    assert rig.monitor._pickup_retract is not None
    rig.node.machine.state = "PAUSING"
    assert not rig.managed.observe(rig.emit(.8, .25, False))
    assert not rig.managed.observe(rig.emit(1.299999, .25, False))
    assert rig.managed.observe(rig.emit(1.3, .25, False))
    assert rig.stops == ["Suction lost during managed Pause rise"]


def test_suction_off_clears_deferral_but_still_fails_required_output_guard():
    rig = RetractRig()
    sample = rig.emit(.1, .15, False, outputs=0, observe=False)
    assert rig.monitor._pickup_retract is None
    with pytest.raises(FeedbackFailure, match="DO13 lost"):
        rig.transport._monitor_motion_policy(
            sample, require_suction=True, forbid_suction=False, stop_on_suction=False,
            before_suction=None, planned_outputs={})


def test_next_pick_gets_its_own_retract_height_and_timer():
    rig = RetractRig()
    rig.emit(.1, .15, False, outputs=0, observe=False)
    rig.emit(.2, .15, True, observe=False)
    rig.transport.defer_pickup_drop(rig.pose((.3,)), rig.monitor.snapshot())
    rig.emit(.8, .2, False)
    assert rig.monitor._pickup_retract is not None and not rig.stops
    assert rig.emit(.9, .3, False).suction_present
    assert rig.emit(1.399999, .3, False).suction_present
    assert not rig.emit(1.4, .3, False).suction_present
