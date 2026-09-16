from dataclasses import replace

import numpy as np
import pytest

from robot_controller.hardware import (
    CARTESIAN_POSITION_TOLERANCE_M, COMMAND_RESPONSE_TIMEOUT_SEC,
    HOME_JOINT_TOLERANCE_RAD, MOTION_HARD_CAP_SEC, MOTION_NO_PROGRESS_SEC,
    READY_STABLE_SEC, SERVICE_DISCOVERY_TIMEOUT_SEC, robot_values)
from robot_controller.motion import (MotionIO, PickExecutor, Target, home_targets,
                                     pick_targets, pose_reached)


def matrix(z=0.5):
    value = np.eye(4)
    value[2, 3] = z
    return value


def settings(*, use_grip=True, close_on_pick=True):
    return {
        "motion": {"standoff_height": 0.0, "prepick_height": 50.0,
                   "retract_height": 100.0},
        "speed": {"travel_percent": 100, "approach_percent": 6,
                  "retract_percent": 6},
        "acceleration": {"travel_percent": 100, "approach_percent": 100,
                         "retract_percent": 100},
        "gripper": {"use_grip": use_grip, "grip_onpick": close_on_pick},
        "timing": {"pick_settling": 0.2},
    }


def test_timing_and_arrival_policy_constants():
    assert SERVICE_DISCOVERY_TIMEOUT_SEC == COMMAND_RESPONSE_TIMEOUT_SEC == 5.0
    assert READY_STABLE_SEC == 0.2
    assert MOTION_NO_PROGRESS_SEC == 3.0
    assert MOTION_HARD_CAP_SEC == 300.0
    assert CARTESIAN_POSITION_TOLERANCE_M == 0.005
    assert HOME_JOINT_TOLERANCE_RAD == pytest.approx(np.deg2rad(1.0))


def test_home_rises_only_when_below_and_finishes_at_exact_joint_target():
    home = matrix(0.7)
    joints = (0.1,) * 6
    below = matrix(0.5)
    plan = home_targets(below, home, joints, speed_percent=100,
                        acceleration_percent=100)
    assert [target.name for target in plan] == ["home_height", "home"]
    assert plan[0].relative_z
    assert plan[0].matrix[2, 3] == 0.7
    assert plan[1].joints_rad == joints
    assert [target.name for target in home_targets(
        matrix(0.8), home, joints, speed_percent=100,
        acceleration_percent=100)] == ["home"]


def test_pick_geometry_rates_and_real_timed_io():
    home = matrix(1.0)
    plan = pick_targets(home, (0.1, 0.2, 0.3), settings(), 1)
    assert [target.name for target in plan] == [
        "p1_transit", "p1_initial", "p1_prepick", "p1_pick",
        "p1_retract", "p1_final"]
    assert [target.matrix[2, 3] for target in plan] == pytest.approx(
        [1.0, 0.45, 0.35, 0.3, 0.35, 0.45])
    assert [target.speed_percent for target in plan] == [100, 100, 100, 6, 6, 100]
    assert [event.vendor_value() for event in plan[1].motion_io] == [
        "{0,50,2,0}", "{0,50,14,1}"]
    assert [event.vendor_value() for event in plan[3].motion_io] == ["{1,0,13,1}"]
    assert all(not target.motion_io for target in (plan[0], plan[2], plan[4], plan[5]))


def test_robot_reply_and_cartesian_tolerance_are_strict():
    assert robot_values("{1,2,3,4,5,6}") == [1, 2, 3, 4, 5, 6]
    with pytest.raises(Exception, match="Malformed canonical"):
        robot_values("0,{1,2,3,4,5,6},GetPose();")
    goal = matrix()
    actual = goal.copy()
    actual[0, 3] += 0.005
    assert pose_reached(actual, goal, translation_m=0.005, rotation_deg=1.0)
    actual[0, 3] += 0.00001
    assert not pose_reached(actual, goal, translation_m=0.005, rotation_deg=1.0)


class FakeHardware:
    def __init__(self, acquisitions):
        self.acquisitions = iter(acquisitions)
        self.log = []
        self.pose = matrix(0.3)

    def sensor(self, active, timeout, *, settling_sec):
        self.log.append(("sensor", active, timeout, settling_sec))
        if not active:
            return True
        return False

    def output(self, channel, active, **_kwargs):
        self.log.append(("output", channel, active))

    def move_batch(self, targets, **kwargs):
        names = tuple(target.name for target in targets)
        self.log.append(("move", names, kwargs))
        return next(self.acquisitions)

    def current_pose(self):
        return self.pose.copy()


def test_success_closes_only_after_suction_and_returns_home_holding():
    hardware = FakeHardware([True])
    plan = pick_targets(matrix(1.0), (0.1, 0.2, 0.3), settings(), 1)
    returned = []
    outcome = PickExecutor(hardware, finish_home=True).run(
        [plan], settings(), check=lambda _index: None,
        return_home=lambda **kwargs: returned.append(kwargs))
    assert outcome == {"picked": True, "candidate": 1, "holding_item": True}
    assert ("output", 14, False) in hardware.log
    assert ("output", 2, True) in hardware.log
    assert returned[0]["require_suction"] is True
    assert returned[0]["forbid_suction"] is False


def test_missed_suction_returns_home_before_advancing_candidate():
    hardware = FakeHardware([False, False])
    plans = [pick_targets(matrix(1.0), (0.1 * index, 0.2, 0.3), settings(), index)
             for index in (1, 2)]
    order = []

    def check(index):
        order.append(("candidate", index))

    def return_home(**kwargs):
        order.append(("home", kwargs["forbid_suction"]))

    outcome = PickExecutor(hardware, finish_home=True).run(
        plans, settings(), check=check, return_home=return_home)
    assert not outcome["picked"]
    assert order == [("candidate", 1), ("home", True),
                     ("candidate", 2), ("home", True)]
    assert sum(entry[:2] == ("output", 13) and entry[2] is False
               for entry in hardware.log) == 4


def test_motion_io_rejects_noncanonical_or_empty_semantics():
    with pytest.raises(ValueError):
        MotionIO(20, 12, True)
    target = Target("plain", matrix(), 100, 100)
    assert target.motion_io == ()
    with pytest.raises(ValueError):
        replace(target, relative_z=True, motion_io=(MotionIO(50, 13, True),))
