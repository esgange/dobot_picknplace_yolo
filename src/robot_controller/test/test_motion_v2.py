from dataclasses import replace

import numpy as np
import pytest

from item_perception_yolo.pick_planning import select_pick_attitude
from robot_controller.hardware import (
    CARTESIAN_POSITION_TOLERANCE_M, COMMAND_RESPONSE_TIMEOUT_SEC,
    HOME_JOINT_TOLERANCE_RAD, MIN_MOTION_DISPATCH_INTERVAL_SEC,
    MOTION_HARD_CAP_SEC, MOTION_NO_PROGRESS_SEC,
    OUTPUT_FEEDBACK_TIMEOUT_SEC, READY_STABLE_SEC, SERVICE_DISCOVERY_TIMEOUT_SEC,
    robot_values)
from robot_controller.motion import (
    MotionIO, PickExecutor, Target, candidate_pose_in_base, home_targets,
    pick_attitude, pick_targets, pose_reached)


def matrix(z=0.5):
    value = np.eye(4)
    value[2, 3] = z
    return value


def item_pose(x=0.1, y=0.2, z=0.3, yaw_deg=0.0):
    value = matrix(z)
    yaw = np.deg2rad(yaw_deg)
    value[:3, :3] = [[np.cos(yaw), -np.sin(yaw), 0.],
                     [np.sin(yaw), np.cos(yaw), 0.], [0., 0., 1.]]
    value[:2, 3] = [x, y]
    return value


def settings(*, use_grip=True, close_on_pick=True):
    return {
        "pick_rotation": 0.0,
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
    assert SERVICE_DISCOVERY_TIMEOUT_SEC == OUTPUT_FEEDBACK_TIMEOUT_SEC == 5.0
    assert COMMAND_RESPONSE_TIMEOUT_SEC == 2.0
    assert MIN_MOTION_DISPATCH_INTERVAL_SEC == pytest.approx(0.05)
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
    plan = pick_targets(home, item_pose(), settings(), 1)
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


def test_pick_green_axis_follows_item_short_axis_through_every_waypoint():
    home = matrix(1.0)
    item = item_pose(yaw_deg=30.0)
    rotation, delta_deg, direction = pick_attitude(home, item)
    plan = pick_targets(home, item, settings(), 1)

    assert delta_deg == pytest.approx(30.0)
    assert direction == "none"
    assert np.allclose(rotation[:, 1], item[:3, 1])
    assert np.allclose(rotation[:, 2], home[:3, 2])
    assert all(np.allclose(target.matrix[:3, :3], rotation) for target in plan)


def test_camera_safe_mirror_is_used_for_every_motion_waypoint():
    home = matrix(1.0)
    candidate = item_pose(x=0.18, y=0.0)
    robot_camera = np.eye(4)
    robot_camera[0, 3] = 0.05
    selected = select_pick_attitude(
        home, candidate, 0.0, 0.0, np.eye(4), robot_camera,
        [[-0.2, -0.15], [-0.2, 0.15], [0.2, 0.15], [0.2, -0.15]])
    assert selected.accepted and selected.mirrored
    plan = pick_targets(home, candidate, settings(), 1, rotation=selected.rotation)
    assert all(np.allclose(target.matrix[:3, :3], selected.rotation)
               for target in plan)
    assert all(np.allclose(target.matrix[:2, 3], candidate[:2, 3])
               for target in plan)
    assert plan[3].matrix[2, 3] == selected.planned_link6[2, 3]


def test_rectangular_axis_uses_nearest_equivalent_tool_rotation():
    home = matrix(1.0)
    item = item_pose(yaw_deg=170.0)
    rotation, delta_deg, _direction = pick_attitude(home, item)

    assert delta_deg == pytest.approx(-10.0)
    assert abs(float(np.dot(rotation[:, 1], item[:3, 1]))) == pytest.approx(1.0)
    assert np.allclose(rotation[:, 2], home[:3, 2])


def test_controller_waypoints_use_selected_camera_safe_mirror_without_moving_pick_point():
    home = matrix(1.0)
    item = item_pose(x=0.18, y=0.0)
    camera = np.eye(4)
    camera[0, 3] = 0.05
    roi = [[-0.2, -0.15], [-0.2, 0.15], [0.2, 0.15], [0.2, -0.15]]
    attitude = select_pick_attitude(
        home, item, 0.0, 0.0, np.eye(4), camera, roi)
    assert attitude.accepted and attitude.mirrored
    plan = pick_targets(home, item, settings(), 1, rotation=attitude.rotation)
    assert all(np.allclose(target.matrix[:3, :3], attitude.rotation) for target in plan)
    assert all(target.matrix[0, 3] == pytest.approx(item[0, 3]) for target in plan)
    assert all(target.matrix[1, 3] == pytest.approx(item[1, 3]) for target in plan)
    assert plan[3].matrix[2, 3] == pytest.approx(item[2, 3])
    with pytest.raises(ValueError, match="3x3 rotation"):
        pick_targets(home, item, settings(), 1, rotation=np.eye(4))


def test_tilted_platform_short_axis_is_projected_while_tool_z_stays_fixed():
    tilt = np.deg2rad(8.0)
    platform = np.eye(4)
    platform[:3, :3] = [[1., 0., 0.],
                        [0., np.cos(tilt), -np.sin(tilt)],
                        [0., np.sin(tilt), np.cos(tilt)]]
    yaw = np.deg2rad(25.0)
    item = candidate_pose_in_base(
        platform, (0.1, 0.2, 0.3),
        (0., 0., np.sin(yaw / 2), np.cos(yaw / 2)))
    home = matrix(1.0)
    rotation, _delta_deg, _direction = pick_attitude(home, item)
    projected = item[:3, 1].copy()
    projected[2] = 0.0
    projected /= np.linalg.norm(projected)

    assert np.allclose(rotation[:, 1], projected)
    assert np.allclose(rotation[:, 2], home[:3, 2])
    assert np.allclose(item[:3, 3], [0.1, 0.2*np.cos(tilt)-0.3*np.sin(tilt),
                                     0.2*np.sin(tilt)+0.3*np.cos(tilt)])


def test_alignment_preserves_a_nonvertical_taught_tool_axis():
    pitch = np.deg2rad(7.0)
    home = matrix(1.0)
    home[:3, :3] = [[np.cos(pitch), 0., np.sin(pitch)],
                    [0., 1., 0.],
                    [-np.sin(pitch), 0., np.cos(pitch)]]
    item = item_pose(yaw_deg=42.0)
    rotation, delta_deg, _direction = pick_attitude(home, item)
    projected = item[:3, 1] - home[:3, 2] * float(
        np.dot(item[:3, 1], home[:3, 2]))
    projected /= np.linalg.norm(projected)

    assert abs(delta_deg) <= 90.0
    assert abs(float(np.dot(rotation[:, 1], projected))) == pytest.approx(1.0)
    assert np.allclose(rotation[:, 2], home[:3, 2])
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1.0)


def test_candidate_pose_rejects_nonplanar_heading_and_degenerate_projection():
    with pytest.raises(ValueError, match="yaw only"):
        candidate_pose_in_base(np.eye(4), (0.1, 0.2, 0.3),
                               (np.sin(.1), 0., 0., np.cos(.1)))
    item = item_pose()
    item[:3, :3] = [[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]]
    with pytest.raises(ValueError, match="cannot be projected"):
        pick_attitude(matrix(1.0), item)


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
        self.targets = []
        self.pose = matrix(0.3)

    def sensor(self, active, timeout, *, settling_sec):
        self.log.append(("sensor", active, timeout, settling_sec))
        if not active:
            return True
        return False

    def output(self, channel, active, **_kwargs):
        self.log.append(("output", channel, active))

    def move_batch(self, targets, **kwargs):
        self.targets.append(tuple(targets))
        names = tuple(target.name for target in targets)
        self.log.append(("move", names, kwargs))
        return next(self.acquisitions) if kwargs.get("stop_on_suction") else False

    def current_pose(self):
        return self.pose.copy()


def test_success_closes_only_after_suction_and_returns_home_holding():
    hardware = FakeHardware([True])
    plan = pick_targets(matrix(1.0), item_pose(), settings(), 1)
    returned = []
    outcome = PickExecutor(hardware, finish_home=True).run(
        [plan], settings(), check=lambda _index: None,
        return_home=lambda **kwargs: returned.append(kwargs))
    assert outcome == {"picked": True, "candidate": 1, "holding_item": True}
    forward = next(entry for entry in hardware.log if entry[0] == "move")
    assert forward[2]["batch_name"] == "candidate_1_home_to_pick"
    assert ("output", 14, False) in hardware.log
    assert ("output", 2, True) in hardware.log
    assert returned[0]["batch_name"] == "candidate_1_pick_to_home"
    assert returned[0]["require_suction"] is True
    assert returned[0]["forbid_suction"] is False


def test_missed_suction_queues_direct_retract_next_prepick_and_pick():
    hardware = FakeHardware([False, False])
    plans = [pick_targets(matrix(1.0), item_pose(x=0.1 * index), settings(), index)
             for index in (1, 2)]
    order = []

    def check(index):
        order.append(("candidate", index))

    def return_home(**kwargs):
        order.append(("home", kwargs["forbid_suction"], kwargs["batch_name"]))

    outcome = PickExecutor(hardware, finish_home=True).run(
        plans, settings(), check=check, return_home=return_home)
    assert not outcome["picked"]
    assert order == [
        ("candidate", 1), ("candidate", 2),
        ("home", True, "candidate_2_pick_to_home")]
    forward_batches = [entry[2]["batch_name"] for entry in hardware.log
                       if entry[0] == "move"]
    assert forward_batches == ["candidate_1_home_to_pick",
                               "candidate_1_pick_to_retry_2_pick",
                               "candidate_2_miss_retract"]
    retry = next(entry for entry in hardware.log if entry[0] == "move"
                 and entry[2]["batch_name"] == "candidate_1_pick_to_retry_2_pick")
    assert retry[1] == ("p1_retract", "p2_prepick", "p2_pick")
    assert retry[2]["stop_on_suction"] is True
    assert retry[2]["require_suction_reset"] is True
    assert retry[2]["settle_suction_sec"] == pytest.approx(0.2)
    retract, next_prepick, next_pick = [
        target for target in hardware.targets[1]]
    assert retract.speed_percent == 100
    assert [(event.percent, event.channel, event.active)
            for event in retract.motion_io] == [
                (20, 13, False), (20, 1, True), (20, 2, False), (20, 14, True)]
    assert [(event.percent, event.channel, event.active)
            for event in next_prepick.motion_io] == [
                (0, 1, False), (0, 2, False), (0, 14, True)]
    assert next_pick.motion_io == (MotionIO(0, 13, True),)
    assert not any(name in retry[1] for name in ("p2_transit", "p2_initial"))
    assert sum(entry[:2] == ("output", 13) and entry[2] is False
               for entry in hardware.log) == 1
    assert not any(entry[0] == "sensor" and entry[1] is True
                   for entry in hardware.log)


def test_second_candidate_success_returns_home_only_after_acquisition():
    hardware = FakeHardware([False, True])
    plans = [pick_targets(matrix(1.0), item_pose(x=0.1 * index), settings(), index)
             for index in (1, 2)]
    returned = []
    outcome = PickExecutor(hardware, finish_home=True).run(
        plans, settings(), check=lambda _index: None,
        return_home=lambda **kwargs: returned.append(kwargs))

    assert outcome == {"picked": True, "candidate": 2, "holding_item": True}
    batches = [entry[2]["batch_name"] for entry in hardware.log if entry[0] == "move"]
    assert batches == ["candidate_1_home_to_pick",
                       "candidate_1_pick_to_retry_2_pick"]
    assert [entry["batch_name"] for entry in returned] == ["candidate_2_pick_to_home"]
    assert returned[0]["require_suction"] is True


def test_missed_repick_without_finger_control_changes_only_vacuum_and_exhaust():
    taught = settings(use_grip=False)
    hardware = FakeHardware([False, False])
    plans = [pick_targets(matrix(1.0), item_pose(x=0.1 * index), taught, index)
             for index in (1, 2)]

    PickExecutor(hardware, finish_home=False).run(
        plans, taught, check=lambda _index: None,
        return_home=lambda **_kwargs: pytest.fail("Home was not requested"))

    retract, next_prepick, _pick = hardware.targets[1]
    assert [event.channel for event in retract.motion_io] == [13, 1]
    assert [event.channel for event in next_prepick.motion_io] == [1]
    assert all(entry[1] not in (2, 14) for entry in hardware.log
               if entry[0] == "output")


def test_every_pick_rotation_selects_nearest_offset_from_home_independently():
    home = matrix(1.0)
    first = item_pose(yaw_deg=80.0)
    rotation, travel, direction = pick_attitude(home, first, 20.0)
    assert direction == "cw"
    assert travel == pytest.approx(60.0)
    assert np.allclose(rotation[:, 1], item_pose(yaw_deg=60.0)[:3, 1])

    second = item_pose(yaw_deg=100.0)
    next_rotation, next_travel, next_direction = pick_attitude(home, second, 20.0)
    assert next_direction == "ccw"
    assert next_travel == pytest.approx(-60.0)
    assert np.allclose(next_rotation[:, 1], item_pose(yaw_deg=-60.0)[:3, 1])


@pytest.mark.parametrize("value", [-0.1, 90.1, float("nan"), "10"])
def test_pick_rotation_rejects_out_of_range_or_non_numeric_values(value):
    with pytest.raises(ValueError, match="pick_rotation"):
        pick_attitude(matrix(1.0), item_pose(), value)


def test_motion_io_rejects_noncanonical_or_empty_semantics():
    with pytest.raises(ValueError):
        MotionIO(20, 12, True)
    target = Target("plain", matrix(), 100, 100)
    assert target.motion_io == ()
    with pytest.raises(ValueError):
        replace(target, relative_z=True, motion_io=(MotionIO(50, 13, True),))
