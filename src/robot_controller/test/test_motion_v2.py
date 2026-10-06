from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from item_perception_yolo.pick_planning import select_pick_attitude
from robot_controller.controller import RobotController
from robot_controller.hardware import (
    CARTESIAN_POSITION_TOLERANCE_M, COMMAND_RESPONSE_TIMEOUT_SEC,
    HOME_JOINT_TOLERANCE_RAD, MOTION_HARD_CAP_SEC, MOTION_NO_PROGRESS_SEC,
    OUTPUT_FEEDBACK_TIMEOUT_SEC, READY_STABLE_SEC, SERVICE_DISCOVERY_TIMEOUT_SEC)
from robot_controller.motion import (
    MotionIO, PickExecutor, Target, candidate_pose_in_base,
    gripper_close_events, gripper_neutral_events, gripper_open_events, home_targets,
    pick_attitude, pick_targets, pick_tray_target, pose_reached, vacuum_exhaust_events,
    vacuum_neutral_events, vacuum_suck_events)


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


def tray_target(taught=None):
    tray = SimpleNamespace(detect_matrix=item_pose(x=-.4, y=.5, z=.6, yaw_deg=30),
                           detect_joints=(.2,) * 6)
    return pick_tray_target(tray, taught or settings())


def detected_item_pose(x=0.1, y=0.2, z=0.3, long_yaw_deg=0.0):
    return item_pose(x, y, z, yaw_deg=long_yaw_deg + 90.0)


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
    assert COMMAND_RESPONSE_TIMEOUT_SEC == 5.0
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
    assert plan[1].joint_motion and not plan[0].joint_motion
    assert [target.name for target in home_targets(
        matrix(0.8), home, joints, speed_percent=100,
        acceleration_percent=100)] == ["home"]


@pytest.mark.parametrize("below_home_m, needs_rise", [
    (-0.1, False), (0.0, False), (0.000013, False), (0.004999, False),
    (0.005, False), (0.005001, True), (0.1, True),
])
def test_home_height_skip_uses_five_mm_boundary(below_home_m, needs_rise):
    home = item_pose(z=0.35, yaw_deg=30)
    current = item_pose(x=0.6, y=-0.1, z=home[2, 3] - below_home_m, yaw_deg=-45)
    joints = (0.1,) * 6
    plan = home_targets(current, home, joints, speed_percent=75,
                        acceleration_percent=60)

    assert [target.name for target in plan] == (
        ["home_height", "home"] if needs_rise else ["home"])
    assert np.array_equal(plan[-1].matrix, home)
    assert plan[-1].joints_rad == joints
    assert all((target.speed_percent, target.acceleration_percent) == (75, 60)
               for target in plan)
    if needs_rise:
        expected = current.copy()
        expected[2, 3] = home[2, 3]
        assert np.array_equal(plan[0].matrix, expected)
        assert plan[0].relative_z


@pytest.mark.parametrize("fields", [
    {"joints_rad": None}, {"joints_rad": (0.,) * 5},
    {"joints_rad": (float("nan"),) * 6}, {"relative_z": True},
    {"motion_io": (MotionIO(50, 14, True),)}, {"joint_motion": "yes"},
])
def test_joint_motion_rejects_invalid_angles_or_linear_options(fields):
    options = dict(joints_rad=(0.,) * 6, joint_motion=True)
    options.update(fields)
    with pytest.raises(ValueError):
        Target("home", matrix(), 75, 60, **options)


def test_pick_geometry_rates_and_real_timed_io():
    home = matrix(1.0)
    plan = pick_targets(home, item_pose(), settings(), 1)
    assert [target.name for target in plan] == [
        "p1_transit", "p1_initial", "p1_prepick", "p1_pick",
        "p1_retract", "p1_final"]
    assert [target.matrix[2, 3] for target in plan] == pytest.approx(
        [1.0, 0.45, 0.35, 0.3, 0.35, 0.45])
    assert [target.speed_percent for target in plan] == [100, 100, 100, 6, 6, 100]
    assert [event.vendor_value() for event in plan[0].motion_io] == [
        "{0,50,2,0}", "{0,50,14,1}"]
    assert [event.vendor_value() for event in plan[3].motion_io] == [
        "{0,20,1,0}", "{0,20,13,1}"]
    assert all(not target.motion_io for target in (plan[1], plan[2], plan[4], plan[5]))


def test_pick_green_axis_follows_item_short_axis_through_every_waypoint():
    home = matrix(1.0)
    item = detected_item_pose(long_yaw_deg=30.0)
    rotation, delta_deg, direction = pick_attitude(home, item)
    plan = pick_targets(home, item, settings(), 1)

    assert delta_deg == pytest.approx(30.0)
    assert direction == "none"
    assert np.allclose(rotation[:, 1], item[:3, 0])
    assert np.allclose(rotation[:, 2], home[:3, 2])
    assert all(np.allclose(target.matrix[:3, :3], rotation) for target in plan)


def test_camera_safe_mirror_is_used_for_every_motion_waypoint():
    home = matrix(1.0)
    candidate = detected_item_pose(x=0.18, y=0.0)
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
    item = detected_item_pose(long_yaw_deg=170.0)
    rotation, delta_deg, _direction = pick_attitude(home, item)

    assert delta_deg == pytest.approx(-10.0)
    assert abs(float(np.dot(rotation[:, 1], item[:3, 0]))) == pytest.approx(1.0)
    assert np.allclose(rotation[:, 2], home[:3, 2])


def test_controller_waypoints_use_selected_camera_safe_mirror_without_moving_pick_point():
    home = matrix(1.0)
    item = detected_item_pose(x=0.18, y=0.0)
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
    yaw = np.deg2rad(115.0)
    item = candidate_pose_in_base(
        platform, (0.1, 0.2, 0.3),
        (0., 0., np.sin(yaw / 2), np.cos(yaw / 2)))
    home = matrix(1.0)
    rotation, _delta_deg, _direction = pick_attitude(home, item)
    projected = item[:3, 0].copy()
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
    item = detected_item_pose(long_yaw_deg=42.0)
    rotation, delta_deg, _direction = pick_attitude(home, item)
    projected = item[:3, 0] - home[:3, 2] * float(
        np.dot(item[:3, 0], home[:3, 2]))
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
    item[:3, :3] = [[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]]
    with pytest.raises(ValueError, match="cannot be projected"):
        pick_attitude(matrix(1.0), item)


def test_cartesian_tolerance_is_strict():
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

    def sensor(self, active, timeout):
        self.log.append(("sensor", active, timeout))
        if not active:
            return True
        return False

    def output(self, channel, active, **_kwargs):
        self.log.append(("output", channel, active))

    def move_batch(self, targets, **kwargs):
        self.targets.append(tuple(targets))
        names = tuple(target.name for target in targets)
        self.log.append(("move", names, kwargs))
        acquired = next(self.acquisitions) if kwargs.get("stop_on_suction") else False
        if kwargs.get("return_terminal_pose"):
            return acquired, self.pose.copy()
        return acquired

    def current_pose(self):
        pytest.fail("Pick must reuse the final-pick terminal feedback pose")


def test_success_closes_only_after_suction_and_finishes_at_tray_holding():
    hardware = FakeHardware([True])
    plan = pick_targets(matrix(1.0), item_pose(), settings(), 1)
    returned = []
    outcome = PickExecutor(hardware, finish_home=True).run(
        [plan], settings(), tray_target=tray_target(),
        check=lambda _index: None,
        return_home=lambda **kwargs: returned.append(kwargs))
    assert outcome == {"picked": True, "candidate": 1, "holding_item": True}
    forward = next(entry for entry in hardware.log if entry[0] == "move")
    assert forward[2]["batch_name"] == "candidate_1_home_to_pick"
    assert forward[2]["pick_settling_sec"] == pytest.approx(0.2)
    assert forward[2]["return_terminal_pose"] is True
    assert np.array_equal(forward[2]["pickup_retract_pose"], plan[4].matrix)
    assert forward[1] == ("p1_transit", "p1_prepick", "p1_pick")
    assert ("output", 14, False) in hardware.log
    assert ("output", 2, True) in hardware.log
    assert not returned
    finish = [entry for entry in hardware.log if entry[0] == "move"][-1]
    assert finish[2]["batch_name"] == "candidate_1_pick_to_tray"
    assert finish[2]["require_suction"] is True
    assert finish[2]["forbid_suction"] is False
    assert np.allclose(finish[2]["confirmed_start_pose"], hardware.pose)
    assert finish[1] == ("p1_retract", "p1_final", "p1_transit_exit", "tray_detect_position")
    assert hardware.targets[-1][-1].joints_rad == tray_target().joints_rad


def test_success_with_deferred_grip_closes_halfway_through_prepick_lift():
    taught = settings(close_on_pick=False)
    hardware = FakeHardware([True])
    plan = pick_targets(matrix(1.0), item_pose(), taught, 1)
    returned = []

    outcome = PickExecutor(hardware, finish_home=True).run(
        [plan], taught, tray_target=tray_target(),
        check=lambda _index: None,
        return_home=lambda **kwargs: returned.append(kwargs))

    assert outcome["picked"]
    assert not any(entry[0] == "output" and entry[1] in (2, 14)
                   for entry in hardware.log)
    assert not returned
    retract, clearance, exit_transit, destination = hardware.targets[-1]
    assert [event.vendor_value() for event in retract.motion_io] == [
        "{0,50,14,0}", "{0,50,2,1}"]
    assert not clearance.motion_io
    assert not exit_transit.motion_io
    assert not destination.motion_io


@pytest.mark.parametrize("use_grip", [False, True])
@pytest.mark.parametrize("close_on_pick", [False, True])
def test_pick_return_queues_finger_policy_and_safety_exit_before_tray_arrival(
        use_grip, close_on_pick):
    from test_placement_queue import QueueRig, HELD

    taught = settings(use_grip=use_grip, close_on_pick=close_on_pick)
    taught["speed"].update(travel_percent=80, retract_percent=6)
    taught["acceleration"].update(travel_percent=70, retract_percent=40)
    rig = QueueRig()
    rig.node.placement = None
    rig.node.expected_outputs = {1: False, 2: close_on_pick, 13: True, 14: not close_on_pick}
    rig.emit(outputs=HELD if close_on_pick else (1 << 12) | (1 << 13), inputs=1, running=0)
    hardware = FakeHardware([True])
    plan = pick_targets(matrix(1.), item_pose(), taught, 1)
    PickExecutor(hardware, finish_home=True).run(
        [plan], taught, tray_target=pick_tray_target(rig.node.configuration.tray, taught),
        check=lambda _index: None, return_home=lambda **_kwargs: pytest.fail("No Home"))
    rig.order.clear()
    rig.steps = iter([dict(outputs=HELD if use_grip else 1 << 12,
                           inputs=1, running=0, currentCommandId=4)])

    rig.transport.move_batch(
        hardware.targets[-1], require_suction=True, confirmed_start_pose=hardware.pose)

    lift_service = "MovL" if use_grip and close_on_pick else "MovLIO"
    assert rig.order == [lift_service, "MovL", "MovL", "MovL", "feedback"]
    lift, clearance, exit_transit, tray = [request for _service, request in rig.requests]
    assert list(getattr(lift, "mdis", ())) == (
        ["{0,50,2,0}", "{0,50,14,0}"] if not use_grip else
        ["{0,50,14,0}", "{0,50,2,1}"] if not close_on_pick else [])
    assert not lift.mode and not clearance.mode and not exit_transit.mode and tray.mode
    assert [list(request.param_value) for request in (lift, clearance, exit_transit, tray)] == [
        ["user=0", "tool=0", "v=6", "a=40"],
        ["user=0", "tool=0", "v=100", "a=70"],
        ["user=0", "tool=0", "v=80", "a=70"],
        ["user=0", "tool=0", "v=80", "a=70"]]
    assert (lift.c, clearance.c, exit_transit.c) == pytest.approx((350., 450., 1000.))
    assert (exit_transit.a, exit_transit.b, exit_transit.d, exit_transit.e, exit_transit.f) == (
        clearance.a, clearance.b, clearance.d, clearance.e, clearance.f)
    assert rig.node.expected_outputs == {1: False, 2: use_grip, 13: True, 14: False}


@pytest.mark.parametrize("use_grip, close_on_pick", [
    (False, False), (False, True), (True, True), (True, False)])
@pytest.mark.parametrize("home_z, stopped_z", [
    (1.0, 0.3), (0.454, 0.3), (0.45, 0.3), (1.0, 0.5), (1.0, 1.1)])
def test_success_and_exhaustion_keep_vertical_safety_exit_before_final_travel(
        use_grip, close_on_pick, home_z, stopped_z):
    taught = settings(use_grip=use_grip, close_on_pick=close_on_pick)
    taught["speed"].update(travel_percent=80, retract_percent=6)
    taught["acceleration"].update(travel_percent=70, retract_percent=40)
    home = matrix(home_z)
    plans = [pick_targets(home, item_pose(), taught, 1)]
    for acquired in (False, True):
        hardware = FakeHardware([acquired])
        hardware.pose = item_pose(x=0.102, y=0.198, z=stopped_z, yaw_deg=1)
        checks = []
        node = SimpleNamespace(
            hardware=hardware, holding_item=acquired, root=None,
            configuration=SimpleNamespace(
                profile=taught, home_matrix=home, home_joints=(0.1,) * 6,
                validate_sources=lambda _root: checks.append("sources")),
            raise_if_cancelled=lambda: None, wait_for_resume=lambda: None,
            _preflight_item_state=lambda holding: checks.append(("holding", holding)),
            operation_progress=lambda *_args, **_kwargs: None)
        node._home_plan = lambda origin: RobotController._home_plan(node, origin)
        outcome = PickExecutor(hardware, finish_home=True).run(
            plans, taught, tray_target=tray_target(taught),
            check=lambda _index: None,
            return_home=lambda **kwargs: RobotController._execute_home(node, **kwargs))

        assert outcome["picked"] is acquired
        assert outcome["holding_item"] is acquired
        outputs = [entry for entry in hardware.log if entry[0] == "output"]
        assert outputs == ([("output", 14, False), ("output", 2, True)]
                           if acquired and close_on_pick else [])
        if outputs:
            # Pickup closes after acquisition, before the lift queue is sent.
            assert hardware.log.index(outputs[0]) > next(
                i for i, entry in enumerate(hardware.log) if entry[0] == "move")
            assert hardware.log.index(outputs[-1]) < max(
                i for i, entry in enumerate(hardware.log) if entry[0] == "move")
        assert checks == ([] if acquired else ["sources"])
        assert len(hardware.targets) == 2  # Approach, then one complete finish group.
        group = hardware.targets[-1]
        assert [target.name for target in group] == ([
            "p1_retract", "p1_final", "p1_transit_exit", "tray_detect_position"] if acquired else [
            "p1_retract", "p1_final", "p1_transit_exit", "home"])
        assert group[-2].matrix[2, 3] == pytest.approx(max(home_z, stopped_z))
        assert not group[-2].motion_io
        assert not group[-2].relative_z
        assert group[-2].joints_rad is None
        first_rates = (6, 40) if acquired else (100, 70)
        assert [(target.speed_percent, target.acceleration_percent)
                for target in group] == [first_rates, (100, 70)] + [(80, 70)] * (
                    len(group) - 2)
        for target in group[:-1]:
            assert np.array_equal(target.matrix[:2, 3], hardware.pose[:2, 3])
            assert np.array_equal(target.matrix[:3, :3], hardware.pose[:3, :3])
            assert target.matrix[2, 3] >= stopped_z
        assert group[-1].joints_rad == ((0.2,) * 6 if acquired else (0.1,) * 6)
        if acquired:
            assert np.array_equal(group[-1].matrix, tray_target(taught).matrix)
        returned = [entry for entry in hardware.log if entry[0] == "move"][-1]
        assert returned[2]["require_suction"] is acquired
        assert returned[2]["forbid_suction"] is False
        assert np.array_equal(returned[2]["confirmed_start_pose"], hardware.pose)
        if acquired:
            assert all(event.channel in (2, 14) for target in group
                       for event in target.motion_io)
            assert group[0].motion_io == (
                gripper_neutral_events(50) if not use_grip else
                gripper_close_events(50) if not close_on_pick else ())
            assert not group[1].motion_io
        else:
            assert group[0].motion_io == vacuum_exhaust_events(80)
            assert group[1].motion_io == (
                gripper_neutral_events(0) + vacuum_neutral_events(0))


@pytest.mark.parametrize("stopped_z", [0.3, 0.997, 1.1])
def test_missed_suction_blends_both_safety_transits_before_next_descent(stopped_z):
    taught = settings()
    taught["acceleration"]["travel_percent"] = 80
    taught["acceleration"]["retract_percent"] = 40
    hardware = FakeHardware([False, False])
    hardware.pose = item_pose(x=.102, y=.198, z=stopped_z, yaw_deg=1.)
    plans = [pick_targets(matrix(1.0), item_pose(x=0.1 * index, yaw_deg=20. * index),
                          taught, index)
             for index in (1, 2)]
    order = []
    returned = []

    def check(index):
        order.append(("candidate", index))

    def return_home(**kwargs):
        returned.append(kwargs)
        order.append(("home", kwargs["forbid_suction"], kwargs["ignore_suction"],
                      kwargs["batch_name"]))

    outcome = PickExecutor(hardware, finish_home=True).run(
        plans, taught, tray_target=tray_target(),
        check=check, return_home=return_home)
    assert not outcome["picked"]
    assert order == [
        ("candidate", 1), ("candidate", 2),
        ("home", False, True, "candidate_2_pick_to_home")]
    forward_batches = [entry[2]["batch_name"] for entry in hardware.log
                       if entry[0] == "move"]
    assert forward_batches == ["candidate_1_home_to_pick",
                               "candidate_1_pick_to_retry_2_pick"]
    final_retract, final_clearance, final_exit = returned[0]["preceding"]
    assert [final_retract.name, final_clearance.name, final_exit.name] == [
        "p2_retract", "p2_final", "p2_transit_exit"]
    assert final_retract.motion_io == vacuum_exhaust_events(80)
    assert final_clearance.motion_io == (
        gripper_neutral_events(0) + vacuum_neutral_events(0))
    assert returned[0]["queue_through_home"] is True
    assert np.allclose(returned[0]["confirmed_start_pose"], hardware.pose)
    retry = next(entry for entry in hardware.log if entry[0] == "move"
                 and entry[2]["batch_name"] == "candidate_1_pick_to_retry_2_pick")
    assert retry[1] == ("p1_retract", "p1_final", "p1_transit_exit", "p2_transit",
                        "p2_initial", "p2_prepick", "p2_pick")
    assert retry[2]["stop_on_suction"] is True
    assert np.array_equal(retry[2]["pickup_retract_pose"], plans[1][4].matrix)
    assert retry[2]["require_suction_reset"] is True
    assert retry[2]["pick_settling_sec"] == pytest.approx(0.2)
    assert retry[2]["return_terminal_pose"] is True
    assert np.allclose(retry[2]["confirmed_start_pose"], hardware.pose)
    (retract, old_clearance, old_exit, transit, next_approach,
     next_prepick, next_pick) = hardware.targets[1]
    assert retract.speed_percent == 100
    assert retract.acceleration_percent == 80
    assert retract.matrix[2, 3] == pytest.approx(max(stopped_z, plans[0][4].matrix[2, 3]))
    assert [(event.percent, event.channel, event.active)
            for event in retract.motion_io] == [
                (80, 13, False), (80, 1, True)]
    assert old_clearance.speed_percent == 100
    assert old_clearance.acceleration_percent == 80
    assert old_clearance.matrix[2, 3] == pytest.approx(
        max(stopped_z, plans[0][5].matrix[2, 3]))
    assert old_clearance.motion_io == (
        gripper_neutral_events(0) + vacuum_neutral_events(0))
    assert np.allclose(retract.matrix[:2, 3], old_clearance.matrix[:2, 3])
    assert np.allclose(retract.matrix[:3, :3], old_clearance.matrix[:3, :3])
    assert np.array_equal(old_exit.matrix[:2, 3], hardware.pose[:2, 3])
    assert np.array_equal(old_exit.matrix[:3, :3], hardware.pose[:3, :3])
    assert old_exit.matrix[2, 3] == transit.matrix[2, 3] == max(stopped_z, 1.0)
    assert not old_exit.motion_io
    assert old_exit.speed_percent == transit.speed_percent
    assert old_exit.acceleration_percent == transit.acceleration_percent
    assert np.array_equal(transit.matrix[:2, 3], plans[1][0].matrix[:2, 3])
    assert np.array_equal(transit.matrix[:3, :3], plans[1][0].matrix[:3, :3])
    assert transit.speed_percent == taught["speed"]["travel_percent"]
    assert transit.acceleration_percent == taught["acceleration"]["travel_percent"]
    assert np.array_equal(next_approach.matrix, plans[1][1].matrix)
    assert not next_approach.motion_io
    assert [(event.percent, event.channel, event.active)
            for event in transit.motion_io] == [
                (50, 2, False), (50, 14, True)]
    assert not next_prepick.motion_io
    assert next_pick.motion_io == vacuum_suck_events(20)
    assert not any(entry[0] == "output" and entry[1] in (1, 2, 13, 14)
                   for entry in hardware.log)
    assert not any(entry[0] == "sensor" and entry[1] is True
                   for entry in hardware.log)


def test_second_candidate_success_finishes_at_tray_without_home():
    hardware = FakeHardware([False, True])
    plans = [pick_targets(matrix(1.0), item_pose(x=0.1 * index), settings(), index)
             for index in (1, 2)]
    returned = []
    outcome = PickExecutor(hardware, finish_home=True).run(
        plans, settings(), tray_target=tray_target(),
        check=lambda _index: None,
        return_home=lambda **kwargs: returned.append(kwargs))

    assert outcome == {"picked": True, "candidate": 2, "holding_item": True}
    batches = [entry[2]["batch_name"] for entry in hardware.log if entry[0] == "move"]
    assert batches == ["candidate_1_home_to_pick",
                       "candidate_1_pick_to_retry_2_pick", "candidate_2_pick_to_tray"]
    assert not returned
    assert hardware.targets[-1][-1].name == "tray_detect_position"


def test_missed_pick_opens_and_neutralizes_without_pickup_close():
    taught = settings(use_grip=False)
    hardware = FakeHardware([False, False])
    plans = [pick_targets(matrix(1.0), item_pose(x=0.1 * index), taught, index)
             for index in (1, 2)]

    PickExecutor(hardware, finish_home=False).run(
        plans, taught, tray_target=tray_target(),
        check=lambda _index: None,
        return_home=lambda **_kwargs: pytest.fail("Home was not requested"))

    (retract, old_clearance, old_exit, transit, next_approach,
     next_prepick, _pick) = hardware.targets[1]
    assert retract.motion_io == vacuum_exhaust_events(80)
    assert old_clearance.motion_io == (
        gripper_neutral_events(0) + vacuum_neutral_events(0))
    assert transit.motion_io == gripper_open_events(50)
    assert not old_exit.motion_io
    assert not next_approach.motion_io
    assert not next_prepick.motion_io
    assert not any(event.active and event.channel == 2
                   for batch in hardware.targets for target in batch
                   for event in target.motion_io)


def test_every_pick_rotation_selects_nearest_offset_from_home_independently():
    home = matrix(1.0)
    first = detected_item_pose(long_yaw_deg=80.0)
    rotation, travel, direction = pick_attitude(home, first, 20.0)
    assert direction == "cw"
    assert travel == pytest.approx(60.0)
    assert np.allclose(rotation[:, 1], item_pose(yaw_deg=60.0)[:3, 1])

    second = detected_item_pose(long_yaw_deg=100.0)
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
    with pytest.raises(ValueError, match="Opposing actuator"):
        replace(target, motion_io=(MotionIO(20, 1, True), MotionIO(20, 13, True)))
    with pytest.raises(ValueError, match="Opposing actuator"):
        replace(target, motion_io=(MotionIO(20, 2, True), MotionIO(20, 14, True)))
    with pytest.raises(ValueError, match="two states"):
        replace(target, motion_io=(MotionIO(20, 13, False), MotionIO(20, 13, True)))
