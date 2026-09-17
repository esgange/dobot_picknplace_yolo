"""Shared metric target construction; no ROS/hardware imports or commands."""

from dataclasses import dataclass, replace
import math

import numpy as np

from camera_calibration_gui.calibration_core import rotation_angle_deg
from item_perception_yolo.item_teach_core import validate_speed, validate_acceleration
from item_perception_yolo.pick_planning import (  # noqa: F401
    candidate_pose_in_base as candidate_pose_in_base, pick_attitude, rigid_matrix)


@dataclass(frozen=True)
class MotionIO:
    percent: int
    channel: int
    active: bool

    def __post_init__(self):
        if type(self.percent) is not int or not 0 <= self.percent <= 100:
            raise ValueError("Motion I/O percentage must be an integer from 0 to 100")
        if type(self.channel) is not int or self.channel not in (1, 2, 13, 14):
            raise ValueError("Motion I/O must use the canonical gripper outputs")
        if type(self.active) is not bool:
            raise ValueError("Motion I/O state must be an explicit boolean")

    def vendor_value(self):
        # The manual explicitly defines distance-mode zero as the start trigger;
        # percentage-mode's listed range otherwise excludes zero.
        mode = 1 if self.percent == 0 else 0
        return f"{{{mode},{self.percent},{self.channel},{int(self.active)}}}"


@dataclass(frozen=True)
class Target:
    name: str
    matrix: np.ndarray
    speed_percent: int
    acceleration_percent: int
    joints_rad: tuple | None = None
    relative_z: bool = False
    motion_io: tuple = ()

    def __post_init__(self):
        if type(self.speed_percent) is not int or not 1 <= self.speed_percent <= 100:
            raise ValueError("Target speed must be an explicit integer from 1 to 100 percent")
        if (type(self.acceleration_percent) is not int
                or not 1 <= self.acceleration_percent <= 100):
            raise ValueError("Target acceleration must be an explicit integer from 1 to 100 "
                             "percent")
        if type(self.motion_io) is not tuple or any(
                not isinstance(event, MotionIO) for event in self.motion_io):
            raise ValueError("Target motion I/O must be a tuple of validated events")
        if self.relative_z and self.motion_io:
            raise ValueError("Relative Home-height moves cannot carry MovLIO events")


def home_targets(current, home, joints, *, speed_percent, acceleration_percent):
    final = Target("home", home.copy(), speed_percent, acceleration_percent, tuple(joints))
    if current[2, 3] >= home[2, 3]:
        return (final,)
    height = current.copy()
    height[2, 3] = home[2, 3]
    return (Target("home_height", height, speed_percent, acceleration_percent, relative_z=True),
            final)


def cartesian_home_targets(current, home, *, speed_percent, acceleration_percent):
    """Standalone Hardware Home: current XY with Home Z/attitude, then Home XYZ/attitude."""
    alignment = home.copy()
    alignment[:2, 3] = current[:2, 3]
    return (Target("home_align", alignment, speed_percent, acceleration_percent),
            Target("home", home.copy(), speed_percent, acceleration_percent))


def pose_reached(actual, goal, *, translation_m=0.001, rotation_deg=0.5):
    return (np.linalg.norm(actual[:3, 3] - goal[:3, 3]) <= translation_m
            and rotation_angle_deg(actual[:3, :3].T @ goal[:3, :3]) <= rotation_deg)


def pick_targets(home, item_in_base, settings, candidate_index, *, rotation=None):
    validate_speed(settings["speed"])
    validate_acceleration(settings["acceleration"])
    item_in_base = rigid_matrix(item_in_base, "Base-relative candidate pose")
    if rotation is None:
        rotation, _travel_deg, _direction = pick_attitude(
            home, item_in_base, settings["pick_rotation"])
    else:
        rotation = np.asarray(rotation, dtype=float)
        if rotation.shape != (3, 3):
            raise ValueError("Selected pick attitude must be a 3x3 rotation")
        frame = np.eye(4)
        frame[:3, :3] = rotation
        rotation = rigid_matrix(frame, "Selected pick attitude")[:3, :3]
    position = item_in_base[:3, 3]
    motion = settings["motion"]
    pick_z = float(position[2]) + motion["standoff_height"] / 1000
    prepick_z = pick_z + motion["prepick_height"] / 1000
    clearance_z = prepick_z + motion["retract_height"] / 1000
    heights = (home[2, 3], clearance_z, prepick_z, pick_z, prepick_z, clearance_z)
    if not all(math.isfinite(v) for v in (*position, *heights)):
        raise ValueError("Pick geometry contains nonfinite values")
    if home[2, 3] < max(heights[1:]):
        raise ValueError("Home Z must be at or above every pick clearance/retract height")
    names = ("transit", "initial", "prepick", "pick", "retract", "final")
    targets = []
    speeds = settings["speed"]
    percentages = (speeds["travel_percent"],) * 3 + (
        speeds["approach_percent"], speeds["retract_percent"], speeds["travel_percent"])
    accelerations = settings["acceleration"]
    acceleration_percentages = (accelerations["travel_percent"],) * 3 + (
        accelerations["approach_percent"], accelerations["retract_percent"],
        accelerations["travel_percent"])
    for name, z, percentage, acceleration in zip(names, heights, percentages,
                                                 acceleration_percentages):
        matrix = home.copy()
        matrix[:3, :3] = rotation
        matrix[:3, 3] = [position[0], position[1], z]
        events = ()
        if name == "initial" and settings["gripper"]["use_grip"]:
            events = (MotionIO(50, 2, False), MotionIO(50, 14, True))
        elif name == "pick":
            events = (MotionIO(0, 13, True),)
        targets.append(Target(f"p{candidate_index}_{name}", matrix, percentage, acceleration,
                              motion_io=events))
    return tuple(targets)


class PickExecutor:
    """Explicit sequence behind fake or real transport; no automatic fault retry."""

    def __init__(self, hardware, *, finish_home):
        if type(finish_home) is not bool:
            raise ValueError("Pick finish policy must be explicitly confirmed")
        self.hardware = hardware
        self.finish_home = finish_home

    def run(self, plans, settings, *, check, return_home, remember_prepick=None,
            progress=None, holding_changed=None):
        grip = settings["gripper"]["use_grip"]
        close = grip and settings["gripper"]["grip_onpick"]
        if not plans:
            return {"picked": False, "candidate": None, "holding_item": False}
        settling = settings["timing"]["pick_settling"]
        if progress is not None:
            progress("CANDIDATE", "Attempting candidate 1", 1)
        check(1)
        # Never turn vacuum off first and then discover a possibly held item.
        if not self.hardware.sensor(False, 0, settling_sec=0):
            raise ValueError("DI1 failed to clear before pickup")
        if remember_prepick is not None:
            remember_prepick(plans[0][2], settings["gripper"])
        self.hardware.output(1, False)
        self.hardware.output(13, False, require_clear=True)
        acquired = self.hardware.move_batch(
            plans[0][:4], batch_name="candidate_1_home_to_pick",
            stop_on_suction=True, settle_suction_sec=settling)
        for index, plan in enumerate(plans, 1):
            if acquired and holding_changed is not None:
                # Establish trusted in-memory holding context before any gripper
                # output or return motion can fail.
                holding_changed(True)
            if acquired and close:
                if progress is not None:
                    progress("GRIP", "Suction confirmed; closing fingers", index)
                self.hardware.output(14, False)
                self.hardware.output(2, True)
            stopped_pose = self.hardware.current_pose()
            stopped_z = stopped_pose[2, 3]
            upward = []
            # A retry passes the old pre-pick before its clearance; the final
            # miss keeps the existing direct clearance rise before Home.
            retrying = not acquired and index < len(plans)
            for target in (plan[4:] if acquired or retrying else plan[5:6]):
                # Vertical recovery preserves actual stopped XY/attitude.
                matrix = stopped_pose.copy()
                matrix[2, 3] = max(stopped_z, target.matrix[2, 3])
                if acquired:
                    events = ((MotionIO(100, 14, False), MotionIO(100, 2, True))
                              if grip and not close and not upward else ())
                    upward.append(replace(target, matrix=matrix, motion_io=events))
                else:
                    # The miss is decided only after the taught final-pose settle.
                    # Release and exhaust on the first upward segment only.
                    events = []
                    if not upward:
                        events = [MotionIO(20, 13, False), MotionIO(20, 1, True)]
                        if grip:
                            events.extend((MotionIO(20, 2, False),
                                           MotionIO(20, 14, True)))
                    upward.append(replace(
                        target, matrix=matrix, speed_percent=100,
                        acceleration_percent=settings["acceleration"]["travel_percent"],
                        motion_io=tuple(events)))
                stopped_z = matrix[2, 3]
            if (acquired and remember_prepick is not None
                    and stopped_pose[2, 3] > plan[2].matrix[2, 3]):
                remember_prepick(replace(plan[2], matrix=upward[0].matrix.copy()),
                                 settings["gripper"])
            if acquired:
                # The shared Home planner appends its conditional rise and exact
                # joint Home after confirmed acquisition.
                return_home(preceding=tuple(upward), require_suction=acquired,
                            forbid_suction=False,
                            batch_name=f"candidate_{index}_pick_to_home")
                return {"picked": True, "candidate": index, "holding_item": True}
            if index == len(plans):
                # There is no next pick to blend into. Complete the miss release,
                # clear exhaust, then return Home with DI1 required clear.
                self.hardware.move_batch(
                    upward, batch_name=f"candidate_{index}_miss_retract",
                    forbid_suction=True)
                self.hardware.output(1, False)
                if self.finish_home:
                    return_home(forbid_suction=True,
                                batch_name=f"candidate_{index}_pick_to_home")
                break
            next_index = index + 1
            if progress is not None:
                progress("CANDIDATE", f"Attempting candidate {next_index}", next_index)
            check(next_index)
            if not self.hardware.sensor(False, 0, settling_sec=0):
                raise ValueError("DI1 activated after missed pickup")
            next_plan = plans[index]
            if remember_prepick is not None:
                remember_prepick(next_plan[2], settings["gripper"])
            # Rise via the old pre-pick to its clearance before lateral travel.
            # Cross to the next clearance, then descend via its pre-pick.
            # Timed output events travel with their owning motion.
            transfer_events = [MotionIO(0, 1, False)]
            if grip:
                transfer_events.extend((MotionIO(0, 2, False), MotionIO(0, 14, True)))
            next_approach = replace(next_plan[1], motion_io=tuple(transfer_events))
            acquired = self.hardware.move_batch(
                (*upward, next_approach, next_plan[2], next_plan[3]),
                batch_name=f"candidate_{index}_pick_to_retry_{next_index}_pick",
                stop_on_suction=True, require_suction_reset=True,
                settle_suction_sec=settling)
        return {"picked": False, "candidate": None, "holding_item": False}
