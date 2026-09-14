"""Shared metric target construction; no ROS/hardware imports or commands."""

from dataclasses import dataclass
import math

import numpy as np

from camera_calibration_gui.calibration_core import rotation_angle_deg


@dataclass(frozen=True)
class Target:
    name: str
    matrix: np.ndarray
    joints_rad: tuple | None = None
    relative_z: bool = False


def home_targets(current, home, joints):
    height = current.copy()
    height[2, 3] = home[2, 3]
    return (Target("home_height", height, relative_z=True),
            Target("home", home.copy(), tuple(joints)))


def pose_reached(actual, goal, *, translation_m=0.001, rotation_deg=0.5):
    return (np.linalg.norm(actual[:3, 3] - goal[:3, 3]) <= translation_m
            and rotation_angle_deg(actual[:3, :3].T @ goal[:3, :3]) <= rotation_deg)


def pick_targets(home, item_in_base, settings, candidate_index):
    motion = settings["motion"]
    if motion["zheight_offset"] < max(motion["prepick_height"], motion["retract_height"]):
        raise ValueError("zheight_offset must be >= prepick_height and retract_height")
    pick_z = float(item_in_base[2]) + motion["standoff_height"] / 1000
    heights = (home[2, 3], pick_z + motion["zheight_offset"] / 1000,
               pick_z + motion["prepick_height"] / 1000, pick_z,
               pick_z + motion["retract_height"] / 1000,
               pick_z + motion["zheight_offset"] / 1000)
    if not all(math.isfinite(v) for v in (*item_in_base, *heights)):
        raise ValueError("Pick geometry contains nonfinite values")
    if home[2, 3] < max(heights[1:]):
        raise ValueError("Home Z must be at or above every pick clearance/retract height")
    names = ("transit", "initial", "prepick", "pick", "retract", "final")
    targets = []
    for name, z in zip(names, heights):
        matrix = home.copy()
        matrix[:3, 3] = [item_in_base[0], item_in_base[1], z]
        targets.append(Target(f"p{candidate_index}_{name}", matrix))
    return tuple(targets)


class PickExecutor:
    """Explicit sequence behind fake or real transport; no automatic fault retry."""

    def __init__(self, hardware, *, finish_home):
        if type(finish_home) is not bool:
            raise ValueError("Pick finish policy must be explicitly confirmed")
        self.hardware = hardware
        self.finish_home = finish_home

    def run(self, plans, settings, *, check, return_home):
        grip = settings["gripper"]["use_grip"]
        close = grip and settings["gripper"]["grip_onpick"]
        for index, plan in enumerate(plans, 1):
            check(index)
            self.hardware.output(1, False)  # Exhaust stays off; no imported release/purge pattern.
            self.hardware.output(13, False)
            if grip:
                self.hardware.output(2, False)
                self.hardware.output(14, True)
            # Stuck-high DI is a sensor/held-item error, not a successful fresh acquisition.
            if not self.hardware.sensor(False, 0, settling_sec=0):
                raise ValueError("DI1 failed to clear before pickup")
            for target in plan[:3]:
                self.hardware.move(target)
            check(index)  # Do not begin final approach on a target aged during transit.
            self.hardware.output(13, True)
            acquired = self.hardware.move(plan[3], stop_on_suction=True)
            if not acquired:
                acquired = self.hardware.sensor(True, settings["timing"]["pick_settling"],
                                                settling_sec=0)
            if acquired and close:
                self.hardware.output(14, False)
                self.hardware.output(2, True)
            stopped_z = self.hardware.current_pose()[2, 3]
            for target in plan[4:]:
                matrix = target.matrix.copy()
                matrix[2, 3] = max(stopped_z, matrix[2, 3])
                self.hardware.move(Target(target.name, matrix), require_suction=acquired)
                stopped_z = matrix[2, 3]
            if acquired:
                if self.finish_home:
                    return_home(require_suction=True)
                return {"picked": True, "candidate": index, "holding_item": True}
            # Final retract has really completed before vacuum-off and candidate advance.
            self.hardware.output(13, False)
            if index < len(plans):
                return_home(require_suction=False)
        return {"picked": False, "candidate": None, "holding_item": False}
