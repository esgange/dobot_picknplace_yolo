"""Read-only CR10 forward kinematics, independent of RViz/robot-state publisher."""

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from camera_calibration_gui.calibration_core import rotation_matrix_to_rpy_deg
from item_perception_yolo.item_teach_core import JOINT_NAMES, file_sha256


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def pose_matrix(values):
    if len(values) != 6 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError("Pose requires six finite mm/degree values")
    result = np.eye(4)
    result[:3, 3] = np.asarray(values[:3], dtype=float) / 1000
    result[:3, :3] = rpy_matrix(*np.deg2rad(values[3:]))
    return result


def pose_values(matrix):
    return [*map(float, matrix[:3, 3] * 1000),
            *rotation_matrix_to_rpy_deg(matrix[:3, :3])]


@dataclass(frozen=True)
class Joint:
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


class Cr10Kinematics:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.sha256 = file_sha256(self.path)
        robot = ET.parse(self.path).getroot()
        if robot.attrib.get("name") != "cr10_robot":
            raise ValueError("Only the canonical CR10 model is supported")
        self.joints = []
        parent = "base_link"
        for index, name in enumerate(JOINT_NAMES, 1):
            matches = robot.findall(f"joint[@name='{name}']")
            if len(matches) != 1:
                raise ValueError(f"CR10 model requires exactly one {name}")
            node = matches[0]
            if (node.attrib["type"] != "revolute"
                    or node.find("parent").attrib["link"] != parent
                    or node.find("child").attrib["link"] != f"Link{index}"):
                raise ValueError("CR10 model has an unsupported base_link <- Link6 chain")
            origin = np.eye(4)
            spec = node.find("origin").attrib
            origin[:3, 3] = [float(v) for v in spec["xyz"].split()]
            origin[:3, :3] = rpy_matrix(*map(float, spec["rpy"].split()))
            axis = np.array([float(v) for v in node.find("axis").attrib["xyz"].split()])
            if not np.allclose(axis, [0, 0, 1], atol=1e-12):
                raise ValueError("CR10 model requires each local revolute axis to be Z")
            limit = node.find("limit").attrib
            self.joints.append(Joint(origin, axis, float(limit["lower"]), float(limit["upper"])))
            parent = f"Link{index}"

    def forward(self, positions):
        if len(positions) != 6:
            raise ValueError("CR10 kinematics requires six Home/actual joints in radians")
        result = np.eye(4)
        for spec, angle in zip(self.joints, positions):
            if (type(angle) not in (float, int) or not math.isfinite(angle)
                    or not spec.lower <= angle <= spec.upper):
                raise ValueError("Joint is invalid or outside the canonical CR10 model limits")
            rotation = np.eye(4)
            rotation[:3, :3] = rpy_matrix(0, 0, angle)
            result = result @ spec.origin @ rotation
        return result
