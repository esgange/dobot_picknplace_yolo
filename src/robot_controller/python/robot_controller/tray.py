"""Controller binding to a saved Tray Teach; never execute model weights here."""

from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from item_perception_yolo.item_teach_core import file_sha256
from tray_perception.calibration import active_robot_camera
from tray_perception.core import load_profile


@dataclass(frozen=True)
class TrayConfiguration:
    path: Path
    profile: dict
    sha256: str
    camera_path: Path
    camera_sha256: str
    detect_joints: tuple | None
    detect_matrix: np.ndarray | None
    deployment: bool

    def validate_sources(self, root):
        profile = load_profile(self.path, root, deployment=self.deployment)
        if profile != self.profile or file_sha256(self.path) != self.sha256:
            raise ValueError("Configured Tray Teach/model changed; configure again")
        if file_sha256(self.camera_path) != self.camera_sha256:
            raise ValueError("Configured tray camera calibration changed; configure again")


def load_tray_configuration(path, root, kinematics, *, deployment=False):
    if not path:
        return None
    profile = load_profile(path, root, deployment=deployment)
    path = Path(path).expanduser().resolve()
    camera = active_robot_camera(root)
    position = profile["tray_teach_position"]
    joints = tuple(position["positions_rad"]) if position is not None else None
    matrix = kinematics.forward(joints) if joints is not None else None
    result = TrayConfiguration(
        path, deepcopy(profile), file_sha256(path), camera.path, camera.sha256,
        joints, matrix, deployment)
    result.validate_sources(root)
    return result
