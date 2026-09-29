"""Immutable, hash-addressed controller configuration snapshots."""

from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np

from item_perception_yolo.item_teach_core import load_item_profile
from item_perception_yolo.runtime_teach import runtime_teach_catalog

from .profiles import load_selection, runtime_selection
from .tray import TrayConfiguration, load_tray_configuration


@dataclass(frozen=True)
class ControllerConfiguration:
    item_path: Path
    bin_path: Path | None
    profile: dict
    profile_sha256: str
    home_joints: tuple
    home_matrix: np.ndarray
    configuration_id: str
    selection: object | None
    deployment: bool
    tray: TrayConfiguration | None = None

    @property
    def pose_candidates(self):
        return self.profile["retry"]["pose_candidates"]

    def validate_sources(self, root):
        profile, digest = load_item_profile(
            self.item_path, root=root, deployment=self.deployment)
        if digest != self.profile_sha256 or profile != self.profile:
            raise ValueError("Configured Item Teach/model changed; configure again")
        if self.selection is not None:
            self.selection.validate(root)
        if self.tray is not None:
            self.tray.validate_sources(root)


def _identifier(profile_digest, selection, tray=None):
    evidence = {
        "item": profile_digest,
        "model": selection.item["model"]["sha256"] if selection else "",
        "bin": selection.bin.sha256 if selection else "",
        "platform": selection.station.platform.sha256 if selection else "",
        "camera": selection.station.camera.sha256 if selection else "",
        "robot_camera": selection.robot_camera.sha256 if selection else "",
        "tray": tray.sha256 if tray else "",
        "tray_camera": tray.camera_sha256 if tray else "",
    }
    return hashlib.sha256(json.dumps(
        evidence, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()


def load_configuration(item_path, bin_path, root, kinematics, *, deployment=False, tray_path=""):
    item_path = Path(item_path).expanduser().resolve()
    bin_path = Path(bin_path).expanduser().resolve() if bin_path else None
    if bin_path is None:
        profile, digest = load_item_profile(item_path, root=root, deployment=deployment)
        selection = None
    else:
        selection = load_selection(item_path, bin_path, root, deployment=deployment)
        profile, digest = selection.item, selection.item_sha256
    joints = tuple(profile["home"]["positions_rad"])
    home = kinematics.forward(joints)
    tray = load_tray_configuration(tray_path, root, kinematics, deployment=deployment)
    result = ControllerConfiguration(
        item_path=item_path, bin_path=bin_path, profile=deepcopy(profile),
        profile_sha256=digest, home_joints=joints, home_matrix=home.copy(),
        configuration_id=_identifier(digest, selection, tray), selection=selection,
        deployment=deployment, tray=tray)
    result.validate_sources(root)
    return result


def load_runtime_configuration(root, kinematics):
    selection = runtime_selection(root)
    tray = load_tray_configuration(
        runtime_teach_catalog(root).tray_yaml, root, kinematics, deployment=True)
    joints = tuple(selection.item["home"]["positions_rad"])
    result = ControllerConfiguration(
        item_path=selection.item_path, bin_path=selection.bin.path,
        profile=deepcopy(selection.item), profile_sha256=selection.item_sha256,
        home_joints=joints, home_matrix=kinematics.forward(joints),
        configuration_id=_identifier(selection.item_sha256, selection, tray),
        selection=selection, deployment=True, tray=tray)
    result.validate_sources(root)
    return result
