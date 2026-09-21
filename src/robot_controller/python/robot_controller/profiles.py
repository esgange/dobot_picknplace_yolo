"""Strict controller artifact selection; no weights are deserialized here."""

from dataclasses import dataclass
from pathlib import Path

from item_perception_yolo.bin_teach_core import (
    load_bin_teach, place_bin_roi, validate_applied_sources, bin_platform_warning)
from item_perception_yolo.item_teach_core import file_sha256, load_item_profile
from item_perception_yolo.runtime_teach import runtime_teach_catalog
from item_perception_yolo.station_calibration import (
    latest_station_calibration, latest_robot_camera_calibration,
    validate_robot_camera_calibration)


@dataclass(frozen=True)
class Selection:
    item_path: Path
    item: dict
    item_sha256: str
    bin: object
    station: object
    robot_camera: object
    deployment: bool

    def validate(self, root):
        profile, digest = load_item_profile(self.item_path, root=root, deployment=self.deployment)
        if digest != self.item_sha256 or profile != self.item:
            raise ValueError("Selected Item Teach/model changed; reload before using it")
        validate_applied_sources(self.station, root=root)
        validate_robot_camera_calibration(self.robot_camera, root=root)
        if file_sha256(self.bin.path) != self.bin.sha256:
            raise ValueError("Selected Bin Teach changed; reload before using it")

    def warning(self):
        return bin_platform_warning(self.bin, self.station.platform)


def load_selection(item, bin_path, root, *, deployment=False):
    profile, digest = load_item_profile(Path(item), root=root, deployment=deployment)
    template = load_bin_teach(Path(bin_path), root=root, deployment=deployment)
    station = latest_station_calibration(root=root)
    robot_camera = latest_robot_camera_calibration(root=root)
    place_bin_roi(template, station.platform)
    selected = Selection(Path(item).resolve(), profile, digest, template, station,
                         robot_camera, deployment)
    selected.validate(root)
    return selected


def runtime_selection(root):
    catalog = runtime_teach_catalog(root)
    return load_selection(catalog.item_yaml, catalog.bin_yaml, root, deployment=True)
