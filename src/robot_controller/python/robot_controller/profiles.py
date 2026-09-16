"""Strict controller artifact selection; no weights are deserialized here."""

from dataclasses import dataclass
from pathlib import Path

import yaml

from item_perception_yolo.bin_teach_core import (
    load_bin_teach, place_bin_roi, validate_applied_sources, bin_platform_warning)
from item_perception_yolo.item_teach_core import (
    _UniqueKeyLoader, file_sha256, load_item_profile)
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
    directory = Path(root).resolve() / "runtime_teach"
    if directory.is_symlink() or not directory.is_dir():
        raise ValueError("Required flat root runtime_teach/ directory is missing or symlinked")
    artifacts = {"item_teach": [], "bin_teach": []}
    for path in sorted(directory.iterdir()):
        if path.name.startswith("."):
            continue
        if path.is_symlink() or not path.is_file():
            raise ValueError("runtime_teach/ requires regular files only; no partitions/symlinks")
        if path.suffix == ".pt":
            continue
        if path.suffix != ".yaml":
            raise ValueError(f"Unsupported runtime_teach file: {path.name}")
        try:
            payload = yaml.load(path.read_bytes(), Loader=_UniqueKeyLoader)
            kind = payload["artifact_type"]
            artifacts[kind].append(path)
        except (OSError, yaml.YAMLError, TypeError, KeyError, ValueError) as exc:
            raise ValueError(f"Invalid/unsupported runtime teach: {path.name}: {exc}") from exc
    if any(len(paths) != 1 for paths in artifacts.values()):
        raise ValueError("Pick-only stage requires exactly one item and one bin in runtime_teach/")
    selected = load_selection(artifacts["item_teach"][0], artifacts["bin_teach"][0], root,
                              deployment=True)
    weights = {p.name for p in directory.glob("*.pt")}
    if weights != {selected.item["model"]["filename"]}:
        raise ValueError("runtime_teach/ must contain exactly the selected item's paired .pt")
    return selected
