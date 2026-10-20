"""Explicit GUI deployment of one complete, validated detector catalog."""

from pathlib import Path
import os
import shutil
import tempfile

from item_perception_yolo.bin_teach_core import load_bin_teach, place_bin_roi
from item_perception_yolo.item_teach_core import file_sha256, load_item_profile
from item_perception_yolo.item_teach_calibration import (
    load_calibration_selection, saved_calibration_paths)
from item_perception_yolo.runtime_teach import runtime_teach_catalog
from tray_perception.core import load_profile
from item_perception_yolo.source_binding import fingerprints


def selected_files(root, selections, *, deployment=False):
    """Validate production sources and calibration without executing model weights."""
    if len(selections) != 3 or not all(selections):
        raise ValueError("Select Item Teach, Bin Teach and Tray Teach first")
    item, bin_path, tray = (Path(p).expanduser().absolute() for p in selections)
    paths = (item, item.with_suffix('.pt'), bin_path, tray, tray.with_suffix('.pt'))
    before = fingerprints(paths)
    load_item_profile(item, root=root, deployment=deployment)
    bin_artifact = load_bin_teach(bin_path, root=root, deployment=deployment)
    load_profile(tray, root, deployment=deployment)
    calibration = saved_calibration_paths(root)
    if calibration is None:
        raise ValueError("Select and save all three calibrations in Item Teach first")
    station, _camera = load_calibration_selection(*calibration, root=root)
    place_bin_roi(bin_artifact, station.platform)
    if len({path.name for path in paths}) != 5:
        raise ValueError("Teach artifact filenames must be distinct")
    if any(path.is_symlink() or not path.is_file() for path in paths):
        raise ValueError("Load Auto requires regular source files, not symlinks")
    result = {path: file_sha256(path) for path in paths}
    if fingerprints(paths) != before:
        raise ValueError("Teach files changed during validation; explicitly reload")
    return result


def runtime_paths(root):
    """Read the complete externally/local deployed catalog, without modifying it."""
    catalog = runtime_teach_catalog(root)
    if catalog.tray_yaml is None:
        raise ValueError("Auto requires a complete Tray Teach YAML/model pair in runtime_teach/")
    return catalog.item_yaml, catalog.bin_yaml, catalog.tray_yaml


def validated_runtime(root):
    paths = runtime_paths(root)
    item, bin_path, tray = paths
    before = fingerprints((item, item.with_suffix('.pt'), bin_path, tray, tray.with_suffix('.pt')))
    files = selected_files(root, paths, deployment=True)
    binding = fingerprints(files)
    if runtime_paths(root) != paths or binding != before:
        raise ValueError("Runtime catalog changed during validation; explicitly reload")
    return paths, files, binding


def check_runtime(root, paths, files, binding):
    if runtime_paths(root) != paths or fingerprints(files) != binding:
        raise ValueError("Runtime recipe changed; turn Load Auto OFF and explicitly reload")


def deploy(root, selections, check):
    """Stage and verify all five files before replacing the complete runtime folder.

    A rejected selection or interrupted copy leaves the previous catalog intact.
    The only published destination is the exact root runtime_teach directory.
    """
    root = Path(root).resolve(strict=True)
    target = root / 'runtime_teach'
    if target.is_symlink() or (target.exists() and not target.is_dir()):
        raise ValueError("runtime_teach must be a real directory, not a symlink or file")
    check()
    sources = selected_files(root, selections)
    with tempfile.TemporaryDirectory(prefix='.load_auto-', dir=root) as temporary:
        staging_root = Path(temporary)
        staged = staging_root / 'runtime_teach'
        staged.mkdir()
        for source, digest in sources.items():
            check()
            destination = staged / source.name
            with source.open('rb') as reader, destination.open('xb') as writer:
                while block := reader.read(1024 * 1024):
                    check()
                    writer.write(block)
                writer.flush()
                os.fsync(writer.fileno())
            if file_sha256(destination) != digest or file_sha256(source) != digest:
                raise ValueError(f"Teach file changed during deployment: {source.name}")
        catalog = runtime_teach_catalog(staging_root)
        if catalog.tray_yaml is None:
            raise ValueError("Load Auto requires a complete tray pair")
        check()
        # Revalidate source bindings immediately before publication.
        if selected_files(root, selections) != sources:
            raise ValueError("Teach selection changed during deployment")
        check()
        if target.is_symlink() or (target.exists() and not target.is_dir()):
            raise ValueError("runtime_teach changed during deployment")
        backup = staging_root / 'previous'
        previous = target.exists()
        if previous:
            target.rename(backup)
        try:
            staged.rename(target)
        except BaseException:
            if previous:
                backup.rename(target)
            raise
        if previous:
            shutil.rmtree(backup)
    return {path.name: digest for path, digest in sources.items()}
