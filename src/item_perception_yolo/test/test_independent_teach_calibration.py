"""Camera replacement preserves taught robot/platform/bin geometry and provenance."""

from datetime import datetime, timezone
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
import yaml

from item_perception_yolo import bin_teach_core as bin_core
from item_perception_yolo import item_teach_calibration as selection
from item_perception_yolo import platform_teach_core as platform_core
from item_perception_yolo import platform_teach_gui as platform_gui
from test_bin_teach import _capture
from test_station_calibration import _camera, _platform, _pose, camera_core, ROBOT_IP


@pytest.fixture
def station(tmp_path):
    content = (f"ROS_LOCALHOST_ONLY=1\nDOBOT_ROBOT_LAN1_IP={ROBOT_IP}\n" +
               "".join(f"{key}=\n" for key in selection.CALIBRATION_ENV_KEYS))
    for name in (".env", ".env.example"):
        (tmp_path / name).write_text(content)
    old_camera = _camera(tmp_path)
    platform_path = _platform(tmp_path, old_camera)
    applied = bin_core.load_bin_teach_calibration_context(
        platform_path, root=tmp_path, camera_path=old_camera.path)
    robot_camera = _camera(tmp_path, prefix="robot_camera", mode=camera_core.CAMERA_ON_HAND)
    capture = _capture(applied)
    bin_path = bin_core.bin_output_path(
        ROBOT_IP, root=tmp_path,
        created_at=datetime(2026, 9, 10, 12, 34, 56, 123456, tzinfo=timezone.utc))
    bin_core.write_bin_teach(
        bin_path, applied, bin_core.BinArucoSettings("DICT_5X5_50", 40.), capture, root=tmp_path)
    return tmp_path, applied, robot_camera, bin_path


def test_moved_camera_updates_projection_without_reteaching_or_old_camera_file(station):
    root, original, robot_camera, bin_path = station
    original_bytes = original.platform.path.read_bytes(), bin_path.read_bytes()
    old_bin = bin_core.load_bin_teach(bin_path, root=root)
    moved = _pose(35)
    moved[:3, 3] += [.2, -.1, .15]
    replacement = _camera(root, stamp="20260929T120000_000000Z", transform=moved)
    original.camera.path.unlink()
    active, _robot = selection.save_calibration_selection(
        original.platform.path, replacement.path, robot_camera.path, root=root)
    bin_core.validate_applied_sources(active, root=root)
    assert bin_core.active_bin_camera_path(root) == replacement.path
    assert bin_core.load_bin_teach_calibration_context(
        original.platform.path, root=root).camera.path == replacement.path
    assert original_bytes == (original.platform.path.read_bytes(), bin_path.read_bytes())
    np.testing.assert_array_equal(active.platform.base_from_platform,
                                  original.platform.base_from_platform)
    loaded = bin_core.load_bin_teach(bin_path, root=root)
    assert loaded.points == old_bin.points
    np.testing.assert_array_equal(bin_core.place_bin_roi(loaded, active.platform),
                                  bin_core.place_bin_roi(old_bin, original.platform))
    internal_tf = _pose(90)
    optical_to_platform = bin_core.compose_platform_from_optical(
        active.platform.base_from_platform, moved, internal_tf)
    old_projection = bin_core.compose_platform_from_optical(
        original.platform.base_from_platform, original.camera.reference_from_camera_link,
        internal_tf)
    assert not np.allclose(old_projection, optical_to_platform)
    point = np.array([loaded.points[0].x_m, loaded.points[0].y_m, 0., 1.])
    world_point = original.platform.base_from_platform @ point
    new_camera_point = np.linalg.inv(moved @ internal_tf) @ world_point
    np.testing.assert_allclose(optical_to_platform @ new_camera_point, point, atol=1e-12)
    # Each active source remains immutable until an explicit reload.
    replacement.path.write_text(replacement.path.read_text() + "\n# changed while armed\n")
    with pytest.raises(ValueError, match="camera calibration changed"):
        bin_core.validate_applied_sources(active, root=root)


def test_new_bin_creation_uses_active_camera_and_loading_needs_no_source_files(station):
    root, original, _robot, old_bin_path = station
    replacement = _camera(root, stamp="20260929T120000_000000Z", transform=_pose(40))
    original.camera.path.unlink()
    active = bin_core.load_bin_teach_calibration_context(
        original.platform.path, root=root, camera_path=replacement.path)
    capture = _capture(active)
    old_bin_path.unlink()
    bin_core.write_bin_teach(
        old_bin_path, active, bin_core.BinArucoSettings("DICT_5X5_50", 40.), capture, root=root)
    payload = yaml.safe_load(old_bin_path.read_text())
    provenance = payload["teaching_provenance"]
    assert provenance["camera_calibration"]["filename"] == replacement.path.name
    assert provenance["platform_calibration"]["schema_version"] == 4
    replacement.path.unlink()
    original.platform.path.unlink()
    assert bin_core.load_bin_teach(old_bin_path, root=root).points == capture.points


def test_explicit_legacy_platform_migration_preserves_transform_and_teaching_history(station):
    root, original, _robot, _bin_path = station
    path = original.platform.path
    current = yaml.safe_load(path.read_text())
    legacy = {key: value for key, value in current.items() if key != "teaching_provenance"}
    legacy.update(current["teaching_provenance"])
    legacy["schema_version"] = 3
    path.write_text(yaml.safe_dump(legacy))
    original.camera.path.unlink()
    old = platform_core.load_platform_calibration(path, root=root)
    assert old.schema_version == 3
    migrated = platform_core.independent_platform_payload(legacy)
    assert migrated == current
    assert platform_core.independent_platform_payload(migrated) == migrated
    path.write_text(yaml.safe_dump(migrated))
    new = platform_core.load_platform_calibration(path, root=root)
    assert new.schema_version == 4
    np.testing.assert_array_equal(new.base_from_platform, old.base_from_platform)
    assert new.camera_calibration_filename == old.camera_calibration_filename
    template = bin_core.load_bin_teach(_bin_path, root=root)
    assert template.source_platform_calibration_sha256 != new.sha256
    assert bin_core.bin_platform_warning(template, new) is None
    moved = new.base_from_platform.copy()
    moved[0, 3] += .001
    assert bin_core.bin_platform_warning(template, replace(new, base_from_platform=moved))
    assert bin_core.bin_platform_warning(template, replace(new, robot_lan1_ip="192.0.2.99"))


def test_platform_startup_reminder_explains_fresh_calibration_and_reuse(monkeypatch):
    show = MagicMock()
    monkeypatch.setattr(platform_gui.QtWidgets.QMessageBox, "information", show)
    node = SimpleNamespace(apply_calibration=MagicMock(), capture_platform=MagicMock())
    window = SimpleNamespace(_node=node)
    platform_gui.PlatformTeachWindow.show_calibration_reminder(window)
    text = show.call_args.args[2]
    assert "freshly calibrate the bin camera" in text
    assert "robot base, platform and bin have not moved" in text
    node.apply_calibration.assert_not_called()
    node.capture_platform.assert_not_called()
