"""Explicit calibration files and .env round trips without hardware or model execution."""

import stat
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from item_perception_yolo import item_teach_calibration as selection
from item_perception_yolo import item_detector as detector
from item_perception_yolo import platform_teach_core as platform
from test_station_calibration import _camera, _platform, camera_core, ROBOT_IP


@pytest.fixture
def files(tmp_path):
    content = (f"# Preserve operator settings\nROS_LOCALHOST_ONLY=1\n"
               f"DOBOT_ROBOT_LAN1_IP={ROBOT_IP}\n" +
               "".join(f"{key}=\n" for key in selection.CALIBRATION_ENV_KEYS))
    for name in (".env", ".env.example"):
        (tmp_path / name).write_text(content)
    (tmp_path / ".env").chmod(0o640)
    camera = _camera(tmp_path)
    custom = camera.path.with_name("camera_to_hand_calibration_station_2.yaml")
    camera.path.rename(custom)
    camera = platform.load_camera_calibration(custom, root=tmp_path)
    station = _platform(tmp_path, camera)
    robot = _camera(tmp_path, prefix="robot_camera", mode=camera_core.CAMERA_ON_HAND)
    return tmp_path, (station, custom, robot.path)


def test_explicit_files_ignore_catalog_names_and_preserve_env_settings(files):
    root, paths = files
    assert selection.saved_calibration_paths(root) is None
    # Neither a newer unrelated file nor a malformed catalog entry may replace explicit choices.
    _camera(root, stamp="20260920T083036_133658Z", prefix="robot_camera",
            mode=camera_core.CAMERA_ON_HAND)
    (root / "calibration/camera_to_hand_calibration_broken.yaml").write_text("not a calibration")
    applied, robot = selection.save_calibration_selection(*paths, root=root)
    assert applied.platform.path == paths[0] and applied.camera.path == paths[1]
    assert robot.path == paths[2]
    assert selection.saved_calibration_paths(root) == paths
    content = (root / ".env").read_text()
    assert content.startswith("# Preserve operator settings\nROS_LOCALHOST_ONLY=1\n")
    assert f"DOBOT_ROBOT_LAN1_IP={ROBOT_IP}\n" in content
    assert str(root) not in content
    assert stat.S_IMODE((root / ".env").stat().st_mode) == 0o640
    selection.validate_selected_robot_camera(robot, root=root)
    assert not list(root.glob(".env.item_teach.*"))


@pytest.mark.parametrize("failure", ["camera_pair", "robot_mode", "missing", "symlink",
                                     "outside", "native_write", "env_changed"])
def test_invalid_selection_or_write_preserves_previous_env(files, monkeypatch, failure):
    root, paths = files
    selection.save_calibration_selection(*paths, root=root)
    original = (root / ".env").read_bytes()
    attempted = list(paths)
    if failure == "camera_pair":
        attempted[1] = _camera(root, stamp="20260921T083036_133658Z").path
    elif failure == "robot_mode":
        attempted[2] = paths[1]
    elif failure == "missing":
        attempted[2] = paths[2].with_name("missing.yaml")
    elif failure == "symlink":
        attempted[2] = paths[2].with_name("camera_on_hand_calibration_alias.yaml")
        attempted[2].symlink_to(paths[2])
    elif failure == "outside":
        attempted[2] = root / paths[2].name
        attempted[2].write_bytes(paths[2].read_bytes())
    elif failure == "native_write":
        def fail(*_args):
            raise OSError("Cannot replace .env")
        monkeypatch.setattr(selection.os, "replace", fail)
    else:
        real_fsync = selection.os.fsync

        def modify(fd):
            real_fsync(fd)
            (root / ".env").write_bytes(original + b"# Concurrent operator change\n")
        monkeypatch.setattr(selection.os, "fsync", modify)
    with pytest.raises((ValueError, OSError)):
        selection.save_calibration_selection(*attempted, root=root)
    expected = (original + b"# Concurrent operator change\n"
                if failure == "env_changed" else original)
    assert (root / ".env").read_bytes() == expected
    assert not list(root.glob(".env.item_teach.*"))


@pytest.mark.parametrize("bad", ["../escape.yaml", "/tmp/test.yaml", "name=bad.yaml", "incomplete"])
def test_invalid_saved_env_is_not_replaced_by_discovery(files, bad):
    root, _paths = files
    key = selection.CALIBRATION_ENV_KEYS[0]
    value = "platform.yaml" if bad == "incomplete" else bad
    path = root / ".env"
    path.write_text(path.read_text().replace(f"{key}=\n", f"{key}={value}\n"))
    with pytest.raises(ValueError):
        selection.saved_calibration_paths(root)


def test_selected_robot_camera_content_change_is_rejected(files):
    root, paths = files
    _applied, robot = selection.load_calibration_selection(*paths, root=root)
    paths[2].write_text(paths[2].read_text() + "\n# modified\n")
    with pytest.raises(ValueError, match="changed"):
        selection.validate_selected_robot_camera(robot, root=root)


def test_gui_keeps_explicit_robot_camera_and_disarms_on_change(files, monkeypatch):
    root, paths = files
    applied, robot = selection.load_calibration_selection(*paths, root=root)
    _camera(root, stamp="20260920T083036_133658Z", prefix="robot_camera",
            mode=camera_core.CAMERA_ON_HAND)
    bin_path = root / "bin.yaml"
    bin_path.write_text("synthetic bin")
    node = SimpleNamespace(
        applied=applied, robot_camera=robot, root=root, explicit_robot_camera=True,
        bin_artifact=SimpleNamespace(path=bin_path, sha256=detector.file_sha256(bin_path)),
        last_view=object(), service=object(), disarm=MagicMock())
    validate = detector.validate_applied_sources
    monkeypatch.setattr(detector, "validate_applied_sources",
                        lambda pair: validate(pair, root=root))
    detector.ItemDetectNode._validate_sources(node)
    node.disarm.assert_not_called()
    paths[2].write_text(paths[2].read_text() + "\n# changed selected camera\n")
    with pytest.raises(ValueError, match="changed"):
        detector.ItemDetectNode._validate_sources(node)
    assert node.last_view is None
    node.disarm.assert_called_once()


def test_missing_config_key_is_not_defaulted(files):
    root, _paths = files
    env = root / ".env"
    env.write_text(env.read_text().replace(f"{selection.CALIBRATION_ENV_KEYS[0]}=\n", ""))
    with pytest.raises(ValueError, match="missing"):
        selection.saved_calibration_paths(root)
