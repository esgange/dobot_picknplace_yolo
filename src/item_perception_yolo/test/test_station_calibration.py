"""Automatic station selection with real strict synthetic calibration artifacts."""

from datetime import datetime, timezone
import os
from pathlib import Path
import runpy
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
import yaml

from camera_calibration_gui import calibration_core as camera_core
from item_perception_yolo import item_detector as detector
from item_perception_yolo import platform_teach_core as platform_core
from item_perception_yolo import station_calibration as station


ROBOT_IP = "192.168.20.204"


def _utc(stamp):
    return datetime.strptime(stamp, "%Y%m%dT%H%M%S_%fZ").replace(tzinfo=timezone.utc)


def _pose(degrees=0.0):
    angle = np.deg2rad(degrees)
    result = np.eye(4)
    result[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    result[:3, 3] = [0.1, 0.2, 0.7]
    return result


def _camera(root, stamp="20260909T083036_133658Z", prefix="bin_camera",
            mode=camera_core.CAMERA_TO_HAND):
    settings = camera_core.CharucoSettings(prefix, "DICT_5X5_50", 5, 7, 30.0, 22.0)
    samples = [camera_core.CalibrationSample(f"C{i + 1}", _pose(i * 20), _pose(i * 20),
                                             tuple(j / 10 for j in range(6)))
               for i in range(5)]
    residuals = tuple(camera_core.SampleResidual(s.sample_id, 0.0, 0.0) for s in samples)
    quality = camera_core.CalibrationQuality(5, 0.0, 0.0, 0.0, "C1", 0.0, "C1",
                                             residuals, np.eye(4))
    diagnostics = camera_core.AccuracyDiagnostics(
        quality, camera_core.SolutionChangeDiagnostics(False, None, None, None, None),
        camera_core.LeaveOneOutDiagnostics(
            "not_available", (), (), None, None, None, None, None, None),
        camera_core.PoseCoverageDiagnostics(100.0, 20.0),
        camera_core.AxXbDiagnostics(4, 0.0, 0.0))
    path = root / "calibration" / f"{mode}_calibration_{stamp}.yaml"
    camera_core.write_calibration_yaml(path, mode, settings, _pose(), diagnostics,
                                       samples, created_at=_utc(stamp))
    return platform_core.load_camera_calibration(path, root=root)


def _platform(root, camera, stamp="20260910T120000_000000Z", robot_ip=ROBOT_IP):
    on_hand = camera.calibration_mode == camera_core.CAMERA_ON_HAND
    base_from_tool = _pose(10) if on_hand else None
    base_from_camera = platform_core.resolve_base_from_camera_link(camera, base_from_tool)
    optical_from_board = _pose(20)
    capture = platform_core.PlatformCapture(
        captured_at_utc=_utc(stamp).isoformat(timespec="microseconds").replace("+00:00", "Z"),
        color_stamp_sec=123, color_stamp_nanosec=456, frame_sequence=1, charuco_corner_count=12,
        calibration_mode=camera.calibration_mode,
        calibration_reference_from_camera_link=camera.reference_from_camera_link.copy(),
        base_from_tool=base_from_tool, robot_tf_stamp_sec=122 if on_hand else None,
        robot_tf_stamp_nanosec=900 if on_hand else None,
        base_from_camera_link=base_from_camera, camera_link_from_optical=np.eye(4),
        optical_from_board=optical_from_board,
        base_from_platform=base_from_camera @ optical_from_board)
    path = platform_core.platform_output_path(robot_ip, root=root, created_at=_utc(stamp))
    platform_core.write_platform_calibration(path, camera, robot_ip, capture, root=root)
    return path


@pytest.fixture
def root(tmp_path):
    content = f"ROS_LOCALHOST_ONLY=1\nDOBOT_ROBOT_LAN1_IP={ROBOT_IP}\n"
    (tmp_path / ".env.example").write_text(content, encoding="utf-8")
    (tmp_path / ".env").write_text(content, encoding="utf-8")
    return tmp_path


@pytest.mark.parametrize("mode", (camera_core.CAMERA_TO_HAND, camera_core.CAMERA_ON_HAND))
def test_latest_pair_uses_current_robot_prefix_and_utc_not_mtime(root, mode):
    camera = _camera(root, mode=mode)
    old = _platform(root, camera, stamp="20260909T120000_000000Z")
    latest = _platform(root, camera)
    _platform(root, camera, stamp="20260911T120000_000000Z", robot_ip="192.168.20.205")
    _camera(root, stamp="20260912T120000_000000Z", prefix="robot_camera")
    os.utime(old, (2_000_000_000, 2_000_000_000))
    os.utime(latest, (1, 1))
    applied = station.latest_station_calibration(root)
    assert applied.platform.path == latest
    assert applied.camera.path == camera.path
    assert applied.camera.sha256 == camera.sha256
    np.testing.assert_allclose(applied.camera.reference_from_camera_link,
                               camera.reference_from_camera_link)
    assert applied.camera.calibration_mode == mode
    assert applied.platform.camera_settings.camera_prefix == "bin_camera"


@pytest.mark.parametrize("mode", (camera_core.CAMERA_TO_HAND, camera_core.CAMERA_ON_HAND))
def test_newer_same_prefix_camera_requires_new_platform_no_older_fallback(root, mode):
    old_camera = _camera(root)
    _platform(root, old_camera)
    latest_camera = _camera(root, stamp="20260911T120000_000000Z", mode=mode)
    with pytest.raises(ValueError, match="Teach a new platform"):
        station.latest_station_calibration(root)
    latest_platform = _platform(root, latest_camera, stamp="20260912T120000_000000Z")
    assert station.latest_station_calibration(root).platform.path == latest_platform


@pytest.mark.parametrize("corruption", ("schema", "hash", "transform", "malformed", "missing"))
def test_invalid_newest_platform_never_falls_back(root, corruption):
    camera = _camera(root)
    _platform(root, camera, stamp="20260909T120000_000000Z")
    latest = _platform(root, camera)
    payload = yaml.safe_load(latest.read_text())
    if corruption == "schema":
        payload["schema_version"] = 2
    elif corruption == "hash":
        payload["camera_calibration"]["sha256"] = "0" * 64
    elif corruption == "transform":
        payload["transform"] = {}
    elif corruption == "missing":
        payload["camera_calibration"]["filename"] = (
            "camera_to_hand_calibration_20260908T120000_000000Z.yaml")
    latest.write_text("[broken" if corruption == "malformed" else yaml.safe_dump(payload))
    with pytest.raises(ValueError):
        station.latest_station_calibration(root)


def test_invalid_latest_camera_is_not_skipped(root):
    camera = _camera(root)
    _platform(root, camera)
    latest = _camera(root, stamp="20260911T120000_000000Z")
    payload = yaml.safe_load(latest.path.read_text())
    payload["schema_version"] = 6
    latest.path.write_text(yaml.safe_dump(payload))
    with pytest.raises(ValueError, match="schema_version must be exactly 7"):
        station.latest_station_calibration(root)


def test_ambiguous_same_prefix_camera_timestamp_fails(root):
    _platform(root, _camera(root))
    _camera(root, mode=camera_core.CAMERA_ON_HAND)
    with pytest.raises(ValueError, match="Ambiguous latest camera"):
        station.latest_station_calibration(root)


@pytest.mark.parametrize("artifact", ("camera", "platform"))
def test_filename_and_payload_timestamps_must_match(root, artifact):
    camera = _camera(root)
    platform = _platform(root, camera)
    if artifact == "platform":
        target = platform.with_name(platform.name.replace("20260910", "20260911"))
        platform.rename(target)
    else:
        target = camera.path.with_name(camera.path.name.replace("20260909", "20260911"))
        camera.path.rename(target)
        payload = yaml.safe_load(platform.read_text())
        payload["camera_calibration"]["filename"] = target.name
        platform.write_text(yaml.safe_dump(payload))
    with pytest.raises(ValueError, match="filename timestamp conflicts|filename must be exactly"):
        station.latest_station_calibration(root)


@pytest.mark.parametrize("kind", ("symlink", "bad_name", "missing_prefix", "duplicate_prefix"))
def test_unidentifiable_camera_catalog_is_terminal_not_a_fallback(root, kind):
    camera = _camera(root)
    _platform(root, camera)
    target = root / "calibration/camera_to_hand_calibration_20260911T120000_000000Z.yaml"
    if kind == "symlink":
        target.symlink_to(camera.path)
    elif kind == "bad_name":
        target = target.with_name("camera_to_hand_calibration_not_a_timestamp.yaml")
        target.write_text("camera: {prefix: bin_camera}\n")
    else:
        target.write_text("camera: {}\n" if kind == "missing_prefix" else
                          "camera: {prefix: bin_camera, prefix: robot_camera}\n")
    with pytest.raises(ValueError, match="regular local file|Noncanonical|Cannot identify"):
        station.latest_station_calibration(root)


def test_missing_current_station_has_no_other_robot_or_env_fallback(root):
    _platform(root, _camera(root), robot_ip="192.168.20.205")
    with pytest.raises(ValueError, match=f"No platform for robot {ROBOT_IP}"):
        station.latest_station_calibration(root)
    (root / ".env").unlink()
    with pytest.raises(ValueError, match="Required project configuration is not a file"):
        station.latest_station_calibration(root)


@pytest.mark.parametrize("changed", ("platform", "camera"))
def test_selected_hash_changes_before_application_cannot_replace_binding(
        root, monkeypatch, changed):
    _platform(root, _camera(root))
    expected = station.latest_station_calibration(root)
    current = SimpleNamespace(platform=expected.platform, camera=expected.camera)
    artifact = getattr(current, changed)
    setattr(current, changed, SimpleNamespace(path=artifact.path, sha256="0" * 64))
    monkeypatch.setattr(detector, "load_bin_teach_calibration_context", lambda _: current)
    reader = MagicMock()
    monkeypatch.setattr(detector, "load_bin_teach", reader)
    node = SimpleNamespace(disarm=MagicMock(), connect_camera=MagicMock(), applied=None)
    with pytest.raises(ValueError, match="changed before application"):
        detector.ItemDetectNode.apply_station(
            node, expected.platform.path, "bin.yaml", expected_station=expected)
    reader.assert_not_called()
    node.connect_camera.assert_not_called()
    assert node.applied is None


@pytest.mark.parametrize("selection_error", (False, True))
def test_headless_main_uses_automatic_station_and_keeps_explicit_trust(
        root, monkeypatch, selection_error):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    _platform(root, _camera(root))
    latest = station.latest_station_calibration(root)
    selection = MagicMock(return_value=latest)
    if selection_error:
        selection.side_effect = ValueError("Latest calibration invalid")
    monkeypatch.setattr(detector, "latest_station_calibration", selection)
    values = {"item_teach_file": "/selected/item.yaml", "bin_teach_file": "/selected/bin.yaml",
              "trusted_model": True, "armed": True}
    node = MagicMock(fatal_error=None)
    node.settings = {"quality": {"request_timeout_sec": 10}}
    node.declare_parameter.side_effect = lambda key, _: SimpleNamespace(value=values[key])
    monkeypatch.setattr(detector, "ItemDetectNode", MagicMock(return_value=node))
    monkeypatch.setattr(detector, "MultiThreadedExecutor", MagicMock())
    monkeypatch.setattr(detector.threading, "Thread", MagicMock())
    monkeypatch.setattr(detector.rclpy, "init", MagicMock())
    monkeypatch.setattr(detector.rclpy, "ok", lambda: False)
    profile = {"model": {"filename": "item.pt"}}
    monkeypatch.setattr(detector, "load_item_profile", lambda _: (profile, "c"))
    monkeypatch.setattr(detector, "settings_from_profile", lambda _: {})
    monkeypatch.setattr(detector, "detection_settings", lambda _: {})
    if selection_error:
        with pytest.raises(ValueError, match="Latest calibration invalid"):
            detector.main()
        node.inspect_model.assert_not_called()
        node.arm.assert_not_called()
    else:
        detector.main()
        node.apply_station.assert_called_once_with(
            latest.platform.path, "/selected/bin.yaml", expected_station=latest)
        node.arm.assert_called_once_with(Path("/selected/item.yaml"))
        node._snapshot.assert_called_once()
        selection.assert_called_once_with()
        declared = [call.args[0] for call in node.declare_parameter.call_args_list]
        assert "platform_teach_file" not in declared
    node.close_runtime.assert_called_once()


def test_headless_launch_requires_no_platform_path(monkeypatch):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    from launch.actions import DeclareLaunchArgument
    launch_path = Path(__file__).parents[1] / "launch/item_detect.launch.py"
    description = runpy.run_path(str(launch_path))["generate_launch_description"]()
    arguments = [action for action in description.entities
                 if isinstance(action, DeclareLaunchArgument)]
    assert [argument.name for argument in arguments] == [
        "item_teach_file", "bin_teach_file", "armed", "trusted_model"]
    for argument in arguments[2:]:
        assert argument.default_value[0].text == "false"
