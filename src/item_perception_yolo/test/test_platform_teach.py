import hashlib
import json
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from camera_calibration_gui.calibration_core import CAMERA_ON_HAND, CAMERA_TO_HAND, CharucoSettings
from item_perception_yolo import platform_teach_core as core


def _rotation_z(degrees: float) -> np.ndarray:
    radians = np.deg2rad(degrees)
    cosine = np.cos(radians)
    sine = np.sin(radians)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _transform(translation, rotation_degrees=0.0) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = _rotation_z(rotation_degrees)
    matrix[:3, 3] = translation
    return matrix


def _settings() -> CharucoSettings:
    return CharucoSettings("bin_camera", "DICT_5X5_50", 5, 7, 30.0, 22.0)


def _applied(
    root: Path,
    mode: str = CAMERA_TO_HAND,
) -> core.AppliedCameraCalibration:
    prefix = "camera_to_hand" if mode == CAMERA_TO_HAND else "camera_on_hand"
    calibration_path = root / "calibration" / f"{prefix}_calibration_test.yaml"
    calibration_path.parent.mkdir(parents=True, exist_ok=True)
    calibration_path.write_text("strict camera calibration bytes\n", encoding="utf-8")
    reference_frame = "base_link" if mode == CAMERA_TO_HAND else "Link6"
    return core.AppliedCameraCalibration(
        path=calibration_path,
        sha256=hashlib.sha256(calibration_path.read_bytes()).hexdigest(),
        created_at_utc="2026-09-09T08:30:36.133658Z",
        calibration_mode=mode,
        reference_frame=reference_frame,
        settings=_settings(),
        reference_from_camera_link=_transform([0.4, -0.1, 0.8], 10.0),
    )


def _capture(mode: str = CAMERA_TO_HAND) -> core.PlatformCapture:
    calibrated_reference_from_camera = _transform([0.4, -0.1, 0.8], 10.0)
    base_from_tool = None
    robot_stamp_sec = None
    robot_stamp_nanosec = None
    if mode == CAMERA_ON_HAND:
        base_from_tool = _transform([0.3, 0.2, 0.6], -15.0)
        robot_stamp_sec = 122
        robot_stamp_nanosec = 900
        base_from_camera = base_from_tool @ calibrated_reference_from_camera
    else:
        base_from_camera = calibrated_reference_from_camera
    camera_link_from_optical = _transform([0.02, 0.0, 0.01], -90.0)
    optical_from_board = _transform([0.1, 0.2, 0.7], 20.0)
    base_from_platform = core.compose_platform_transform(
        base_from_camera,
        camera_link_from_optical,
        optical_from_board,
    )
    return core.PlatformCapture(
        captured_at_utc="2026-09-09T12:34:56.123456Z",
        color_stamp_sec=123,
        color_stamp_nanosec=456,
        frame_sequence=17,
        charuco_corner_count=12,
        calibration_mode=mode,
        calibration_reference_from_camera_link=calibrated_reference_from_camera,
        base_from_tool=base_from_tool,
        robot_tf_stamp_sec=robot_stamp_sec,
        robot_tf_stamp_nanosec=robot_stamp_nanosec,
        base_from_camera_link=base_from_camera,
        camera_link_from_optical=camera_link_from_optical,
        optical_from_board=optical_from_board,
        base_from_platform=base_from_platform,
    )


def test_platform_transform_uses_full_calibrated_camera_chain():
    base_from_camera = _transform([0.4, -0.1, 0.8], 10.0)
    camera_from_optical = _transform([0.02, 0.0, 0.01], -90.0)
    optical_from_board = _transform([0.1, 0.2, 0.7], 20.0)
    expected = base_from_camera @ camera_from_optical @ optical_from_board
    actual = core.compose_platform_transform(
        base_from_camera,
        camera_from_optical,
        optical_from_board,
    )
    assert np.allclose(actual, expected)
    assert np.allclose(actual[:3, :3].T @ actual[:3, :3], np.eye(3))


def test_camera_calibration_selection_accepts_both_strict_modes(monkeypatch, tmp_path):
    path = tmp_path / "calibration" / "camera_to_hand_calibration_test.yaml"
    path.parent.mkdir()
    path.write_text("artifact\n", encoding="utf-8")
    fake = SimpleNamespace(
        calibration_mode=CAMERA_TO_HAND,
        created_at_utc="2026-09-09T00:00:00.000000Z",
        settings=_settings(),
        reference_from_camera_link=_transform([0.1, 0.2, 0.3]),
    )
    monkeypatch.setattr(core, "load_calibration_yaml", lambda _path: fake)
    loaded = core.load_camera_calibration(path, root=tmp_path)
    assert loaded.path == path.resolve()
    assert loaded.settings == _settings()
    assert loaded.calibration_mode == CAMERA_TO_HAND
    assert loaded.reference_frame == "base_link"
    assert np.allclose(
        loaded.reference_from_camera_link,
        fake.reference_from_camera_link,
    )

    fake.calibration_mode = CAMERA_ON_HAND
    with pytest.raises(ValueError, match="filename conflicts"):
        core.load_camera_calibration(path, root=tmp_path)
    on_hand_path = tmp_path / "calibration" / "camera_on_hand_calibration_test.yaml"
    on_hand_path.write_text("artifact\n", encoding="utf-8")
    loaded = core.load_camera_calibration(on_hand_path, root=tmp_path)
    assert loaded.calibration_mode == CAMERA_ON_HAND
    assert loaded.reference_frame == "Link6"

    outside = tmp_path / "outside.yaml"
    outside.write_text("artifact\n", encoding="utf-8")
    with pytest.raises(ValueError, match="root calibration"):
        core.load_camera_calibration(outside, root=tmp_path)


def test_camera_mount_resolution_is_explicit_for_each_mode(tmp_path):
    fixed = _applied(tmp_path, CAMERA_TO_HAND)
    assert np.allclose(
        core.resolve_base_from_camera_link(fixed),
        fixed.reference_from_camera_link,
    )
    with pytest.raises(ValueError, match="must not use"):
        core.resolve_base_from_camera_link(fixed, np.eye(4))

    on_hand = _applied(tmp_path, CAMERA_ON_HAND)
    with pytest.raises(ValueError, match="requires live"):
        core.resolve_base_from_camera_link(on_hand)
    base_from_tool = _transform([0.2, 0.3, 0.4], -20.0)
    assert np.allclose(
        core.resolve_base_from_camera_link(on_hand, base_from_tool),
        base_from_tool @ on_hand.reference_from_camera_link,
    )


def test_strict_root_env_supplies_lan1_identity(tmp_path):
    content = (
        "ROS_LOCALHOST_ONLY=1\n"
        "DOBOT_ROBOT_LAN1_IP=192.168.20.204\n"
        "DOBOT_ROBOT_LAN2_IP=192.168.200.1\n"
    )
    (tmp_path / ".env.example").write_text(content, encoding="utf-8")
    (tmp_path / ".env").write_text(content, encoding="utf-8")
    assert core.load_robot_lan1_ip(tmp_path) == "192.168.20.204"

    (tmp_path / ".env").write_text(content + "UNSUPPORTED=1\n", encoding="utf-8")
    with pytest.raises(ValueError, match="exactly match"):
        core.load_robot_lan1_ip(tmp_path)


def test_ui_state_is_prefill_only_and_strict(tmp_path):
    path = tmp_path / "logs" / "item_perception_yolo" / "last_session.json"
    timestamp = datetime(2026, 9, 9, 10, 11, 12, 123456, tzinfo=timezone.utc)
    state = core.write_ui_state(
        path,
        "camera_on_hand_calibration_20260909T083036_133658Z.yaml",
        saved_at=timestamp,
    )
    assert state.saved_at_utc == "2026-09-09T10:11:12.123456Z"
    assert core.load_ui_state(path) == state

    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["auto_apply"] = True
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="unsupported or missing"):
        core.load_ui_state(path)


@pytest.mark.parametrize("mode", [CAMERA_TO_HAND, CAMERA_ON_HAND])
def test_platform_yaml_round_trip_contains_no_frame_archive(tmp_path, mode):
    applied = _applied(tmp_path, mode)
    capture = _capture(mode)
    created_at = datetime(2026, 9, 9, 12, 34, 56, 123456, tzinfo=timezone.utc)
    path = core.platform_output_path(
        "192.168.20.204",
        root=tmp_path,
        created_at=created_at,
    )
    assert path.name == (
        "platform_calibration_20260909T123456_123456Z_192.168.20.204.yaml"
    )
    core.write_platform_calibration(
        path,
        applied,
        "192.168.20.204",
        capture,
        root=tmp_path,
    )
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert payload["schema_version"] == 3
    assert payload["platform"]["reference"] == core.PLATFORM_REFERENCE_DEFINITION
    assert payload["artifact_type"] == "platform_calibration"
    assert payload["transform"]["target_frame"] == "base_link"
    assert payload["transform"]["source_frame"] == "platform_reference"
    assert payload["camera_calibration"]["schema_version"] == 7
    assert payload["camera_calibration"]["calibration_mode"] == mode
    assert payload["capture"]["robot_tf"]["required"] is (
        mode == CAMERA_ON_HAND
    )
    assert payload["charuco"]["dictionary"] == "DICT_5X5_50"
    artifact_text = path.read_text(encoding="utf-8").lower()
    assert "image_data" not in artifact_text
    assert "overlay" not in artifact_text

    loaded = core.load_platform_calibration(path, root=tmp_path)
    assert loaded.robot_lan1_ip == "192.168.20.204"
    assert loaded.platform_frame == "platform_reference"
    assert loaded.reference_convention == core.PLATFORM_REFERENCE_CONVENTION
    assert loaded.camera_calibration_mode == mode
    assert np.allclose(loaded.base_from_platform, capture.base_from_platform)
    with pytest.raises(ValueError, match="already exists"):
        core.write_platform_calibration(
            path,
            applied,
            "192.168.20.204",
            capture,
            root=tmp_path,
        )
    for schema in (1, 2):
        payload["schema_version"] = schema
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
        with pytest.raises(ValueError, match="schema_version must be exactly 3"):
            core.load_platform_calibration(path, root=tmp_path)
    payload["schema_version"] = 3
    payload["platform"]["reference"]["axes"] = "arbitrary_station_axes"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="reference convention is invalid"):
        core.load_platform_calibration(path, root=tmp_path)


def test_camera_on_hand_platform_capture_requires_robot_tf(tmp_path):
    applied = _applied(tmp_path, CAMERA_ON_HAND)
    capture = replace(
        _capture(CAMERA_ON_HAND),
        base_from_tool=None,
        robot_tf_stamp_sec=None,
        robot_tf_stamp_nanosec=None,
    )
    path = core.platform_output_path(
        "192.168.20.204",
        root=tmp_path,
        created_at=datetime(2026, 9, 9, 12, 34, 56, 123456, tzinfo=timezone.utc),
    )
    with pytest.raises(ValueError, match="requires robot TF"):
        core.write_platform_calibration(
            path,
            applied,
            "192.168.20.204",
            capture,
            root=tmp_path,
        )


def test_camera_calibration_change_blocks_save(tmp_path):
    applied = _applied(tmp_path)
    applied.path.write_text("changed\n", encoding="utf-8")
    path = core.platform_output_path(
        "192.168.20.204",
        root=tmp_path,
        created_at=datetime(2026, 9, 9, tzinfo=timezone.utc),
    )
    with pytest.raises(ValueError, match="changed after"):
        core.write_platform_calibration(
            path,
            applied,
            "192.168.20.204",
            _capture(CAMERA_TO_HAND),
            root=tmp_path,
        )


def test_package_event_log_overwrites_before_record_1001(tmp_path):
    logger = core.PackageEventLogger(tmp_path)
    logger.path.parent.mkdir(parents=True, exist_ok=True)
    logger.path.write_text(
        "".join(f'{{"existing":{index}}}\n' for index in range(1000)),
        encoding="utf-8",
    )
    logger.record("INFO", "bounded", "new session record")
    lines = logger.path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 1
    record = json.loads(lines[0])
    assert record["package"] == "item_perception_yolo"
    assert record["node"] == "platform_teach"
    assert record["event"] == "bounded"
