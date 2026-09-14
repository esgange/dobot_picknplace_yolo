import hashlib
import json
import threading
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import MagicMock

from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
from rclpy.time import Time
from sensor_msgs.msg import Image
import yaml

from camera_calibration_gui.calibration_core import (
    CAMERA_ON_HAND,
    CAMERA_TO_HAND,
    CharucoSettings,
)
from item_perception_yolo import bin_teach_core as core
from item_perception_yolo import bin_teach_gui
from item_perception_yolo import platform_teach_core
from item_perception_yolo.ui_state import load_package_ui_state


def _transform(translation=(0.0, 0.0, 0.0), rotation_degrees=0.0) -> np.ndarray:
    radians = np.deg2rad(rotation_degrees)
    cosine = np.cos(radians)
    sine = np.sin(radians)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    matrix[:3, 3] = translation
    return matrix


def _applied(
    tmp_path: Path,
    mode: str = CAMERA_TO_HAND,
) -> core.AppliedBinTeachCalibration:
    env_content = (
        "ROS_LOCALHOST_ONLY=1\n"
        "DOBOT_ROBOT_LAN1_IP=192.168.20.204\n"
    )
    (tmp_path / ".env.example").write_text(env_content, encoding="utf-8")
    (tmp_path / ".env").write_text(env_content, encoding="utf-8")
    calibration = tmp_path / "calibration"
    calibration.mkdir(parents=True)
    mode_prefix = "camera_to_hand" if mode == CAMERA_TO_HAND else "camera_on_hand"
    camera_path = calibration / f"{mode_prefix}_calibration_test.yaml"
    camera_path.write_text("camera calibration\n", encoding="utf-8")
    camera_digest = hashlib.sha256(camera_path.read_bytes()).hexdigest()
    platform_path = (
        calibration
        / "platform_calibration_20260910T120000_000000Z_192.168.20.204.yaml"
    )
    platform_path.write_text("platform calibration\n", encoding="utf-8")
    camera = platform_teach_core.AppliedCameraCalibration(
        path=camera_path,
        sha256=camera_digest,
        created_at_utc="2026-09-09T08:30:36.133658Z",
        calibration_mode=mode,
        reference_frame="base_link" if mode == CAMERA_TO_HAND else "Link6",
        settings=CharucoSettings("bin_camera", "DICT_5X5_50", 5, 7, 30.0, 22.0),
        reference_from_camera_link=_transform((0.4, -0.1, 0.8), 10.0),
    )
    platform = platform_teach_core.PlatformCalibrationArtifact(
        path=platform_path,
        sha256=hashlib.sha256(platform_path.read_bytes()).hexdigest(),
        created_at_utc="2026-09-10T12:00:00.000000Z",
        robot_lan1_ip="192.168.20.204",
        platform_frame="platform_reference",
        reference_convention=platform_teach_core.PLATFORM_REFERENCE_CONVENTION,
        camera_calibration_filename=camera_path.name,
        camera_calibration_sha256=camera_digest,
        camera_calibration_mode=mode,
        camera_settings=camera.settings,
        calibration_reference_from_camera_link=(
            camera.reference_from_camera_link.copy()
        ),
        base_from_platform=_transform((0.2, 0.3, 0.1), 5.0),
    )
    return core.AppliedBinTeachCalibration(platform=platform, camera=camera)


def _capture(applied: core.AppliedBinTeachCalibration) -> core.BinTeachCapture:
    base_from_tool = None
    robot_tf_stamp_sec = None
    robot_tf_stamp_nanosec = None
    if applied.camera.calibration_mode == CAMERA_ON_HAND:
        base_from_tool = _transform((0.3, 0.2, 0.6), -15.0)
        robot_tf_stamp_sec = 122
        robot_tf_stamp_nanosec = 900
    base_from_camera = platform_teach_core.resolve_base_from_camera_link(
        applied.camera,
        base_from_tool,
    )
    camera_from_optical = _transform((0.01, 0.02, 0.03), 90.0)
    platform_from_optical = core.compose_platform_from_optical(
        applied.platform.base_from_platform,
        base_from_camera,
        camera_from_optical,
    )
    return core.BinTeachCapture(
        captured_at_utc="2026-09-10T12:34:56.123456Z",
        color_stamp_sec=123,
        color_stamp_nanosec=456,
        frame_sequence=17,
        calibration_mode=applied.camera.calibration_mode,
        calibration_reference_from_camera_link=(
            applied.camera.reference_from_camera_link.copy()
        ),
        base_from_tool=base_from_tool,
        robot_tf_stamp_sec=robot_tf_stamp_sec,
        robot_tf_stamp_nanosec=robot_tf_stamp_nanosec,
        base_from_camera_link=base_from_camera,
        camera_link_from_optical=camera_from_optical,
        platform_from_optical=platform_from_optical,
        points=(
            core.BinRoiPoint(-0.4, -0.3, 2, 2),
            core.BinRoiPoint(-0.4, 0.3, 0, 0),
            core.BinRoiPoint(0.4, 0.3, 3, 1),
            core.BinRoiPoint(0.4, -0.3, 1, 3),
        ),
    )


def _marker(center_x, center_y, half_size=0.05):
    return np.asarray(
        [
            [center_x - half_size, center_y + half_size, 0.1],
            [center_x + half_size, center_y + half_size, 0.1],
            [center_x + half_size, center_y - half_size, 0.1],
            [center_x - half_size, center_y - half_size, 0.1],
        ],
        dtype=np.float64,
    )


def test_outside_corner_selection_ignores_marker_id_order():
    corners = {
        0: _marker(1.0, 1.0),
        1: _marker(-1.0, -1.0),
        2: _marker(-1.0, 1.0),
        3: _marker(1.0, -1.0),
    }
    points = core.select_outside_roi_corners(corners)
    assert [(point.x_m, point.y_m) for point in points] == [
        (-1.05, -1.05),
        (-1.05, 1.05),
        (1.05, 1.05),
        (1.05, -1.05),
    ]
    assert [point.source_marker_id for point in points] == [1, 2, 0, 3]


def test_platform_camera_transform_chain_maps_all_marker_corners():
    base_from_platform = _transform((0.5, -0.2, 0.1), 15.0)
    base_from_camera = _transform((0.2, 0.3, 0.8), -5.0)
    camera_from_optical = _transform((0.01, 0.02, 0.03), 90.0)
    camera_from_optical[:3, :3] = camera_from_optical[:3, :3] @ np.diag([1., -1., -1.])
    platform_from_optical = core.compose_platform_from_optical(
        base_from_platform,
        base_from_camera,
        camera_from_optical,
    )
    corners = {marker_id: _marker(marker_id, marker_id + 1.0) for marker_id in range(4)}
    expected = (
        np.linalg.inv(base_from_platform)
        @ base_from_camera
        @ camera_from_optical
    )
    assert np.allclose(platform_from_optical, expected)
    rays = {}
    for marker_id, points in corners.items():
        points[:, 2] = 0.0
        optical = (np.linalg.inv(expected) @ np.column_stack((points, np.ones(4))).T).T[:, :3]
        rays[marker_id] = optical / optical[:, 2, None]
    transformed = core.project_marker_rays_to_plane(platform_from_optical, rays)
    assert np.allclose(transformed[2], corners[2])


def test_bin_tf_preview_matches_saved_xy_points_and_platform_transform(tmp_path):
    applied = _applied(tmp_path)
    capture = _capture(applied)
    stamp = TimeMessage(sec=123, nanosec=456)

    transforms = bin_teach_gui.build_bin_preview_transforms(
        applied.platform.base_from_platform,
        capture.points,
        stamp,
    )

    assert len(transforms) == 5
    platform = transforms[0]
    assert platform.header.frame_id == "base_link"
    assert platform.child_frame_id == "platform_reference"
    assert platform.header.stamp == stamp
    assert np.allclose(
        [
            platform.transform.translation.x,
            platform.transform.translation.y,
            platform.transform.translation.z,
        ],
        applied.platform.base_from_platform[:3, 3],
    )
    expected_quaternion = platform_teach_core.rotation_matrix_to_quaternion(
        applied.platform.base_from_platform[:3, :3]
    )
    assert np.allclose(
        [
            platform.transform.rotation.x,
            platform.transform.rotation.y,
            platform.transform.rotation.z,
            platform.transform.rotation.w,
        ],
        expected_quaternion,
    )
    for index, (message, point) in enumerate(
        zip(transforms[1:], capture.points),
        start=1,
    ):
        assert message.header.frame_id == "platform_reference"
        assert message.child_frame_id == f"bin_corner_{index}"
        assert message.header.stamp == stamp
        assert message.transform.translation.x == point.x_m
        assert message.transform.translation.y == point.y_m
        assert message.transform.translation.z == 0.0
        assert message.transform.rotation.x == 0.0
        assert message.transform.rotation.y == 0.0
        assert message.transform.rotation.z == 0.0
        assert message.transform.rotation.w == 1.0


def test_bin_context_requires_matching_platform_camera_hash_and_robot(monkeypatch, tmp_path):
    platform_path = tmp_path / "calibration" / (
        "platform_calibration_20260910T120000_000000Z_192.168.20.204.yaml"
    )
    camera_path = tmp_path / "calibration" / "camera_to_hand_calibration_test.yaml"
    camera = SimpleNamespace(
        path=camera_path,
        sha256="a" * 64,
        calibration_mode=CAMERA_TO_HAND,
        settings="same-settings",
        reference_from_camera_link=np.eye(4),
    )
    platform = SimpleNamespace(
        path=platform_path,
        robot_lan1_ip="192.168.20.204",
        camera_calibration_filename=camera_path.name,
        camera_calibration_sha256="a" * 64,
        camera_calibration_mode=CAMERA_TO_HAND,
        camera_settings="same-settings",
        calibration_reference_from_camera_link=np.eye(4),
    )
    monkeypatch.setattr(core, "load_platform_calibration", lambda *_args, **_kwargs: platform)
    monkeypatch.setattr(core, "load_robot_lan1_ip", lambda _root=None: "192.168.20.204")
    monkeypatch.setattr(
        core,
        "load_camera_calibration",
        lambda *_args, **_kwargs: camera,
    )
    loaded = core.load_bin_teach_calibration_context(platform_path, root=tmp_path)
    assert loaded.platform is platform and loaded.camera is camera

    camera.sha256 = "b" * 64
    with pytest.raises(ValueError, match="different SHA-256"):
        core.load_bin_teach_calibration_context(platform_path, root=tmp_path)
    camera.sha256 = "a" * 64
    platform.robot_lan1_ip = "192.168.20.205"
    with pytest.raises(ValueError, match="robot identity"):
        core.load_bin_teach_calibration_context(platform_path, root=tmp_path)


def test_shared_schema_three_ui_state_preserves_both_teach_forms(tmp_path):
    path = tmp_path / "logs" / "item_perception_yolo" / "last_session.json"
    platform_teach_core.write_ui_state(
        path,
        "camera_on_hand_calibration_20260909T083036_133658Z.yaml",
        saved_at=datetime(2026, 9, 10, 10, 0, tzinfo=timezone.utc),
    )
    core.write_bin_state(
        path,
        "platform_calibration_20260910T120000_000000Z_192.168.20.204.yaml",
        "DICT_5X5_50",
        40.0,
        saved_at=datetime(2026, 9, 10, 10, 1, tzinfo=timezone.utc),
    )
    state = load_package_ui_state(path)
    assert state.platform_camera_calibration_filename.startswith("camera_on_hand_")
    assert state.bin_platform_calibration_filename.startswith("platform_calibration_")
    assert state.bin_aruco_dictionary == "DICT_5X5_50"
    assert state.bin_marker_size_mm == 40.0
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["schema_version"] == 6
    payload["schema_version"] = 4
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="schema_version must be exactly 6"):
        load_package_ui_state(path)
    payload["schema_version"] = 6
    payload["bin_teach"]["unknown"] = True
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="fields are invalid"):
        load_package_ui_state(path)


@pytest.mark.parametrize("mode", [CAMERA_TO_HAND, CAMERA_ON_HAND])
def test_bin_yaml_round_trip_has_exact_four_xy_points_and_no_frames(
    monkeypatch,
    tmp_path,
    mode,
):
    applied = _applied(tmp_path, mode)
    capture = _capture(applied)
    path = core.bin_output_path(
        "192.168.20.204",
        root=tmp_path,
        created_at=datetime(
            2026,
            9,
            10,
            12,
            34,
            56,
            123456,
            tzinfo=timezone.utc,
        ),
    )
    settings = core.BinArucoSettings("DICT_5X5_50", 40.0)
    core.write_bin_teach(path, applied, settings, capture, root=tmp_path)
    assert path.parent == tmp_path / "offline_teach" / "bin_teach"
    old_path = tmp_path / "calibration" / path.name
    old_path.write_bytes(path.read_bytes())
    with pytest.raises(ValueError, match="root offline_teach/bin_teach/"):
        core.load_bin_teach(old_path, root=tmp_path)
    with pytest.raises(ValueError, match="root offline_teach/bin_teach/"):
        core.write_bin_teach(old_path, applied, settings, capture, root=tmp_path)
    monkeypatch.setattr(
        core,
        "load_platform_calibration",
        lambda *_args, **_kwargs: applied.platform,
    )
    monkeypatch.setattr(
        core,
        "load_camera_calibration",
        lambda *_args, **_kwargs: applied.camera,
    )
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert payload["schema_version"] == 3
    assert payload["artifact_type"] == "bin_teach"
    assert payload["roi"]["coordinate_frame"] == "platform_reference"
    assert len(payload["roi"]["points"]) == 4
    assert payload["reference"] == platform_teach_core.PLATFORM_REFERENCE_DEFINITION
    provenance = payload["teaching_provenance"]
    assert provenance["camera_calibration"]["calibration_mode"] == mode
    assert provenance["capture"]["robot_tf"]["required"] is (
        mode == CAMERA_ON_HAND
    )
    assert all("x" in point and "y" in point for point in payload["roi"]["points"])
    text = path.read_text(encoding="utf-8").lower()
    assert "image_data" not in text
    assert "overlay" not in text
    assert "marker_pose" not in text
    loaded = core.load_bin_teach(path, root=tmp_path)
    assert loaded.dictionary_name == "DICT_5X5_50"
    assert loaded.marker_size_mm == 40.0
    assert loaded.source_camera_calibration_mode == mode
    assert loaded.source_platform_calibration_filename == applied.platform.path.name
    assert loaded.source_platform_calibration_sha256 == applied.platform.sha256
    assert core.bin_platform_warning(loaded, applied.platform) is None
    assert loaded.points == capture.points
    with pytest.raises(ValueError, match="already exists"):
        core.write_bin_teach(path, applied, settings, capture, root=tmp_path)
    payload["schema_version"] = 1
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="schema_version must be 3"):
        core.load_bin_teach(path, root=tmp_path)


def test_camera_on_hand_bin_capture_requires_robot_tf(tmp_path):
    applied = _applied(tmp_path, CAMERA_ON_HAND)
    capture = replace(
        _capture(applied),
        base_from_tool=None,
        robot_tf_stamp_sec=None,
        robot_tf_stamp_nanosec=None,
    )
    path = core.bin_output_path(
        "192.168.20.204",
        root=tmp_path,
        created_at=datetime(2026, 9, 10, 12, 34, 56, 123456, tzinfo=timezone.utc),
    )
    with pytest.raises(ValueError, match="requires robot TF"):
        core.write_bin_teach(
            path,
            applied,
            core.BinArucoSettings("DICT_5X5_50", 40.0),
            capture,
            root=tmp_path,
        )


def test_only_5x5_dictionary_and_positive_measured_size_are_accepted():
    core.BinArucoSettings("DICT_5X5_100", 25.0).validate()
    with pytest.raises(ValueError, match="5x5"):
        core.BinArucoSettings("DICT_4X4_50", 25.0).validate()
    with pytest.raises(ValueError, match="greater than zero"):
        core.BinArucoSettings("DICT_5X5_50", 0.0).validate()


def test_source_artifact_change_blocks_bin_save(tmp_path):
    applied = _applied(tmp_path)
    applied.platform.path.write_text("changed platform\n", encoding="utf-8")
    path = core.bin_output_path(
        "192.168.20.204",
        root=tmp_path,
        created_at=datetime(2026, 9, 10, 12, 34, 56, 123456, tzinfo=timezone.utc),
    )
    with pytest.raises(ValueError, match="platform calibration changed"):
        core.write_bin_teach(
            path,
            applied,
            core.BinArucoSettings("DICT_5X5_50", 40.0),
            _capture(applied),
            root=tmp_path,
        )


def test_bin_events_use_shared_bounded_package_log(tmp_path):
    logger = platform_teach_core.PackageEventLogger(tmp_path, node_name="bin_teach")
    logger.path.write_text(
        "".join(f'{{"existing":{index}}}\n' for index in range(1000)),
        encoding="utf-8",
    )
    logger.record("INFO", "bin_test", "bounded bin event")
    lines = logger.path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 1
    record = json.loads(lines[0])
    assert record["node"] == "bin_teach"
    assert record["event"] == "bin_test"


def _write_template(root, mode):
    root.mkdir(parents=True, exist_ok=True)
    applied = _applied(root, mode)
    path = core.bin_output_path(
        applied.platform.robot_lan1_ip,
        root=root,
        created_at=datetime(2026, 9, 10, 12, 34, 56, 123456, tzinfo=timezone.utc),
    )
    core.write_bin_teach(
        path, applied, core.BinArucoSettings("DICT_5X5_50", 40.0),
        _capture(applied), root=root,
    )
    return path


@pytest.mark.parametrize("same_filename", [True, False])
def test_bin_platform_warning_compares_hashes_without_binding_roi(tmp_path, same_filename):
    source = _write_template(tmp_path / "source", CAMERA_TO_HAND)
    template = core.load_bin_teach(source, root=tmp_path / "source")
    destination_root = tmp_path / "destination"
    destination_root.mkdir()
    destination = _applied(destination_root, CAMERA_ON_HAND).platform
    if not same_filename:
        destination = replace(destination, path=destination.path.with_name(
            "platform_calibration_different_station.yaml"))
    # A same-content copy is not a mismatch, regardless of path or name.
    assert core.bin_platform_warning(template, destination) is None
    changed = replace(destination, sha256="b" * 64,
                      base_from_platform=_transform((.4, -.2, .9), 15.0))
    before = core.place_bin_roi(template, changed)
    warning = core.bin_platform_warning(template, changed)
    assert "WARNING: Bin/platform mismatch" in warning
    assert template.source_platform_calibration_filename in warning
    assert changed.path.name in warning
    assert "SHA-256 checksums differ" in warning
    assert "Portable reuse is allowed" in warning
    assert np.array_equal(core.place_bin_roi(template, changed), before)


@pytest.mark.parametrize("mode", [CAMERA_TO_HAND, CAMERA_ON_HAND])
def test_portable_bin_load_uses_no_source_files_or_station_env(tmp_path, monkeypatch, mode):
    source = _write_template(tmp_path / "source", mode)
    destination = tmp_path / "destination"
    destination_bin_dir = destination / "offline_teach" / "bin_teach"
    destination_bin_dir.mkdir(parents=True)
    copied = destination_bin_dir / source.name
    copied.write_bytes(source.read_bytes())

    def forbidden(*args, **kwargs):
        pytest.fail("Portable bin reader must not load a station or a source artifact")

    monkeypatch.setattr(core, "load_platform_calibration", forbidden)
    monkeypatch.setattr(core, "load_camera_calibration", forbidden)
    monkeypatch.setattr(core, "load_robot_lan1_ip", forbidden)
    template = core.load_bin_teach(copied, root=destination)
    assert template.source_robot_lan1_ip == "192.168.20.204"
    assert template.sha256 == hashlib.sha256(copied.read_bytes()).hexdigest()
    assert template.reference_convention == platform_teach_core.PLATFORM_REFERENCE_CONVENTION
    # The warning uses saved provenance only; it never opens the original platform.
    warning = core.bin_platform_warning(template, SimpleNamespace(
        path=Path("platform_calibration_new_station.yaml"), sha256="c" * 64))
    assert "Portable reuse is allowed" in warning
    assert len(list(destination_bin_dir.iterdir())) == 1
    assert not (destination / "calibration").exists()
    assert not (destination / ".env").exists()


@pytest.mark.parametrize("mode", [CAMERA_TO_HAND, CAMERA_ON_HAND])
def test_same_xy_template_uses_destination_platform_height_and_rotation(tmp_path, mode):
    source = _write_template(tmp_path / "source", mode)
    template = core.load_bin_teach(source, root=tmp_path / "source")
    destination_root = tmp_path / "destination"
    destination_root.mkdir()
    other_mode = CAMERA_ON_HAND if mode == CAMERA_TO_HAND else CAMERA_TO_HAND
    destination = replace(
        _applied(destination_root, other_mode).platform,
        robot_lan1_ip="192.168.20.205",
        base_from_platform=_transform((0.5, -0.4, 0.9), 90.0),
    )
    before = template.points
    placed = core.place_bin_roi(template, destination)
    assert np.allclose(placed, [
        [0.5 - point.y_m, -0.4 + point.x_m, 0.9] for point in template.points
    ])
    lower = replace(destination, base_from_platform=_transform((0.5, -0.4, 0.7), 90.0))
    assert np.allclose(placed - core.place_bin_roi(template, lower), [0.0, 0.0, 0.2])
    assert template.points == before
    with pytest.raises(ValueError, match="reference conventions differ"):
        core.place_bin_roi(template, replace(destination, reference_convention="other_origin"))


def test_portable_template_rejects_legacy_schema_and_invalid_provenance(tmp_path):
    path = _write_template(tmp_path, CAMERA_TO_HAND)
    valid = yaml.safe_load(path.read_text())
    for schema in (1, 2):
        payload = dict(valid, schema_version=schema)
        path.write_text(yaml.safe_dump(payload))
        with pytest.raises(ValueError, match="schema_version must be 3"):
            core.load_bin_teach(path, root=tmp_path)
    payload = dict(valid, reference={"convention": "unknown"})
    path.write_text(yaml.safe_dump(payload))
    with pytest.raises(ValueError, match="reference convention is invalid"):
        core.load_bin_teach(path, root=tmp_path)
    valid["teaching_provenance"]["capture"]["base_from_platform"]["translation_m"]["z"] += 1.0
    path.write_text(yaml.safe_dump(valid))
    with pytest.raises(ValueError, match="optical transform conflicts"):
        core.load_bin_teach(path, root=tmp_path)


def test_loaded_preview_is_explicit_not_a_capture_and_retake_stops_tf(tmp_path, monkeypatch):
    path = _write_template(tmp_path / "source", CAMERA_TO_HAND)
    template = core.load_bin_teach(path, root=tmp_path / "source")
    destination_root = tmp_path / "destination"
    destination_root.mkdir()
    applied = _applied(destination_root, CAMERA_ON_HAND)
    records = []
    messages = []
    node = SimpleNamespace(
        _lock=threading.RLock(), _fatal_error=None,
        _applied=applied, _configuration_generation=1,
        _aruco_settings=core.BinArucoSettings("DICT_5X5_50", 40.0),
        _event_logger=SimpleNamespace(record=lambda *args, **kwargs: records.append(args)),
        _tf_broadcaster=SimpleNamespace(sendTransform=messages.append),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: TimeMessage(sec=900, nanosec=0))
        ),
    )
    bin_teach_gui.BinTeachNode._clear_runtime_state(node)
    monkeypatch.setattr(bin_teach_gui, "load_bin_teach", lambda _: template)
    monkeypatch.setattr(bin_teach_gui, "validate_applied_sources", lambda _: None)

    bin_teach_gui.BinTeachNode._rebroadcast_bin_preview(node)
    assert not messages
    bin_teach_gui.BinTeachNode.load_bin_roi(node, path)
    assert node._capture is None
    assert node._loaded_bin is template
    assert bin_teach_gui.BinTeachNode.save(node)[0] is False
    assert bin_teach_gui.BinTeachNode.capture_bin_roi(node)[0] is False
    bin_teach_gui.BinTeachNode._rebroadcast_bin_preview(node)
    assert len(messages) == 1
    assert len(messages[0]) == 5
    assert messages[0][0].transform.translation.z == applied.platform.base_from_platform[2, 3]
    bin_teach_gui.BinTeachNode.retake(node)
    bin_teach_gui.BinTeachNode._rebroadcast_bin_preview(node)
    assert len(messages) == 1
    assert node._loaded_bin is None
    assert any(record[1] == "bin_roi_loaded" for record in records)

    node._loaded_bin = template
    bin_teach_gui.BinTeachNode._clear_runtime_state(node)
    bin_teach_gui.BinTeachNode._rebroadcast_bin_preview(node)
    assert len(messages) == 1
    node._applied = None
    with pytest.raises(ValueError, match="Apply this station"):
        bin_teach_gui.BinTeachNode.load_bin_roi(node, path)
    node._applied = applied
    node._loaded_bin = template
    node.get_logger = lambda: SimpleNamespace(fatal=lambda message: records.append(message))
    bin_teach_gui.BinTeachNode._hard_fail_worker(node, RuntimeError("synthetic worker failure"))
    bin_teach_gui.BinTeachNode._rebroadcast_bin_preview(node)
    assert len(messages) == 1
    assert node._loaded_bin is None
    with pytest.raises(ValueError, match="synthetic worker failure"):
        bin_teach_gui.BinTeachNode.load_bin_roi(node, path)


def _loaded_rgb_node(tmp_path, mode):
    applied = _applied(tmp_path, mode)
    applied = replace(
        applied,
        camera=replace(applied.camera, reference_from_camera_link=np.eye(4)),
        platform=replace(applied.platform, base_from_platform=_transform((0, 0, 1))),
    )
    node = SimpleNamespace(
        _lock=threading.RLock(), _configuration_generation=1, _fatal_error=None,
        _applied=applied, _aruco_settings=core.BinArucoSettings("DICT_5X5_50", 40.0),
        _event_logger=MagicMock(), _opencv_worker=MagicMock(),
        get_clock=lambda: SimpleNamespace(now=lambda: Time.from_msg(TimeMessage(sec=1000))),
    )
    bin_teach_gui.BinTeachNode._clear_runtime_state(node)
    node._loaded_bin = SimpleNamespace(
        path=Path("bin_teach_test.yaml"), points=_capture(applied).points,
    )
    node._camera_matrix = np.array([[300., 0., 320.], [0., 300., 240.], [0., 0., 1.]])
    node._distortion = np.zeros(8)
    node._camera_distortion_model = "plumb_bob"
    node._camera_info_size = (480, 640)
    node._tf_buffer = MagicMock()
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    node._tf_buffer.lookup_transform.return_value = transform
    node._lookup_fresh_base_from_tool = lambda: (np.eye(4), 1000, 0, 0.0)
    node._opencv_worker.project_points.side_effect = lambda **kw: (
        kw["optical_points"][:, :2] / kw["optical_points"][:, 2, None]
        * 300.0 + [320., 240.]
    )
    for name in ("_skip_sensor_frame", "_render_loaded_border", "_set_detection_status"):
        setattr(node, name, MethodType(getattr(bin_teach_gui.BinTeachNode, name), node))
    message = Image()
    message.header.frame_id = applied.camera.settings.optical_frame
    message.header.stamp = TimeMessage(sec=1000, nanosec=0)
    message.encoding = "rgb8"
    message.width, message.height, message.step = 640, 480, 640 * 3
    message.data = bytes([20]) * (480 * 640 * 3)
    return node, message


@pytest.mark.parametrize("mode", [CAMERA_TO_HAND, CAMERA_ON_HAND])
def test_loaded_border_renders_without_markers_and_hides_after_stream_stalls(tmp_path, mode):
    node, message = _loaded_rgb_node(tmp_path, mode)
    bin_teach_gui.BinTeachNode._on_color_image(node, message)
    node._opencv_worker.detect_aruco.assert_not_called()
    node._opencv_worker.project_points.assert_called_once()
    assert node._capture is None
    snapshot = bin_teach_gui.BinTeachNode.status_snapshot(node)
    pixels = snapshot["loaded_border_pixels"]
    assert pixels.shape == (4 * core.ROI_BORDER_SAMPLES_PER_EDGE, 2)
    assert np.allclose(pixels[::core.ROI_BORDER_SAMPLES_PER_EDGE], [
        [200, 150], [200, 330], [440, 330], [440, 150],
    ])
    assert np.all(snapshot["overlay"] == 20)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time.from_msg(TimeMessage(sec=1001)))
    snapshot = bin_teach_gui.BinTeachNode.status_snapshot(node)
    assert snapshot["loaded_border_pixels"] is None
    assert "stale" in snapshot["loaded_border_status"]
    assert node._loaded_bin is not None


@pytest.mark.parametrize("invalid", ["missing_info", "stale_rgb", "stale_tf", "behind", "model"])
def test_loaded_border_blocks_invalid_projection_inputs_without_losing_template(tmp_path, invalid):
    node, message = _loaded_rgb_node(tmp_path, CAMERA_ON_HAND)
    if invalid == "missing_info":
        node._camera_matrix = None
    elif invalid == "stale_rgb":
        message.header.stamp.sec = 998
    elif invalid == "stale_tf":
        node._lookup_fresh_base_from_tool = MagicMock(side_effect=ValueError("robot TF stale"))
    elif invalid == "behind":
        node._applied = replace(
            node._applied,
            platform=replace(node._applied.platform, base_from_platform=_transform((0, 0, -1))),
        )
    else:
        node._camera_distortion_model = "equidistant"
    bin_teach_gui.BinTeachNode._on_color_image(node, message)
    node._opencv_worker.project_points.assert_not_called()
    node._opencv_worker.detect_aruco.assert_not_called()
    assert node._loaded_border_pixels is None
    assert node._loaded_bin is not None
    assert node._fatal_error is None


@pytest.mark.parametrize("mode", [CAMERA_TO_HAND, CAMERA_ON_HAND])
def test_loaded_border_follows_current_station_and_camera_transforms(tmp_path, mode):
    node, message = _loaded_rgb_node(tmp_path, mode)
    mounting = _transform((0.1, 0.05, 0.02), 20.0)
    platform = _transform((0.25, -0.1, 1.2), 45.0)
    robot = _transform((0.1, -0.05, 0.2), -15.0)
    node._applied = replace(
        node._applied,
        camera=replace(node._applied.camera, reference_from_camera_link=mounting),
        platform=replace(node._applied.platform, base_from_platform=platform),
    )
    node._lookup_fresh_base_from_tool = MagicMock(return_value=(robot, 1000, 0, 0.0))
    internal = node._tf_buffer.lookup_transform.return_value.transform
    internal.translation.x, internal.translation.y, internal.translation.z = 0.03, 0.01, 0.04
    internal.rotation.z = np.sin(np.deg2rad(5.0))
    internal.rotation.w = np.cos(np.deg2rad(5.0))
    bin_teach_gui.BinTeachNode._on_color_image(node, message)
    base_camera = mounting if mode == CAMERA_TO_HAND else robot @ mounting
    optical_platform = np.linalg.inv(
        base_camera @ _transform((0.03, 0.01, 0.04), 10.0)
    ) @ platform
    points = np.array([[p.x_m, p.y_m, 0, 1] for p in node._loaded_bin.points])
    expected = (optical_platform @ points.T).T
    expected = expected[:, :2] / expected[:, 2, None] * 300 + [320, 240]
    assert np.allclose(node._loaded_border_pixels[::core.ROI_BORDER_SAMPLES_PER_EDGE], expected)
    if mode == CAMERA_TO_HAND:
        node._lookup_fresh_base_from_tool.assert_not_called()
    else:
        node._lookup_fresh_base_from_tool.assert_called_once()


def test_retake_during_projection_discards_inflight_border(tmp_path):
    node, message = _loaded_rgb_node(tmp_path, CAMERA_TO_HAND)

    def interrupted_projection(**kwargs):
        bin_teach_gui.BinTeachNode.retake(node)
        return np.zeros((core.ROI_BORDER_SAMPLES_PER_EDGE * 4, 2))

    node._opencv_worker.project_points.side_effect = interrupted_projection
    bin_teach_gui.BinTeachNode._on_color_image(node, message)
    assert node._loaded_border_pixels is None
    assert node._loaded_bin is None
    assert node._latest_overlay is None


@pytest.mark.parametrize("result", [None, np.zeros((4, 3)), np.full((4, 2), np.nan)])
def test_malformed_worker_projection_is_terminal(result):
    from camera_calibration_gui.opencv_worker import OpenCvWorkerClient, OpenCvWorkerFailure

    client = SimpleNamespace(
        _request=MagicMock(return_value=result),
        _failure=MagicMock(return_value=OpenCvWorkerFailure(
            "malformed projected point result", operation="project_points",
            worker_pid=123, exit_code=None,
        )),
    )
    with pytest.raises(OpenCvWorkerFailure, match="malformed projected point result"):
        OpenCvWorkerClient.project_points(
            client, optical_points=np.ones((4, 3)), camera_matrix=np.eye(3),
            distortion=np.zeros(8),
        )
    client._request.assert_called_once()
    client._failure.assert_called_once_with("project_points", "malformed projected point result")


def test_green_loaded_border_and_label_are_drawn_without_filling_roi(tmp_path, monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    app = (
        bin_teach_gui.QtWidgets.QApplication.instance()
        or bin_teach_gui.QtWidgets.QApplication([])
    )
    node, message = _loaded_rgb_node(tmp_path, CAMERA_TO_HAND)
    bin_teach_gui.BinTeachNode._on_color_image(node, message)
    snapshot = bin_teach_gui.BinTeachNode.status_snapshot(node)
    image = bin_teach_gui.QtGui.QImage(640, 480, bin_teach_gui.QtGui.QImage.Format_RGB888)
    image.fill(bin_teach_gui.QtGui.QColor(20, 20, 20))
    bin_teach_gui.BinTeachWindow._paint_roi_overlay(None, image, snapshot)
    green = bin_teach_gui.QtGui.QColor(0, 255, 0)
    assert image.pixelColor(200, 240) == green
    assert image.pixelColor(320, 240) == bin_teach_gui.QtGui.QColor(20, 20, 20)
    assert any(image.pixelColor(x, y) == green for x in range(12, 300) for y in range(12, 45))
    assert app is not None


def test_private_worker_projects_loaded_geometry_with_lens_distortion():
    from camera_calibration_gui.opencv_worker import OpenCvWorkerClient

    root = Path(__file__).resolve().parents[3]
    runtime = root / "install/camera_calibration/lib/camera_calibration/opencv_runtime"
    worker = OpenCvWorkerClient(runtime_dir=runtime)
    try:
        points = np.array([[1., 1., 2.], [-1., 0., 2.]])
        matrix = np.array([[400., 0., 320.], [0., 500., 240.], [0., 0., 1.]])
        distortion = np.array([.1, -.02, .003, -.004, .001, .005, -.001, .0001])
        actual = worker.project_points(
            optical_points=points, camera_matrix=matrix, distortion=distortion,
        )
        expected = []
        for x, y, z in points:
            x, y = x / z, y / z
            r2 = x*x + y*y
            radial = (1 + .1*r2 - .02*r2**2 + .001*r2**3) / (
                1 + .005*r2 - .001*r2**2 + .0001*r2**3
            )
            dx = 2*.003*x*y - .004*(r2 + 2*x*x)
            dy = .003*(r2 + 2*y*y) - 2*.004*x*y
            expected.append([400*(x*radial + dx) + 320, 500*(y*radial + dy) + 240])
        assert np.allclose(actual, expected, atol=1e-9)
    finally:
        worker.close()
