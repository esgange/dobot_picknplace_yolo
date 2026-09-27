import copy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from tray_perception import core


def settings():
    return {"name": "tray", "model_task": "segment", "geometry_source": "mask",
            "geometry": {"length_mm": 200., "width_mm": 100., "tolerance_mm": 2.},
            "yolo": {"class_ids": [0], "confidence": .25, "iou": .7,
                     "max_detections": 100, "image_size": 640}}


def camera_info():
    return {"width": 640, "height": 480,
            "k": [400., 0., 320., 0., 400., 240., 0., 0., 1.],
            "d": [0.] * 5, "distortion_model": "plumb_bob"}


def plane():
    transform = np.eye(4)
    transform[:3, 3] = [.1, .1, .2]
    return {"corners_base_m": [[.1, .1, .2], [.3, .1, .2], [.3, .2, .2], [.1, .2, .2]],
            "base_from_plane": transform.tolist(),
            "pixels": [[1., 1.], [2., 1.], [2., 2.], [1., 2.]],
            "max_error_mm": 0., "source_stamp_ns": 100, "depth_stamp_ns": 100,
            "camera": camera_info(), "depth_camera": camera_info()}


def position():
    return {"robot_lan1_ip": "192.0.2.1", "joint_names": [f"joint{i}" for i in range(1, 7)],
            "positions_rad": [.1] * 6, "recorded_at_utc": "2026-09-27T10:00:00.000000Z",
            "feedback_stamp": {"sec": 100, "nanosec": 0}, "source_topic": "/joint_states",
            "publisher_node": "/dobot_bringup_ros2"}


@pytest.fixture
def artifact(tmp_path):
    camera_path = tmp_path / "calibration/camera_to_hand_calibration_test.yaml"
    camera_path.parent.mkdir()
    camera_path.write_text("synthetic calibration hash fixture")
    camera = SimpleNamespace(path=camera_path, sha256=core.file_sha256(camera_path))
    model = tmp_path / "source.pt"
    model.write_bytes(b"hash-only synthetic model; never executed")
    path = core.save_profile(settings(), position(), plane(), camera, model,
                             core.file_sha256(model), tmp_path)
    return path, camera, model


def test_pair_roundtrip_is_independent_of_item_teach(tmp_path, artifact):
    path, camera, model = artifact
    profile = core.load_profile(path, tmp_path)
    assert profile["origin_convention"] == "nearest_base_corner_v1"
    assert profile["tray_teach_position"] == position()
    assert profile["reference_plane"] == plane()
    assert not any(k in profile for k in ("home", "item_teach", "placement_targets", "motion"))
    assert not (tmp_path / "offline_teach/item_teach").exists()
    assert path.with_suffix(".pt").read_bytes() == model.read_bytes()
    second = core.save_profile(settings(), position(), plane(), camera, model,
                               core.file_sha256(model), tmp_path)
    assert second != path and core.load_profile(second, tmp_path)


@pytest.mark.parametrize("change", ["model", "digest", "schema", "unknown", "plane", "duplicate"])
def test_corrupt_profile_refused(tmp_path, artifact, change):
    path, _, _ = artifact
    profile = core.load_profile(path, tmp_path)
    if change == "model":
        path.with_suffix(".pt").write_bytes(b"changed")
    elif change == "duplicate":
        path.write_text(path.read_text() + "schema_version: 1\n")
    else:
        if change == "digest":
            profile["model"]["sha256"] = "0" * 64
        elif change == "schema":
            profile["schema_version"] = 2
        elif change == "unknown":
            profile["home"] = position()
        else:
            profile["reference_plane"]["base_from_plane"][2][3] += .1
        path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError):
        core.load_profile(path, tmp_path)


def test_copy_position_does_not_retain_source_object(tmp_path, monkeypatch):
    home = position()
    monkeypatch.setattr(core, "load_item_profile", lambda *_a, **_k: ({"home": home}, "hash"))
    copied = core.copy_teach_position(Path("synthetic.yaml"), tmp_path)
    home["positions_rad"][0] = 10
    assert copied == position()


def test_ui_state_is_strict_and_atomic(tmp_path):
    assert core.read_session(tmp_path) is None
    state = {"schema_version": 1, "profile_filename": "", "camera_filename": "camera.yaml",
             "item_filename": "", "model_path": "/synthetic.pt", "settings": settings()}
    core.write_session(tmp_path, state)
    assert core.read_session(tmp_path) == state
    broken = copy.deepcopy(state)
    broken["settings"]["geometry"]["width_mm"] = float("nan")
    with pytest.raises(ValueError):
        core.write_session(tmp_path, broken)
    assert core.read_session(tmp_path) == state
    core.session_path(tmp_path).write_text('{"schema_version": 99}')
    with pytest.raises(ValueError):
        core.read_session(tmp_path)


def test_model_copy_never_overwrites_or_accepts_changed_sources(tmp_path, artifact, monkeypatch):
    path, camera, model = artifact
    profile = core.load_profile(path, tmp_path)
    monkeypatch.setattr(core, "utc_now", lambda: profile["created_at_utc"])
    before = path.read_bytes()
    with pytest.raises(FileExistsError):
        core.save_profile(settings(), position(), plane(), camera, model,
                          core.file_sha256(model), tmp_path)
    assert path.read_bytes() == before and core.load_profile(path, tmp_path)
    camera.path.write_text("changed")
    with pytest.raises(ValueError, match="changed"):
        core.save_profile(settings(), position(), plane(), camera, model,
                          core.file_sha256(model), tmp_path)


def test_event_log_caps_at_1000(tmp_path):
    logger = core.EventLogger(tmp_path)
    logger.path.parent.mkdir(parents=True)
    logger.path.write_text("{}\n" * 1000)
    logger.record("INFO", "synthetic", "test")
    assert len(logger.path.read_text().splitlines()) == 1
