"""Exercise real file validation once, with immutable metadata guards on capture."""
import os
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import MagicMock

import pytest

from item_perception_yolo import item_detector as detector, item_teach_core as core
from item_perception_yolo import item_teach_calibration as selection
from item_perception_yolo import platform_teach_core as platform
from test_item_detector import service_node, call  # noqa: F401
from test_item_teach import pair, home, settings  # noqa: F401
from test_item_teach_calibration import files  # noqa: F401


@pytest.fixture
def bound_node(pair, service_node, monkeypatch):
    root, source, path, profile = pair
    node, _ = service_node
    node.root, node.profile_path = root, path
    node.profile_digest = core.file_sha256(path)
    node.model_config = {"path": str(source), "sha256": profile["model"]["sha256"]}
    node._bound_profile = MethodType(detector.ItemDetectNode._bound_profile, node)
    node._check_profile = MethodType(detector.ItemDetectNode._check_profile, node)
    monkeypatch.setattr(detector, "load_item_profile",
                        lambda path: core.load_item_profile(path, root=root))
    monkeypatch.setattr(detector, "file_sha256", core.file_sha256)
    node._bound_profile(path)
    return node, source, path


def test_repeated_capture_and_frozen_view_never_read_bound_files(bound_node, monkeypatch):
    node, _, path = bound_node
    # Block content reads and parsing, while allowing lstat-based identity fences.
    fail = MagicMock(side_effect=AssertionError("Bound file was reread"))
    monkeypatch.setattr(Path, "open", fail)
    monkeypatch.setattr(detector, "load_item_profile", fail)
    monkeypatch.setattr(detector, "file_sha256", fail)
    for _ in range(3):
        response = call(node, digest=node.profile_digest)
        assert response.success, response.message
        detector.ItemDetectNode.validate_simulation_view(node, {
            "simulation_epoch": node.arm_epoch,
            "simulation_profile": (str(path), node.profile_digest)})
    fail.assert_not_called()


@pytest.mark.parametrize("source_index", [0, 1, 2])
@pytest.mark.parametrize("change", ["replace", "same_size_mtime", "delete", "symlink"])
def test_bound_item_change_blocks_without_reloading(bound_node, monkeypatch, source_index, change):
    node, loaded, profile = bound_node
    path = (profile, profile.with_suffix(".pt"), loaded)[source_index]
    original, info = path.read_bytes(), path.stat()
    if change == "replace":
        other = path.with_name(path.name + ".new")
        other.write_bytes(original)
        os.replace(other, path)
    elif change == "same_size_mtime":
        path.write_bytes(bytes([original[0] ^ 1]) + original[1:])
        os.utime(path, ns=(info.st_atime_ns, info.st_mtime_ns))
    elif change == "delete":
        path.unlink()
    else:
        other = path.with_name(path.name + ".original")
        path.rename(other)
        path.symlink_to(other)
    loader = MagicMock(side_effect=AssertionError("Must not reload changed source"))
    monkeypatch.setattr(detector, "load_item_profile", loader)
    for _ in range(2):
        response = call(node, digest=node.profile_digest)
        assert not response.success and not response.candidates
    node.infer.assert_not_called()
    loader.assert_not_called()


@pytest.mark.parametrize("phase", ["inference", "logging", "debug"])
def test_changed_source_during_capture_revokes_result(bound_node, monkeypatch, phase):
    node, _, profile = bound_node
    def edit(*a, **kw):
        profile.write_bytes(profile.read_bytes() + b"\n# changed\n")
    if phase == "inference":
        view = node.infer.return_value
        def infer(*a, **kw):
            edit()
            return view
        node.infer.side_effect = infer
    elif phase == "logging":
        node.events.record.side_effect = edit
    else:
        monkeypatch.setattr(detector, "save_pick_debug_pair", lambda *a: (edit() or {}))
    request = detector.GetItemPoses.Request(
        pose_convention=detector.GetItemPoses.Request.POSE_CONVENTION,
        repick_contract=detector.REPICK_CONTRACT, max_candidates=3,
        profile_sha256=node.profile_digest, save_debug_images=phase == "debug")
    response = detector.ItemDetectNode._request(node, request, detector.GetItemPoses.Response())
    assert not response.success and not response.candidates


def test_profile_mutation_does_not_corrupt_cached_values(bound_node):
    node, _, path = bound_node
    first, digest = node._bound_profile(path)
    first["retry"]["pose_candidates"] = 999
    assert node._bound_profile(path)[0]["retry"]["pose_candidates"] == 3
    path.write_bytes(path.read_bytes() + b"\n")
    with pytest.raises(ValueError, match="source changed"):
        node._bound_profile(path)
    node._profile_binding = None  # Explicit arm/load is allowed to validate a new file.
    assert node._bound_profile(path)[1] != digest


def test_profile_validation_race_cannot_create_cached_binding(bound_node, monkeypatch):
    node, _, path = bound_node
    node._profile_binding = None
    original = detector.load_item_profile
    def changing(path):
        result = original(path)
        path.write_bytes(path.read_bytes() + b"\n")
        return result
    monkeypatch.setattr(detector, "load_item_profile", changing)
    with pytest.raises(ValueError, match="source changed"):
        node._bound_profile(path)
    assert node._profile_binding is None


def test_legacy_camera_catalog_change_requires_reapplication(files):
    root, paths = files
    # Automatic catalogs require timestamp names; this fixture otherwise exercises
    # explicit selection, which intentionally accepts a custom camera basename.
    canonical = paths[1].with_name("camera_to_hand_calibration_20260909T083036_133658Z.yaml")
    paths[1].rename(canonical)
    applied, camera = selection.load_calibration_selection(paths[0], canonical, paths[2], root=root)
    bin_path = root / "bin.yaml"
    bin_path.write_text("already validated bin")
    node = SimpleNamespace(root=root, applied=applied, robot_camera=camera,
        bin_artifact=SimpleNamespace(path=bin_path, sha256=core.file_sha256(bin_path)),
        explicit_robot_camera=False, service=None)
    detector.ItemDetectNode._validate_sources(node)
    detector.ItemDetectNode._validate_sources(node)
    (root / "calibration/camera_on_hand_calibration_20261019T120000_000000Z.yaml").write_text("new")
    with pytest.raises(ValueError, match="catalog changed"):
        detector.ItemDetectNode._validate_sources(node)


@pytest.mark.parametrize("source_index", range(6))
def test_station_cache_keeps_provenance_checks_without_repeated_reads(files, monkeypatch, source_index):
    root, paths = files
    applied, camera = selection.load_calibration_selection(*paths, root=root)
    bin_path = root / "bin.yaml"
    bin_path.write_text("already validated bin")
    node = SimpleNamespace(root=root, applied=applied, robot_camera=camera,
        bin_artifact=SimpleNamespace(path=bin_path, sha256=core.file_sha256(bin_path)),
        explicit_robot_camera=True, service=object(), disarm=MagicMock(), last_view=object())
    detector.ItemDetectNode._validate_sources(node)
    path = node._source_binding["paths"][source_index]
    original = path.read_bytes()
    fail = MagicMock(side_effect=AssertionError("Must not reload unchanged calibration/config"))
    monkeypatch.setattr(detector, "validate_selected_robot_camera", fail)
    monkeypatch.setattr(detector, "validate_applied_sources", fail)
    monkeypatch.setattr(detector, "file_sha256", fail)
    for _ in range(3):
        detector.ItemDetectNode._validate_sources(node)
    fail.assert_not_called()
    path.write_bytes(original + b"\n")
    with pytest.raises(ValueError, match="source changed"):
        detector.ItemDetectNode._validate_sources(node)
    assert node.last_view is None
    node.disarm.assert_called_once()


def test_source_changed_during_initial_validation_never_binds(files, monkeypatch):
    root, paths = files
    applied, camera = selection.load_calibration_selection(*paths, root=root)
    bin_path = root / "bin.yaml"
    bin_path.write_text("already validated bin")
    node = SimpleNamespace(root=root, applied=applied, robot_camera=camera,
        bin_artifact=SimpleNamespace(path=bin_path, sha256=core.file_sha256(bin_path)),
        explicit_robot_camera=True, service=None)
    original = detector.validate_selected_robot_camera
    def changing(*a, **kw):
        value = original(*a, **kw)
        paths[1].write_text(paths[1].read_text() + "\n")
        return value
    monkeypatch.setattr(detector, "validate_selected_robot_camera", changing)
    with pytest.raises(ValueError, match="source changed"):
        detector.ItemDetectNode._validate_sources(node)
    assert getattr(node, "_source_binding", None) is None


@pytest.mark.parametrize("value", [None, "auto", "8", "0", "08", ""])
def test_calibration_config_accepts_optional_auto_cpu_keys(tmp_path, value):
    base = "ROS_LOCALHOST_ONLY=1\nDOBOT_ROBOT_LAN1_IP=192.168.20.204\n"
    (tmp_path / ".env.example").write_text(base + "ITEM_TORCH_THREADS=auto\nTRAY_TORCH_THREADS=auto\n")
    (tmp_path / ".env").write_text(base if value is None else base + f"ITEM_TORCH_THREADS={value}\n")
    if value in (None, "auto", "8"):
        assert platform.load_robot_lan1_ip(tmp_path) == "192.168.20.204"
    else:
        with pytest.raises(ValueError):
            platform.load_robot_lan1_ip(tmp_path)
