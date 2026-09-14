import copy
import json
import threading
import time
import zipfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
import yaml

from item_perception_yolo import item_teach_core as core
from item_perception_yolo import item_teach_gui as gui
from item_perception_yolo import ui_state
from item_perception_yolo.item_teach_recovery import recover_item_fields


@pytest.fixture
def settings():
    return {
        "item": {"name": "test_part"}, "model_task": "detect",
        "geometry_source": "none", "quality": dict(core.QUALITY_DEFAULTS),
        "motion": {"standoff_height": 150.0, "zheight_offset": 250.0,
                   "prepick_height": 50.0, "retract_height": 80.0},
        "timing": {"pick_settling": 0.5},
        "gripper": {"use_grip": True, "grip_onpick": True},
        "retry": {"pose_candidates": 3},
        "geometry": {"height": 100.0, "width": 50.0, "tolerance": 5.0,
                     "pickdepth_radius": 30.0},
        "yolo": {"confidence": 0.6, "iou": 0.5, "image_size": 640,
                 "max_detections": 20, "class_ids": [0, 1]},
    }


@pytest.fixture
def home():
    return core.record_home(
        list(reversed(core.JOINT_NAMES)), [0.6, 0.5, 0.4, 0.3, 0.2, 0.1], 100, 1,
        now_ns=100_500_000_001, robot_ip="192.168.20.204", publisher="/dobot_bringup_ros2",
    )


@pytest.fixture
def pair(tmp_path, settings, home):
    # Opaque bytes exercise integrity only: this stage never deserializes weights.
    source = tmp_path / "external models" / "weights.pt"
    source.parent.mkdir()
    source.write_bytes(b"opaque test weights; not an inference fixture")
    root = tmp_path / "workspace"
    path, profile = core.save_item_profile(settings, home, source, root=root)
    return root, source, path, profile


def test_anywhere_source_becomes_independent_local_pair(pair):
    root, source, path, expected = pair
    assert path.parent == root / "offline_teach/item_teach"
    assert path.with_suffix(".pt").read_bytes() == source.read_bytes()
    assert str(source) not in path.read_text()
    source.unlink()
    profile, digest = core.load_item_profile(path, root=root)
    assert profile == expected
    assert digest == core.file_sha256(path)
    assert profile["home"]["positions_rad"] == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert profile["controller_contract"]["motion_enabled"] is False
    assert profile["model"]["verification"] == "file_sha256_only"
    assert profile["schema_version"] == 4
    assert profile["retry"] == {"pose_candidates": 3}
    assert "retry_limit" not in path.read_text()


def save_target(pair):
    root, _, path, profile = pair
    return core.item_save_target(
        path, profile["item"]["name"], core.file_sha256(path),
        created_at_utc=profile["created_at_utc"], root=root)


def test_same_name_overwrites_yaml_keeps_weights_and_one_previous_version(pair):
    root, _, path, profile = pair
    model = path.with_suffix(".pt")
    original_model, inode = model.read_bytes(), model.stat().st_ino
    for confidence in (0.75, 0.85):
        original_yaml = path.read_bytes()
        settings = core.settings_from_profile(profile)
        settings["yolo"]["confidence"] = confidence
        result, profile = core.save_item_profile(
            settings, profile["home"], model, root=root, save_target=save_target(pair))
        assert result == path
        assert model.stat().st_ino == inode and model.read_bytes() == original_model
        saved, _ = core.load_item_profile(path, root=root)
        assert saved == profile and saved["yolo"]["confidence"] == confidence
        assert saved["created_at_utc"] == pair[3]["created_at_utc"]
        with zipfile.ZipFile(path.with_name(f".{path.stem}.previous.zip")) as backup:
            assert backup.read(path.name) == original_yaml
            assert backup.read(model.name) == original_model
    assert list(path.parent.glob("*.yaml")) == [path]
    assert len(list(path.parent.glob(".*.previous.zip"))) == 1
    assert not [p for p in path.parent.glob(".item_teach_*") if p.is_dir()]


def test_changed_name_creates_new_pair_and_preserves_loaded_pair(pair):
    root, source, path, profile = pair
    original = path.read_bytes(), path.with_suffix(".pt").read_bytes()
    settings = core.settings_from_profile(profile)
    settings["item"]["name"] = "different_item"
    result, saved = core.save_item_profile(
        settings, profile["home"], source, root=root, save_target=save_target(pair))
    assert result != path and result.name.startswith("item_teach_different_item_")
    assert core.load_item_profile(result, root=root)[0] == saved
    assert original == (path.read_bytes(), path.with_suffix(".pt").read_bytes())
    assert not list(path.parent.glob(".*.previous.zip"))


def test_same_name_can_replace_model_with_verified_local_copy(pair):
    root, source, path, profile = pair
    target = save_target(pair)
    old_yaml, old_model = path.read_bytes(), path.with_suffix(".pt").read_bytes()
    source.write_bytes(b"new operator-selected weights")
    result, saved = core.save_item_profile(
        core.settings_from_profile(profile), profile["home"], source,
        root=root, save_target=target)
    assert result == path and path.with_suffix(".pt").read_bytes() == source.read_bytes()
    assert core.load_item_profile(path, root=root)[0] == saved
    with zipfile.ZipFile(path.with_name(f".{path.stem}.previous.zip")) as backup:
        assert backup.read(path.name) == old_yaml
        assert backup.read(path.with_suffix(".pt").name) == old_model


@pytest.mark.parametrize("file_suffix,remove", [
    (".yaml", False), (".pt", False), (".yaml", True), (".pt", True)])
def test_external_changes_block_overwrite_without_touching_files(pair, file_suffix, remove):
    root, source, path, profile = pair
    target = save_target(pair)
    changed = path.with_suffix(file_suffix)
    if remove:
        changed.unlink()
    else:
        changed.write_bytes(b"external change")
    observed = {p.name: p.read_bytes() for p in path.parent.iterdir()}
    with pytest.raises(ValueError, match="changed externally"):
        core.save_item_profile(core.settings_from_profile(profile), profile["home"], source,
                               root=root, save_target=target)
    assert observed == {p.name: p.read_bytes() for p in path.parent.iterdir()}


@pytest.mark.parametrize("replace_model", [False, True])
def test_update_publication_failure_preserves_old_pair_and_backup(
        pair, monkeypatch, replace_model):
    root, source, path, profile = pair
    target = save_target(pair)
    original = path.read_bytes(), path.with_suffix(".pt").read_bytes()
    if replace_model:
        source.write_bytes(b"selected replacement weights")
    real_replace = core.os.replace

    def fail_yaml(source_file, destination):
        if Path(destination) == path:
            # A concurrent strict reader must never accept mismatched YAML/weights.
            if replace_model:
                with pytest.raises(ValueError, match="SHA-256 mismatch"):
                    core.load_item_profile(path, root=root)
            raise OSError("injected YAML failure")
        real_replace(source_file, destination)

    monkeypatch.setattr(core.os, "replace", fail_yaml)
    with pytest.raises(OSError, match="injected YAML failure"):
        core.save_item_profile(core.settings_from_profile(profile), profile["home"], source,
                               root=root, save_target=target)
    assert original == (path.read_bytes(), path.with_suffix(".pt").read_bytes())
    assert core.load_item_profile(path, root=root)[0] == profile
    assert path.with_name(f".{path.stem}.previous.zip").is_file()


def test_backup_failure_leaves_pair_untouched(pair, monkeypatch):
    root, source, path, profile = pair
    target = save_target(pair)
    original = path.read_bytes(), path.with_suffix(".pt").read_bytes()
    real_replace = core.os.replace

    def fail_backup(source_file, destination):
        if Path(destination).suffix == ".zip":
            raise OSError("injected backup failure")
        real_replace(source_file, destination)

    monkeypatch.setattr(core.os, "replace", fail_backup)
    with pytest.raises(OSError, match="backup failure"):
        core.save_item_profile(core.settings_from_profile(profile), profile["home"], source,
                               root=root, save_target=target)
    assert original == (path.read_bytes(), path.with_suffix(".pt").read_bytes())


@pytest.mark.parametrize("known_name", [False, True])
def test_recovery_target_with_missing_weights_only_overwrites_known_item_name(pair, known_name):
    root, source, path, profile = pair
    path.write_bytes(b"old incomplete content")
    path.with_suffix(".pt").unlink()
    target = core.item_save_target(path, profile["item"]["name"] if known_name else None,
                                   core.file_sha256(path), root=root)
    output, saved = core.save_item_profile(
        core.settings_from_profile(profile), profile["home"], source, root=root, save_target=target)
    assert (output == path) == known_name
    assert core.load_item_profile(output, root=root)[0] == saved
    if known_name:
        with zipfile.ZipFile(path.with_name(f".{path.stem}.previous.zip")) as backup:
            assert backup.namelist() == [path.name]
            assert backup.read(path.name) == b"old incomplete content"
    else:
        assert path.read_bytes() == b"old incomplete content"


@pytest.mark.parametrize("file_suffix", [".yaml", ".pt"])
def test_save_target_rejects_symlink_replacement(pair, file_suffix):
    root, source, path, profile = pair
    target = save_target(pair)
    replaced = path.with_suffix(file_suffix)
    other = path.parent / ("other" + file_suffix)
    replaced.rename(other)
    replaced.symlink_to(other)
    with pytest.raises(ValueError, match="regular local"):
        core.save_item_profile(core.settings_from_profile(profile), profile["home"], source,
                               root=root, save_target=target)
    assert replaced.is_symlink()


def test_model_tamper_and_missing_pair_fail(pair):
    root, _source, path, _ = pair
    path.with_suffix(".pt").write_bytes(b"changed")
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        core.load_item_profile(path, root=root)
    path.with_suffix(".pt").unlink()
    with pytest.raises(ValueError, match="missing or empty"):
        core.load_item_profile(path, root=root)


def test_no_alternate_paths_or_model_pair_names(pair, tmp_path):
    root, _source, path, profile = pair
    with pytest.raises(ValueError, match="directly in"):
        core.load_item_profile(tmp_path / path.name, root=root)
    profile["model"]["filename"] = "different.pt"
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match="same stem"):
        core.load_item_profile(path, root=root)
    profile["model"]["filename"] = "../escape.pt"
    with pytest.raises(ValueError, match="paired local"):
        core.validate_profile(profile)


def test_duplicate_yaml_keys_and_old_schema_rejected(pair):
    root, _source, path, profile = pair
    path.write_text(path.read_text() + "schema_version: 1\n")
    with pytest.raises(ValueError, match="Duplicate YAML key"):
        core.load_item_profile(path, root=root)
    for old_version in (1, 2, 3):
        profile["schema_version"] = old_version
        with pytest.raises(ValueError, match="exactly 4"):
            core.validate_profile(profile)


@pytest.mark.parametrize("fields", [{"retry_limit": 3},
                                   {"pose_candidates": 3, "retry_limit": 3}, {}])
def test_old_retry_name_is_not_an_alias(pair, fields):
    root, _, path, profile = pair
    profile["retry"] = fields
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match="exactly: pose_candidates"):
        core.load_item_profile(path, root=root)


def test_gui_recovery_of_old_count_does_not_convert_file_or_weaken_runtime(pair):
    root, _, path, profile = pair
    profile["schema_version"] = 3
    profile["retry"] = {"retry_limit": 3}
    path.write_text(yaml.safe_dump(profile))
    original = path.read_bytes()
    draft = recover_item_fields(path, root=root)
    assert draft.values["pose_candidates"] == 3
    assert draft.values["confidence"] == .6 and draft.values["image_size"] == 640
    assert draft.home == profile["home"]
    assert draft.model_path == path.with_suffix(".pt")
    assert draft.model_sha256 == profile["model"]["sha256"]
    assert "retry_limit" in " ".join(draft.issues)
    with pytest.raises(ValueError, match="exactly 4"):
        core.load_item_profile(path, root=root)
    assert path.read_bytes() == original


def test_recovery_clears_invalid_values_and_conflicts_without_defaults(pair):
    root, _, path, profile = pair
    profile["motion"]["retract_height"] = ""
    profile["yolo"].update(confidence=40, class_ids=[0, "1"], image_size=None)
    profile["gripper"]["use_grip"] = "false"
    profile["geometry"].update(height=20, width=50)
    profile["quality"].update(depth_min_mm=1000, depth_max_mm=200)
    profile["home"]["positions_rad"][0] = float("nan")
    profile["retry"] = {"retry_limit": 3, "pose_candidates": 5}
    path.write_text(yaml.safe_dump(profile))
    draft = recover_item_fields(path, root=root)
    for key in ("retract_height", "confidence", "class_ids", "image_size", "use_grip",
                "height", "width", "depth_min_mm", "depth_max_mm", "pose_candidates"):
        assert draft.values[key] is None, key
    assert draft.home is None
    assert draft.values["standoff_height"] == 150.
    assert draft.values["grip_onpick"] is True
    assert draft.model_path is not None and draft.issues


def test_recovery_unknown_units_and_missing_sections_are_not_guessed(pair):
    root, _, path, profile = pair
    profile["units"] = {"distance": "m", "time": "ms", "home_joints": "deg"}
    del profile["retry"]
    del profile["gripper"]
    path.write_text(yaml.safe_dump(profile))
    draft = recover_item_fields(path, root=root)
    assert draft.home is None
    for key in (*core.MOTION_FIELDS, *core.GEOMETRY_FIELDS, *core.GRIPPER_FIELDS,
                "pick_settling", "pose_candidates", "input_max_age_sec", "depth_min_mm"):
        assert draft.values[key] is None, key
    assert draft.values["iou"] == .5 and draft.values["max_detections"] == 20


@pytest.mark.parametrize("content", ["item: [", "schema_version: 3\nschema_version: 4",
                                    "!!python/object/apply:os.system ['false']", "[]",
                                    "artifact_type: bin_teach\nschema_version: 3",
                                    "artifact_type: item_teach\nschema_version: 999"])
def test_uninterpretable_yaml_recovers_only_an_empty_draft(pair, content):
    root, _, path, _ = pair
    path.write_text(content)
    draft = recover_item_fields(path, root=root)
    assert not draft.values and draft.home is None and draft.model_path is None
    assert draft.issues and path.read_text() == content


@pytest.mark.parametrize("failure", ["missing", "tampered", "wrong_name", "wrong_hash"])
def test_recovery_never_uses_unverified_model_bytes(pair, failure):
    root, _, path, profile = pair
    if failure == "missing":
        path.with_suffix(".pt").unlink()
    elif failure == "tampered":
        path.with_suffix(".pt").write_bytes(b"changed")
    elif failure == "wrong_name":
        profile["model"]["filename"] = "../external.pt"
    else:
        profile["model"]["sha256"] = None
    path.write_text(yaml.safe_dump(profile))
    draft = recover_item_fields(path, root=root)
    assert draft.model_path is None and draft.model_sha256 is None
    assert draft.values["name"] == "test_part"
    assert any("Model cleared" in message for message in draft.issues)


@pytest.mark.parametrize("section,key,value", [
    ("motion", "standoff_height", -1), ("motion", "retract_height", float("nan")),
    ("timing", "pick_settling", True), ("gripper", "use_grip", 1),
    ("retry", "pose_candidates", 0), ("retry", "pose_candidates", 21),
    ("retry", "pose_candidates", True), ("retry", "pose_candidates", 3.0),
    ("retry", "pose_candidates", "3"), ("retry", "pose_candidates", 1001),
    ("yolo", "max_detections", 1.0), ("yolo", "image_size", 641),
    ("yolo", "confidence", 1.1), ("yolo", "class_ids", []),
    ("yolo", "class_ids", [0, 0]), ("yolo", "class_ids", [-1]),
    ("geometry", "height", 0), ("geometry", "width", -1),
    ("geometry", "tolerance", -1), ("geometry", "pickdepth_radius", 0),
])
def test_strict_grouped_settings(settings, section, key, value):
    settings[section][key] = value
    with pytest.raises(ValueError):
        core.validate_settings(settings)


def test_disabled_grip_retains_but_does_not_enable_onpick(settings):
    settings["gripper"] = {"use_grip": False, "grip_onpick": True}
    core.validate_settings(settings)


@pytest.mark.parametrize("now_ns", [99_999_999_999, 101_000_000_001])
def test_home_stale_or_future_rejected(now_ns):
    with pytest.raises(ValueError, match="stale or future"):
        core.record_home(
            list(core.JOINT_NAMES), [0.1] * 6, 100, 0, now_ns=now_ns,
            robot_ip="192.168.20.204", publisher="/dobot_bringup_ros2",
        )


def test_home_invalid_joint_count_nan_zero_stamp_rejected(home):
    for field, value in (("positions_rad", [0.0] * 5),
                         ("positions_rad", [float("nan")] * 6),
                         ("joint_names", ["joint1"] * 6),
                         ("feedback_stamp", {"sec": 0, "nanosec": 0})):
        bad = copy.deepcopy(home)
        bad[field] = value
        with pytest.raises(ValueError):
            core.validate_home(bad)


def test_save_failure_and_collision_never_overwrite(pair, monkeypatch):
    root, source, path, profile = pair
    original = path.read_bytes(), path.with_suffix(".pt").read_bytes()
    monkeypatch.setattr(core, "utc_now", lambda: profile["created_at_utc"])
    with pytest.raises(FileExistsError):
        core.save_item_profile(core.settings_from_profile(profile), profile["home"],
                               source, root=root)
    assert original == (path.read_bytes(), path.with_suffix(".pt").read_bytes())
    assert not list(path.parent.glob(".item_teach_*"))


def test_model_copy_change_is_rejected(pair, monkeypatch):
    root, source, path, profile = pair

    def bad_copy(_source, destination):
        Path(destination).write_bytes(b"corrupted during copy")

    monkeypatch.setattr(core.shutil, "copyfile", bad_copy)
    with pytest.raises(ValueError, match="changed during copy"):
        core.save_item_profile(core.settings_from_profile(profile), profile["home"],
                               source, root=root)
    assert sorted(path.parent.iterdir()) == sorted([path, path.with_suffix(".pt")])


def test_yaml_publication_failure_rolls_back_only_new_copy(pair, monkeypatch):
    root, source, path, profile = pair
    real_link = core.os.link

    def fail_yaml(source_path, destination):
        if Path(destination).suffix == ".yaml":
            raise OSError("synthetic publication failure")
        real_link(source_path, destination)

    monkeypatch.setattr(core.os, "link", fail_yaml)
    with pytest.raises(OSError, match="publication failure"):
        core.save_item_profile(core.settings_from_profile(profile), profile["home"],
                               source, root=root)
    assert sorted(path.parent.iterdir()) == sorted([path, path.with_suffix(".pt")])


def test_ui_sections_preserved_and_older_state_rejected(tmp_path):
    state_path = tmp_path / "last_session.json"
    ui_state.write_platform_ui_state(state_path, "camera_to_hand_calibration_test.yaml")
    ui_state.write_bin_ui_state(state_path, "platform_calibration_test.yaml", "DICT_5X5_50", 30)
    ui_state.write_item_ui_state(state_path, "item_teach_test.yaml")
    ui_state.write_platform_ui_state(state_path, "camera_on_hand_calibration_test.yaml")
    restored = ui_state.load_package_ui_state(state_path)
    assert restored.item_profile_filename == "item_teach_test.yaml"
    assert restored.bin_marker_size_mm == 30
    assert restored.platform_camera_calibration_filename == "camera_on_hand_calibration_test.yaml"
    payload = json.loads(state_path.read_text())
    payload["schema_version"] = 4
    state_path.write_text(json.dumps(payload))
    with pytest.raises(ValueError, match="exactly 6"):
        ui_state.load_package_ui_state(state_path)


def test_record_home_requires_fresh_canonical_feedback(home):
    message = SimpleNamespace(
        name=list(core.JOINT_NAMES), position=home["positions_rad"],
        header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0)),
    )
    endpoint = SimpleNamespace(node_namespace="/", node_name="dobot_bringup_ros2")
    node = SimpleNamespace(
        publisher_node="/dobot_bringup_ros2", robot_ip="192.168.20.204", events=MagicMock(),
        _feedback_lock=threading.Lock(), _joints=message, _receipt=time.monotonic(),
        get_publishers_info_by_topic=lambda _: [endpoint],
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100_500_000_000),
        ),
    )
    assert gui.ItemTeachNode.capture_home(node)["positions_rad"] == home["positions_rad"]
    node._receipt = time.monotonic() - 2
    with pytest.raises(ValueError, match="No fresh"):
        gui.ItemTeachNode.capture_home(node)
    node.get_publishers_info_by_topic = lambda _: [endpoint, endpoint]
    with pytest.raises(ValueError, match="sole publisher"):
        gui.ItemTeachNode.capture_home(node)


def test_gui_prefill_and_portable_home_do_not_send_commands(pair, monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    root, _source, path, profile = pair
    app = gui.QtWidgets.QApplication.instance() or gui.QtWidgets.QApplication([])
    monkeypatch.setattr(gui, "item_directory", lambda: core.item_directory(root))
    monkeypatch.setattr(gui, "workspace_root", lambda: root)
    monkeypatch.setattr(gui, "load_item_profile", lambda p: core.load_item_profile(p, root=root))
    from item_perception_yolo.item_teach_recovery import recover_item_fields
    monkeypatch.setattr(gui, "recover_item_fields", lambda p: recover_item_fields(p, root=root))
    monkeypatch.setattr(gui, "ui_state_path", lambda: root / "logs/item/last_session.json")
    monkeypatch.setattr(gui, "load_package_ui_state", lambda _: SimpleNamespace(
        item_profile_filename=path.name, item_preview_camera_prefix=None,
        item_platform_filename=None, item_bin_filename=None,
    ))
    node = SimpleNamespace(events=MagicMock(), robot_ip="192.168.20.205",
                           applied=None, last_view=None,
                           disarm=MagicMock(), close_runtime=MagicMock(), service=None,
                           clear_selected_pose=MagicMock(),
                           yolo_enabled=False, native=SimpleNamespace(failed=False),
                           camera_snapshot=lambda: (None, "No camera in fixture"))
    window = gui.ItemTeachWindow(node)
    try:
        assert window.home == profile["home"]
        assert window.saved_path == path
        assert not hasattr(window, "send")
        window._load(path, prefill=False)
        assert window.saved_path == path
        assert window._settings() == core.settings_from_profile(profile)
        assert "image_size" not in window.inputs
        # Removing the widget must not silently rewrite a loaded profile's input size.
        changed = copy.deepcopy(profile)
        changed["yolo"]["image_size"] = 1280
        path.write_text(yaml.safe_dump(changed))
        window._load(path, prefill=False)
        assert window._settings()["yolo"]["image_size"] == 1280
        path.write_text(yaml.safe_dump(profile))
        window._load(path, prefill=False)
        window.inputs["confidence"].setText("0.7")
        assert window.saved_path is None
        monkeypatch.setattr(gui.QtWidgets.QMessageBox, "question", lambda *_: (
            gui.QtWidgets.QMessageBox.Yes
        ))
        confirmation = MagicMock()
        monkeypatch.setattr(gui.QtWidgets.QMessageBox, "information", confirmation)
        monkeypatch.setattr(gui.QtWidgets.QMessageBox, "warning", MagicMock())
        window._save()
        assert window.saved_path == path
        assert window.saved_path.is_file()
        assert window.home["robot_lan1_ip"] != node.robot_ip  # provenance, not a binding
        confirmation.assert_called_once()
        # The source chooser imposes no workspace-root restriction.
        window._populate_classes({0: "ignore", 1: "pick", 2: "ignore_too"}, [])
        assert window._selected_classes() == []
        window.classes.item(1).setCheckState(gui.QtCore.Qt.Checked)
        assert window._yolo_settings()["class_ids"] == [1]
        assert window.classes.item(1).text() == "1: pick"
        assert window.saved_path is None
        window._populate_sources(["mask", "obb"], "obb")
        assert window.geometry_source.currentData() == "obb"
        window._populate_sources(["mask"], "obb")
        assert window.geometry_source.currentData() == "none"
        external = root.parent / "other folder" / "selected.pt"
        monkeypatch.setattr(gui.QtWidgets.QFileDialog, "getOpenFileName", lambda *_: (
            str(external), "PyTorch model (*.pt)",
        ))
        window._browse_model()
        assert window.model.text() == str(external)
        assert window.saved_path is None
    finally:
        window.timer.stop()
        window.close()
        app.processEvents()
