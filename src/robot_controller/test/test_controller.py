import ast
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from rclpy.parameter import Parameter

from item_perception_yolo import item_teach_core as core
from robot_controller.controller import PackageEventLogger, RobotController, inspect_profile


@pytest.fixture
def pair(tmp_path):
    model = tmp_path / "source.pt"
    model.write_bytes(b"opaque synthetic bytes; never deserialized")
    home = core.record_home(
        list(core.JOINT_NAMES), [0.1] * 6, 100, 0, now_ns=100_500_000_000,
        robot_ip="192.168.20.204", publisher="/dobot_bringup_ros2",
    )
    settings = {
        "item": {"name": "test"}, "model_task": "segment",
        "geometry_source": "mask", "quality": dict(core.QUALITY_DEFAULTS),
        "motion": dict.fromkeys(core.MOTION_FIELDS, 10.0),
        "timing": {"pick_settling": 1.0},
        "gripper": {"use_grip": False, "grip_onpick": True},
        "retry": {"retry_limit": 3},
        "geometry": {"height": 100.0, "width": 50.0, "tolerance": 5.0,
                     "pickdepth_radius": 30.0},
        "yolo": {"confidence": 0.5, "iou": 0.5, "image_size": 640,
                 "max_detections": 20, "class_ids": [0]},
    }
    path, profile = core.save_item_profile(settings, home, model, root=tmp_path)
    return tmp_path, path, profile


def test_other_station_home_is_portable_provenance_only(pair):
    root, path, profile = pair
    result = inspect_profile(path, root=root, robot_ip="192.168.20.205",
                             publisher_node="/destination_bringup")
    assert result["state"] == "PROFILE_VALIDATED_NOT_ARMED"
    assert result["home"] == profile["home"]
    assert result["home_identity_policy"] == "recording_provenance_only"
    assert result["requested_pose_count"] == 3
    assert result["maximum_detections"] == 20
    assert result["execution_enabled"] is False
    assert result["inference_enabled"] is False
    path.with_suffix(".pt").write_bytes(b"tampered")
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        inspect_profile(path, root=root, robot_ip="192.168.20.205", publisher_node="/other")


def test_atomic_parameter_validation_and_rejection():
    node = SimpleNamespace(_load=MagicMock(), events=MagicMock(), summary={"message": "valid"})
    result = RobotController._set_parameters(node, [Parameter("item_teach_file", value="a.yaml")])
    assert result.successful
    node._load.assert_called_once_with("a.yaml")
    for parameters in ([], [Parameter("other", value="a")],
                       [Parameter("item_teach_file", value="")],
                       [Parameter("item_teach_file", value=1)]):
        assert not RobotController._set_parameters(node, parameters).successful
    node._load.side_effect = ValueError("bad model hash")
    result = RobotController._set_parameters(node, [Parameter("item_teach_file", value="bad")])
    assert not result.successful
    assert "bad model hash" in result.reason


def test_validation_failure_explicitly_not_armed():
    node = SimpleNamespace(profile_path=None, events=MagicMock(), _publish=MagicMock())
    result = RobotController._validate(node, None, SimpleNamespace())
    assert not result.success
    assert node.summary["state"] == "PROFILE_INVALID"
    assert node.summary["execution_enabled"] is False
    node._publish.assert_called_once()


def test_bounded_package_events(tmp_path):
    logger = PackageEventLogger(tmp_path)
    for i in range(1001):
        logger.record("INFO", "test", "synthetic event", sequence=i)
    records = [json.loads(line) for line in logger.path.read_text().splitlines()]
    assert len(records) == 1
    assert records[0]["sequence"] == 1000
    assert records[0]["timestamp_utc"].endswith("Z")


def test_read_only_controller_pose_request_checks_fresh_batch(pair):
    import threading
    from rclpy.task import Future
    from item_perception_interfaces.msg import ItemCandidate
    from item_perception_interfaces.srv import GetItemPoses
    root, path, profile = pair
    result = GetItemPoses.Response(success=True, status="SHORTAGE", message="One candidate")
    result.header.frame_id = "platform_reference"
    result.header.stamp.sec = result.depth_stamp.sec = 100
    result.batch_id = "fixture"
    result.diagnostics_json = json.dumps({"profile_sha256": core.file_sha256(path)})
    candidate = ItemCandidate(id="fixture:1", priority=1, class_id=0, confidence=.9,
                              center_distance=.01)
    candidate.pose.orientation.w = 1.
    candidate.pose.position.z = .1
    result.candidates = [candidate]
    future = Future()
    future.set_result(result)
    node = SimpleNamespace(pose_lock=threading.Lock(), profile_path=path, root=root,
        events=MagicMock(), pose_client=MagicMock(), get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100_200_000_000)))
    node.pose_client.call_async.return_value = future
    response = RobotController._request_poses(node, None, SimpleNamespace())
    assert response.success
    summary = json.loads(response.message)
    assert not summary["execution_enabled"] and summary["targets"][0]["position_m"][2] == .1
    sent = node.pose_client.call_async.call_args.args[0]
    assert sent.max_candidates == profile["retry"]["retry_limit"]
    assert sent.profile_sha256 == core.file_sha256(path)
    result.header.stamp.sec = 90
    assert not RobotController._request_poses(node, None, SimpleNamespace()).success
    result.header.stamp.sec = 100
    result.header.frame_id = "base_link"
    assert not RobotController._request_poses(node, None, SimpleNamespace()).success


def test_no_hardware_clients_or_model_deserialization():
    import robot_controller.controller as controller
    from item_perception_yolo import item_teach_gui
    for module in (controller, item_teach_gui, core):
        tree = ast.parse(Path(module.__file__).read_text())
        calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
        client_calls = [node for node in calls if isinstance(node.func, ast.Attribute)
                        and node.func.attr == "create_client"]
        if module is item_teach_gui:
            assert len(client_calls) == 1
            assert client_calls[0].args[1].value == "/robot_controller/set_parameters_atomically"
        elif module is controller:
            assert len(client_calls) == 1
            assert client_calls[0].args[1].value == "/item_detect/get_item_poses"
        else:
            assert not client_calls
        imports = [node for node in ast.walk(tree)
                   if isinstance(node, (ast.Import, ast.ImportFrom))]
        assert all("dobot_msgs_v4" not in ast.unparse(node) for node in imports)
        assert all("torch" not in ast.unparse(node) for node in imports)
