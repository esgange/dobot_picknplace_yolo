import copy
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from rclpy.time import Time
from sensor_msgs.msg import JointState

from tray_perception import core, documents
from tray_perception.node import TrayTeachNode
from test_core import artifact as saved_artifact
from test_core import plane, settings
from test_documents import ready_form


@pytest.fixture
def reader():
    node = SimpleNamespace(
        deployment=False, robot_ip="192.0.2.1", publisher_node="/dobot_bringup_ros2",
        lock=threading.RLock(), events=MagicMock(), _joints=None, _joint_receipt=None,
        get_publishers_info_by_topic=MagicMock(return_value=[SimpleNamespace(
            node_namespace="/", node_name="dobot_bringup_ros2")]),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100)))
    message = JointState()
    message.header.stamp = Time(seconds=99, nanoseconds=500_000_000).to_msg()
    message.name = [f"joint{i}" for i in range(6, 0, -1)]
    message.position = [1.6, -.5, .4, -.3, .2, -.1]
    TrayTeachNode._on_joints(node, message)
    return node


def test_detect_pose_records_joint_angles_and_roundtrips_draft_and_profile(
        reader, tmp_path):
    pose = TrayTeachNode.capture_detect_pose(reader)
    assert pose["joint_names"] == [f"joint{i}" for i in range(1, 7)]
    assert pose["positions_rad"] == [-.1, .2, -.3, .4, -.5, 1.6]
    assert pose["robot_lan1_ip"] == reader.robot_ip
    assert pose["publisher_node"] == reader.publisher_node
    assert pose["source_topic"] == "/joint_states"
    assert pose["feedback_stamp"] == {"sec": 99, "nanosec": 500_000_000}
    reader._joints.position[0] = 99.  # Recorded values are independent of later feedback.
    assert pose["positions_rad"][-1] == 1.6
    path, _, _, _ = documents.save_document(
        ready_form(), None, pose, None, None, None, tmp_path)
    assert documents.load_document(path, tmp_path)["tray_teach_position"] == pose
    _, camera, model = saved_artifact.__wrapped__(tmp_path)
    path = core.save_profile(settings(), pose, plane(), camera, model,
                             core.file_sha256(model), tmp_path)
    profile = core.load_profile(path, tmp_path)
    assert profile["tray_teach_position"] == pose
    assert profile["units"]["tray_teach_position"] == "rad"


@pytest.mark.parametrize("fault", [
    "missing", "stale_receipt", "stale_stamp", "future_stamp", "zero_stamp",
    "names", "missing_angle", "nan_angle", "infinite_angle"])
def test_detect_pose_rejects_unusable_joint_feedback(reader, fault):
    if fault == "missing":
        reader._joints = None
    elif fault == "stale_receipt":
        reader._joint_receipt = time.monotonic() - 1.1
    elif fault in ("stale_stamp", "future_stamp", "zero_stamp"):
        reader._joints.header.stamp = Time(
            seconds={"stale_stamp": 98, "future_stamp": 101, "zero_stamp": 0}[fault]).to_msg()
    elif fault == "names":
        reader._joints.name[0] = "joint1"
    elif fault == "missing_angle":
        reader._joints.position.pop()
    else:
        reader._joints.position[0] = float("nan" if fault == "nan_angle" else "inf")
    with pytest.raises(ValueError):
        TrayTeachNode.capture_detect_pose(reader)
    reader.events.record.assert_not_called()


@pytest.mark.parametrize("names", [[], ["other_robot"], ["dobot_bringup_ros2"] * 2])
def test_detect_pose_requires_one_canonical_publisher(reader, names):
    reader.get_publishers_info_by_topic.return_value = [
        SimpleNamespace(node_namespace="/", node_name=name) for name in names]
    with pytest.raises(ValueError, match="sole publisher"):
        TrayTeachNode.capture_detect_pose(reader)


def test_headless_does_not_record_feedback(reader):
    reader.deployment = True
    before = copy.deepcopy(reader._joints)
    with pytest.raises(ValueError, match="Record Tray Detect Pose in Tray Teach"):
        TrayTeachNode.capture_detect_pose(reader)
    reader.get_publishers_info_by_topic.assert_not_called()
    assert reader._joints == before


def exercise_joint_transport(root):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from tray_perception import node as module

    repository = Path(__file__).resolve().parents[3]
    for filename in (".env", ".env.example"):
        (root / filename).write_bytes((repository / ".env.example").read_bytes())
    module.workspace_root = lambda: root
    rclpy.init()
    node = module.TrayTeachNode()
    peer = Node(node.publisher_node.lstrip("/"))
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.add_node(peer)
    try:
        publisher = peer.create_publisher(JointState, "/joint_states", qos_profile_sensor_data)
        message = JointState()
        message.name = [f"joint{i}" for i in range(1, 7)]
        message.position = [-.1, .2, -.3, .4, -.5, .6]
        deadline = time.monotonic() + 5
        while node._joints is None and time.monotonic() < deadline:
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            executor.spin_once(timeout_sec=.05)
        assert node.capture_detect_pose()["positions_rad"] == list(message.position)
        assert node.camera is None and node.model is None and node.native.process is None
        assert not list(node.clients)  # Recording has no hardware command endpoint.
    finally:
        node.close()
        executor.shutdown()
        peer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_real_ros_joint_recording_without_camera_or_model(tmp_path):
    command = ("import sys; from pathlib import Path; sys.path.insert(0,sys.argv[1]); "
               "from test_detect_pose import exercise_joint_transport; "
               "exercise_joint_transport(Path(sys.argv[2]))")
    result = subprocess.run([sys.executable, "-c", command, str(Path(__file__).parent),
                             str(tmp_path)], capture_output=True, text=True, timeout=15,
                            env={**os.environ, "ROS_DOMAIN_ID": "205", "ROS_LOCALHOST_ONLY": "1"})
    assert result.returncode == 0, result.stdout + result.stderr
