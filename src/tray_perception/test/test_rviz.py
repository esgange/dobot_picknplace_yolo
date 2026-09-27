from types import SimpleNamespace
from unittest.mock import MagicMock
import os
import subprocess
import sys
import textwrap

import numpy as np
import pytest
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from tray_perception import rviz


def sample(stamp=100, connection=1):
    points = np.zeros(2, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
    points["x"], points["z"], points["rgb"] = [.1, .2], .3, 0x112233
    return {"data": points.tobytes(), "point_count": 2, "stamp_ns": stamp,
            "depth_stamp_ns": stamp, "connection": connection, "detections": []}


@pytest.fixture
def cloud(monkeypatch):
    now = [10.]
    monkeypatch.setattr(rviz.time, "monotonic", lambda: now[0])
    node = SimpleNamespace(create_publisher=MagicMock(side_effect=lambda *_: MagicMock()))
    return rviz.TrayRvizPreview(node), now


def test_voxels_are_latched_in_base_with_original_timestamp(cloud):
    preview, _ = cloud
    qos = preview.node.create_publisher.call_args_list[0].args[2]
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL and qos.depth == 1
    preview.accept(sample())
    message = preview.publisher.publish.call_args.args[0]
    assert message.header.frame_id == "base_link" and message.header.stamp.nanosec == 100
    assert bytes(message.data) == sample()["data"] and message.width == 2


def test_gap_retains_then_greys_geometry_without_empty_cloud(cloud):
    preview, now = cloud
    preview.accept(sample())
    preview.hold("Depth missing")
    now[0] += 4.99
    preview.tick()
    assert preview.publisher.publish.call_count == 1 and not preview.grey
    now[0] += .02
    preview.tick()
    grey = preview.publisher.publish.call_args.args[0]
    original = np.frombuffer(sample()["data"], "<u4").reshape(-1, 4)
    actual = np.frombuffer(bytes(grey.data), "<u4").reshape(-1, 4)
    assert np.array_equal(actual[:, :3], original[:, :3])
    assert (actual[:, 3] == 0x808080).all() and grey.header.stamp.nanosec == 100
    now[0] += 100
    preview.tick()
    assert preview.publisher.publish.call_count == 2 and preview.grey
    assert preview.status()["reason"] == "Depth missing"


def test_duplicates_empty_clouds_and_invalidation_cannot_restore_color(cloud):
    preview, now = cloud
    preview.accept(sample())
    now[0] += 1
    preview.invalidate("Settings changed")
    preview.accept(sample())
    preview.accept({**sample(101), "point_count": 0, "data": b""})
    preview.tick()
    assert preview.grey and preview.publisher.publish.call_count == 2
    assert preview.refreshed_at == 10
    preview.accept(sample(102))
    assert not preview.grey and preview.publisher.publish.call_count == 3
    assert bytes(preview.publisher.publish.call_args.args[0].data) == sample()["data"]
    preview.invalidate("Orderly exit")
    assert preview.grey and preview.publisher.publish.call_count == 4


def test_canonical_rviz_has_independent_matching_tray_display():
    from pathlib import Path
    import yaml
    root = Path(__file__).parents[3]
    path = root / "src/DOBOT_6Axis_ROS2_V4/dobot_rviz/rviz/urdf.rviz"
    config = yaml.safe_load(path.read_text())
    displays = config["Visualization Manager"]["Displays"]
    cloud = next(d for d in displays if d.get("Topic", {}).get("Value") ==
                 "/tray_teach/voxel_cloud")
    assert cloud["Enabled"] and cloud["Decay Time"] == 0 and cloud["Size (m)"] == .01
    assert cloud["Topic"]["Durability Policy"] == "Transient Local"
    assert cloud["Topic"]["Reliability Policy"] == "Reliable" and cloud["Topic"]["Depth"] == 1


def test_late_ros_subscriber_receives_latest_grey_cloud():
    script = '''
        import struct
        import time
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from tray_perception.rviz import TrayRvizPreview

        rclpy.init()
        sender, receiver = Node("tray_voxel_test_sender"), Node("tray_voxel_test_receiver")
        try:
            preview = TrayRvizPreview(sender)
            preview.accept({"data": struct.pack("<fffI", .1, .2, .3, 0x112233),
                            "point_count": 1, "stamp_ns": 123, "depth_stamp_ns": 124,
                            "connection": 1, "detections": []})
            preview.invalidate("Synthetic source invalidation")
            received = []
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            receiver.create_subscription(PointCloud2, "/tray_teach/voxel_cloud",
                                         received.append, qos)
            deadline = time.monotonic() + 5
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(receiver, timeout_sec=.05)
            assert received, "Late transient-local subscriber received no retained cloud"
            message = received[-1]
            assert message.width == 1 and message.header.frame_id == "base_link"
            assert message.header.stamp.nanosec == 123
            assert bytes(message.data) == struct.pack("<fffI", .1, .2, .3, 0x808080)
        finally:
            receiver.destroy_node()
            sender.destroy_node()
            rclpy.shutdown()
    '''
    result = subprocess.run([sys.executable, "-c", textwrap.dedent(script)],
                            env={**os.environ, "ROS_DOMAIN_ID": "201", "ROS_LOCALHOST_ONLY": "1"},
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
