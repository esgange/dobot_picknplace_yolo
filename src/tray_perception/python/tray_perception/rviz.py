"""Retained read-only tray voxels, independent of YOLO and the taught plane."""

import copy
import json
import threading
import time

import numpy as np
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from item_perception_yolo.item_teach_rviz import cloud_message


class TrayRvizPreview:
    def __init__(self, node, *, topic_prefix="/tray_teach"):
        self.node = node
        self.lock = threading.RLock()
        self.displayed = None
        self.refreshed_at = None
        self.grey = False
        self.reason = "Waiting for calibrated RGB/depth"
        self.next_status = 0.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = node.create_publisher(PointCloud2, f"{topic_prefix}/voxel_cloud", qos)
        self.diagnostics = node.create_publisher(String, f"{topic_prefix}/rviz_diagnostics", 1)

    def accept(self, snapshot):
        with self.lock:
            if snapshot is None or not snapshot["point_count"]:
                self.hold("No valid depth voxels")
                return
            previous = self.displayed
            if previous is not None and previous["connection"] == snapshot["connection"]:
                if (snapshot["stamp_ns"] <= previous["stamp_ns"]
                        or snapshot["depth_stamp_ns"] <= previous["depth_stamp_ns"]):
                    self.hold("Waiting for a new RGB/depth observation")
                    return
            self.displayed = snapshot
            self.refreshed_at = time.monotonic()
            self.grey, self.reason = False, ""
            self.publisher.publish(cloud_message(snapshot["data"], snapshot["point_count"],
                                                 Time(nanoseconds=snapshot["stamp_ns"]).to_msg()))

    def hold(self, reason):
        with self.lock:
            self.reason = reason

    def invalidate(self, reason):
        with self.lock:
            self.reason = reason
            if self.displayed is None or self.grey:
                return
            sample = self.displayed
            data = bytearray(sample["data"])
            np.frombuffer(data, "<u4").reshape(-1, 4)[:, 3] = 0x808080
            self.publisher.publish(cloud_message(
                bytes(data), sample["point_count"], Time(nanoseconds=sample["stamp_ns"]).to_msg()))
            self.grey = True

    def tick(self):
        with self.lock:
            now = time.monotonic()
            if self.displayed is not None and now - self.refreshed_at >= 5.:
                self.invalidate(self.reason or "No fresh validated voxels for five seconds")
            if now < self.next_status:
                return
            self.next_status = now + 1.
            self.diagnostics.publish(String(data=json.dumps(self.status(), allow_nan=False)))

    def status(self):
        with self.lock:
            sample = self.displayed
            return {"status": "waiting" if sample is None else (
                "stale_grey" if self.grey else "retained_cloud" if self.reason else "fresh"),
                "reason": self.reason, "voxel_size_mm": 10,
                "point_count": 0 if sample is None else sample["point_count"],
                "source_stamp_ns": None if sample is None else sample["stamp_ns"],
                "depth_stamp_ns": None if sample is None else sample["depth_stamp_ns"],
                "refresh_age_sec": None if sample is None else
                max(0., time.monotonic() - self.refreshed_at),
                "detections": [] if sample is None or self.grey else
                copy.deepcopy(sample["detections"])}
