"""Headless, request-driven tray detection from one pinned deployment pair."""

import os
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from item_perception_yolo.item_teach_core import file_sha256
from item_perception_yolo.runtime_teach import runtime_tray_catalog
from .node import TrayTeachNode
from .requests import REQUEST_TIMEOUT


def configure(node):
    path, model = runtime_tray_catalog(node.root)
    digest = file_sha256(path)
    profile = node.load_saved(path)
    node.yolo_enabled = True
    node.events.record("INFO", "runtime_tray_selected", "Loaded deployed tray pair",
                       profile=str(path), model=str(model), profile_sha256=digest)
    node.requests._fresh_view(0, 0, time.monotonic() + REQUEST_TIMEOUT, lambda: None)
    node.requests.arm(path, profile["settings"], digest)


def main(args=None):
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("ROS_LOCALHOST_ONLY=1 is required")
    rclpy.init(args=args)
    node = executor = thread = None
    try:
        node = TrayTeachNode(deployment=True)
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        configure(node)
        while rclpy.ok() and not node.fatal_error:
            time.sleep(.1)
        if node.fatal_error:
            raise RuntimeError(node.fatal_error)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.events.record("FATAL", "tray_detector_failed", str(exc))
            node.get_logger().fatal(str(exc))
        raise
    finally:
        if node is not None:
            node.close()
        if executor is not None:
            executor.shutdown(timeout_sec=2)
        if thread is not None:
            thread.join(timeout=2)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
