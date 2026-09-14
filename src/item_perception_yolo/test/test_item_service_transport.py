"""Exercise generated service types/executor with synthetic streams in an isolated ROS domain."""

import os
from pathlib import Path
import subprocess


def exercise_service():
    import threading
    import time
    from pathlib import Path
    from types import SimpleNamespace
    from unittest.mock import MagicMock
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo
    from geometry_msgs.msg import TransformStamped
    from item_perception_interfaces.srv import GetItemPoses
    from item_perception_yolo import item_detector as module
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
    import numpy as np

    module.PackageEventLogger = lambda **_: MagicMock()
    rclpy.init()
    detector = module.ItemDetectNode("synthetic_item_detector")
    sensor = Node("synthetic_item_sensor")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(detector)
    executor.add_node(sensor)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        detector.connect_camera("synthetic")
        camera_settings = SimpleNamespace(camera_prefix="synthetic",
            camera_link_frame="synthetic_link", optical_frame="synthetic_color_optical_frame")
        transform = np.diag([1., -1., -1., 1.])
        transform[2, 3] = .8
        camera = SimpleNamespace(settings=camera_settings, calibration_mode="camera_to_hand",
            reference_frame="base_link", reference_from_camera_link=transform, sha256="c"*64)
        detector.applied = SimpleNamespace(camera=camera,
            platform=SimpleNamespace(base_from_platform=np.eye(4), sha256="d"*64))
        detector.bin_artifact = SimpleNamespace(sha256="e"*64, points=[
            SimpleNamespace(x_m=x, y_m=y) for x, y in [(-.2,-.2),(-.2,.2),(.2,.2),(.2,-.2)]])
        detector._validate_sources = MagicMock()
        settings = {"model_task": "segment", "geometry_source": "mask",
            "geometry": {"height": 100., "width": 50., "tolerance": 5., "pickdepth_radius": 30.},
            "quality": dict(QUALITY_DEFAULTS), "yolo": {"class_ids": [1], "confidence": .6,
                "max_detections": 20, "image_size": 640, "iou": .5}}
        profile = {**settings, "model": {"declared_task": "segment", "sha256": "b"*64},
            "item": {}, "motion": {}, "timing": {}, "gripper": {}, "retry": {"pose_candidates": 3}}
        module.load_item_profile = lambda _: (profile, "a"*64)
        module.file_sha256 = lambda _: "a"*64
        detector.model_config = {"sha256": "b"*64}
        detector.model_metadata = {"classes": {"1": "part"}, "task": "segment",
                                   "geometry_sources": ["mask"]}
        detector.enable_yolo(settings)
        tf = TransformStamped()
        tf.header.frame_id = "synthetic_link"
        tf.child_frame_id = "synthetic_color_optical_frame"
        tf.transform.rotation.w = 1.
        detector.tf_buffer.set_transform_static(tf, "synthetic_fixture")
        publishers = {topic: sensor.create_publisher(kind, f"/synthetic/{topic}", qos_profile_sensor_data)
            for topic, kind in (("color/image_raw", Image), ("depth/image_raw", Image),
                               ("color/camera_info", CameraInfo), ("depth/camera_info", CameraInfo))}

        def publish():
            for topic, publisher in publishers.items():
                if topic.endswith("camera_info"):
                    message = CameraInfo()
                    message.width, message.height = 16, 8
                    message.k = [20.,0.,8.,0.,20.,4.,0.,0.,1.]
                    message.d = [0.] * 5
                    message.distortion_model = "plumb_bob"
                else:
                    message = Image()
                    message.width, message.height = 16, 8
                    message.encoding = "rgb8" if topic.startswith("color") else "16UC1"
                    message.step = 16 * (3 if message.encoding == "rgb8" else 2)
                    message.data = bytes(message.step * 8)
                message.header.frame_id = "synthetic_color_optical_frame"
                message.header.stamp = sensor.get_clock().now().to_msg()
                publisher.publish(message)
        timer = sensor.create_timer(.05, publish)
        detector._snapshot(0, time.monotonic()+5, wait=True)
        detector.arm(Path("synthetic_profile.yaml"))
        client = sensor.create_client(GetItemPoses, module.SERVICE_NAME)
        assert client.wait_for_service(timeout_sec=5)
        observed = []

        def infer(rgb, depth, context, **_):
            observed.append((rgb["stamp_ns"], depth["stamp_ns"], context))
            candidate = {"source_index": 0, "class_id": 1, "class_name": "part", "confidence": .9,
                "position": [.01,.02,.1], "quaternion": [0.,0.,0.,1.], "length": .1,
                "width": .05, "center_distance": .02, "filtered_camera_depth": .7,
                "depth_sigma": .001, "accepted_depth_count": 100, "rejected_depth_count": 2}
            return {"metadata": {"count": 1, "candidates": [candidate], "rejected": [],
                                  "inference_ms": 1.}}
        detector.infer = infer
        start = sensor.get_clock().now().nanoseconds
        future = client.call_async(GetItemPoses.Request(max_candidates=3, profile_sha256="a"*64))
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        assert done.wait(5)
        result = future.result()
        assert result.success, result.message
        assert result.status == "SHORTAGE" and len(result.candidates) == 1
        assert result.header.frame_id == "platform_reference"
        assert min(observed[0][:2]) >= start
        assert np.allclose(observed[0][2]["platform_from_optical"], transform)
        # A delayed TF must not cause the detector to chase newer incoming frames.
        real_lookup = detector.tf_buffer.lookup_transform
        looked_up = []
        def delayed_lookup(target, source, instant):
            looked_up.append(instant.nanoseconds)
            if len(looked_up) == 1:
                time.sleep(.12)  # synthetic sensor publishes new frames during this delay
                raise module.TransformException("Synthetic delayed TF")
            return real_lookup(target, source, instant)
        detector.tf_buffer.lookup_transform = delayed_lookup
        detector._snapshot(0, time.monotonic()+1, wait=True)
        assert len(looked_up) >= 2 and len(set(looked_up)) == 1
        detector.tf_buffer.lookup_transform = real_lookup
        detector.disarm()
        assert detector.service is None
        # A local simulation stays unarmed yet uses the same fresh-observation
        # acquisition, timestamped TF and typed response as the real service.
        start = sensor.get_clock().now().nanoseconds
        simulated = detector.simulate_trigger(Path("synthetic_profile.yaml"), expected_digest="a"*64)
        assert simulated["response"].success, simulated["response"].message
        assert simulated["response"].status == "SHORTAGE"
        assert simulated["response"].candidates[0].pose == result.candidates[0].pose
        assert min(observed[-1][:2]) >= start
        assert detector.service is None
        assert simulated["response"].batch_id != result.batch_id
        # Re-arm the same node: discovery of its retired endpoint must not self-collide.
        detector.arm(Path("synthetic_profile.yaml"))
        assert detector.service is not None
        detector.disarm()
        sensor.destroy_timer(timer)
        print("synthetic ROS service: fresh pair, TF, typed response, disarm/rearm passed")
    finally:
        detector.close_runtime()
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        sensor.destroy_node()
        detector.destroy_node()
        rclpy.shutdown()


def test_synthetic_ros_service_transport():
    code = "import runpy,sys; runpy.run_path(sys.argv[1])['exercise_service']()"
    result = subprocess.run(["/usr/bin/python3", "-c", code, str(Path(__file__).resolve())],
        env=dict(os.environ, ROS_DOMAIN_ID="232", ROS_LOCALHOST_ONLY="1"),
        capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
