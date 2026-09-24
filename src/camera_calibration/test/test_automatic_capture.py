"""Fresh observations, handshake ordering, cancellation and manual-edit isolation."""

from types import SimpleNamespace
from unittest.mock import MagicMock
import threading
import time

import numpy as np
import pytest
from rclpy.clock import ClockType
from rclpy.time import Time
from robot_controller_interfaces.action import ReplayCalibration
from robot_controller_interfaces.srv import CaptureCalibration

from camera_calibration_gui import automatic_capture as automatic_module
from camera_calibration_gui.automatic_capture import AutomaticCapture
from camera_calibration_gui.main import CalibrationNode, TransformStamped


def ros_time(seconds):
    return Time(seconds=seconds, clock_type=ClockType.ROS_TIME)


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(automatic_module, "ActionClient", MagicMock())
    result = object.__new__(CalibrationNode)
    result._lock = threading.RLock()
    result._event_logger = MagicMock()
    result.create_client = MagicMock()
    result.create_service = MagicMock()
    result.create_timer = MagicMock()
    result._fatal_error = None
    result._last_frame_metadata = {"color_stamp_ns": ros_time(9.9).nanoseconds}
    result._latest_valid_rgb_stamp = ros_time(9.9).nanoseconds
    result._camera_matrix = np.eye(3)
    result._latest_pose = np.eye(4)
    result._latest_pose_stamp = ros_time(9.9).nanoseconds
    result._latest_pose_time = time.monotonic()
    result._latest_corner_count = 8
    result._latest_joint_positions = (0.,) * 6
    result._latest_joint_state_stamp = ros_time(9.9)
    result._calibration_samples = []
    result._minimum_samples = 5
    result._next_calibration_sample_number = 1
    result._solution = result._diagnostics = result._latest_overlay = None
    result._target_gate_locked = lambda: (True, "ready")
    result.get_clock = lambda: SimpleNamespace(now=lambda: ros_time(10.))
    transform = TransformStamped()
    transform.header.stamp = ros_time(9.9).to_msg()
    transform.transform.rotation.w = 1.
    result._tf_buffer = SimpleNamespace(lookup_transform=lambda *_a, **_k: transform)
    result.automatic = AutomaticCapture(result)
    result.automatic.recipe = SimpleNamespace(samples=[
        SimpleNamespace(joint_positions_rad=(0.,) * 6) for _ in range(5)])
    result.automatic.active = True
    result.automatic.run_id = "a" * 32
    return result


def finish_inline(coroutine):
    with pytest.raises(StopIteration) as stopped:
        coroutine.send(None)
    return stopped.value.value


def prepare(node, **changes):
    values = dict(phase=CaptureCalibration.Request.PREPARE,
                  run_id=node.automatic.run_id, position_index=1)
    values.update(changes)
    return finish_inline(node.automatic._service(
        CaptureCalibration.Request(**values), CaptureCalibration.Response()))


def capture_coroutine(node):
    return node.automatic._service(CaptureCalibration.Request(
        phase=CaptureCalibration.Request.CAPTURE, run_id=node.automatic.run_id,
        position_index=1, not_before=ros_time(9.8).to_msg()), CaptureCalibration.Response())


def test_async_service_yields_until_fresh_capture_and_preserves_recipe(node):
    source_recipe = node.automatic.recipe
    node._calibration_samples = [object()] * 5
    assert prepare(node).success
    assert node._calibration_samples == []
    coroutine = capture_coroutine(node)
    coroutine.send(None)  # Pending Future releases the service's executor thread.
    assert node.automatic.pending is not None
    node.automatic._tick()
    response = finish_inline(coroutine)
    assert response.success and node.automatic.captured == 1
    assert len(node._calibration_samples) == 1
    assert node._calibration_samples[0].joint_positions_rad == (0.,) * 6
    assert node.automatic.recipe is source_recipe


@pytest.mark.parametrize("changes", [{"run_id": "wrong"}, {"position_index": 2}])
def test_wrong_session_or_order_cannot_reset_or_capture(node, changes):
    original = [object()] * 5
    node._calibration_samples = original
    assert not prepare(node, **changes).success
    assert node._calibration_samples is original


def test_preparation_requires_live_rgb_and_rejects_duplicates(node):
    node._latest_valid_rgb_stamp = None
    assert not prepare(node).success
    node._latest_valid_rgb_stamp = ros_time(9.9).nanoseconds
    assert prepare(node).success
    assert not prepare(node).success


@pytest.mark.parametrize("old_field", ["rgb", "joints", "tf", "stale_rgb", "wrong_joints"])
def test_capture_rejects_pre_arrival_or_stale_observations(node, old_field):
    if old_field == "rgb":
        node._latest_pose_stamp = ros_time(9.7).nanoseconds
    elif old_field == "stale_rgb":
        node._latest_pose_stamp = ros_time(9.4).nanoseconds
    elif old_field == "joints":
        node._latest_joint_state_stamp = ros_time(9.7)
    elif old_field == "wrong_joints":
        node._latest_joint_positions = (.1,) * 6
    else:
        node._tf_buffer.lookup_transform().header.stamp = ros_time(9.7).to_msg()
    success, _message = node.capture_sample(
        not_before_ns=ros_time(9.8 if old_field != "stale_rgb" else 9.3).nanoseconds,
        expected_joints=(0.,) * 6, run_id=node.automatic.run_id)
    assert not success and not node._calibration_samples


def test_stop_resolves_pending_capture_and_never_accepts_later_frame(node):
    assert prepare(node).success
    coroutine = capture_coroutine(node)
    coroutine.send(None)
    node.automatic.stop()
    assert not finish_inline(coroutine).success
    node.automatic._tick()
    assert node._calibration_samples == []
    node.automatic.stop_client.call_async.assert_called_once()


def test_manual_edits_and_save_are_blocked_during_run_and_partial_run_cannot_save(node):
    original = [object()] * 5
    node._calibration_samples = original
    assert not node.capture_sample()[0]
    assert not node.remove_sample("C1")[0]
    assert "Stop automatic" in node.reset_samples()
    assert not node.save()[0]
    assert node._calibration_samples is original
    with pytest.raises(RuntimeError, match="Stop automatic"):
        node.automatic.clear_recipe()
    node.automatic.active = False
    node.automatic.started = True
    assert not node.save()[0]
    recipe = node.automatic.recipe
    with pytest.raises(ValueError):
        node.configure(SimpleNamespace(validate=lambda: None), "invalid_mode")
    assert node.automatic.recipe is recipe and node.automatic.started
    assert not node.save()[0]


def test_invalid_frame_wait_is_bounded_and_not_reused(node):
    assert prepare(node).success
    node._latest_pose_stamp = ros_time(9.7).nanoseconds
    coroutine = capture_coroutine(node)
    coroutine.send(None)
    node.automatic._tick()
    assert node.automatic.pending is not None
    request, future, _deadline = node.automatic.pending
    node.automatic.pending = request, future, 0.
    node.automatic._tick()
    result = finish_inline(coroutine)
    assert not result.success and "10 seconds" in result.message
    assert not node._calibration_samples


def test_missing_controller_or_old_feedback_does_not_report_completion(node):
    node.automatic.client.server_is_ready.return_value = False
    node.automatic._tick()
    assert node.automatic.stopping and not node.automatic.complete
    assert "unconfirmed" in node.automatic.message
    node.automatic.message = "current run"
    node.automatic._feedback("old_run", SimpleNamespace(feedback=ReplayCalibration.Feedback()))
    assert node.automatic.message == "current run"


def test_fatal_camera_failure_requests_stop_once_without_automatic_retry(node):
    node._fatal_error = "test worker failure"
    for _ in range(3):
        node.automatic._tick()
    assert node.automatic.stopping
    node.automatic.stop_client.call_async.assert_called_once()


def test_gui_blocks_edits_and_partial_save_but_keeps_stop_available(node):
    from camera_calibration_gui.main import CalibrationWindow, QtWidgets

    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    node.load_last_session = lambda: None
    window = CalibrationWindow(node)
    window._timer.stop()
    try:
        window._configuration_applied = True
        window._set_action_states(True, True, 5, True, True)
        assert window.automatic_stop_button.isEnabled()
        for widget in (window.apply_button, window.load_button, window.capture_button,
                       window.reset_button, window.save_new_button, window.camera_prefix):
            assert not widget.isEnabled()
        node.automatic.active = False
        node.automatic.started = True
        window._set_action_states(True, True, 5, True, True)
        assert window.automatic_button.isEnabled()
        assert not window.save_new_button.isEnabled()
        node.automatic.complete = True
        window._set_action_states(True, True, 5, True, True)
        assert window.save_new_button.isEnabled()
        assert window.save_new_button.text() == "Save as New Calibration"
    finally:
        window.close()
        application.processEvents()


def test_real_ros_handshake_keeps_two_thread_camera_executor_responsive():
    """Exercise async rclpy scheduling with local fake sensors and no Dobot endpoints."""
    import rclpy
    from rclpy.action import ActionServer
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    context = Context()
    rclpy.init(context=context)

    class Camera(Node):
        def __init__(self):
            super().__init__("camera_calibration", context=context)
            self._lock = threading.RLock()
            self._event_logger = MagicMock()
            self._fatal_error = None
            self._calibration_samples = []
            self._camera_matrix = np.eye(3)
            self._latest_pose_stamp = None
            self._last_frame_metadata = None
            self._latest_valid_rgb_stamp = None
            self.automatic = AutomaticCapture(self)
            self.automatic.recipe = SimpleNamespace(samples=[
                SimpleNamespace(joint_positions_rad=(0.,) * 6) for _ in range(5)])
            self.frames = 0
            self.create_timer(.01, self.sensor)

        def sensor(self):
            self.frames += 1
            self._latest_pose_stamp = self.get_clock().now().nanoseconds
            self._last_frame_metadata = {"color_stamp_ns": self._latest_pose_stamp}
            self._latest_valid_rgb_stamp = self._latest_pose_stamp

        def reset_samples(self, **_kwargs):
            self._calibration_samples.clear()

        def capture_sample(self, *, not_before_ns, **_kwargs):
            if self._latest_pose_stamp <= not_before_ns:
                return False, "Waiting for new image"
            self._calibration_samples.append(self._latest_pose_stamp)
            return True, "Fresh test image"

    camera = Camera()
    controller = Node("calibration_protocol_test", context=context)
    client = controller.create_client(CaptureCalibration, "/camera_calibration/capture_automatic")
    history = []

    def execute(goal):
        result = ReplayCalibration.Result()
        for index in range(1, 6):
            for phase in (CaptureCalibration.Request.PREPARE, CaptureCalibration.Request.CAPTURE):
                request = CaptureCalibration.Request(
                    phase=phase, run_id=goal.request.run_id, position_index=index,
                    not_before=controller.get_clock().now().to_msg())
                future = client.call_async(request)
                deadline = time.monotonic() + 3.
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(.005)
                if not future.done() or not future.result().success:
                    goal.abort()
                    result.message = "Capture failed or executor stalled"
                    return result
                history.append((index, phase))
            result.captured_count = index
        result.outcome = result.SUCCESS
        goal.succeed()
        return result

    server = ActionServer(
        controller, ReplayCalibration, "/robot_controller/replay_calibration",
        execute_callback=execute, callback_group=ReentrantCallbackGroup())
    executors = [MultiThreadedExecutor(num_threads=2, context=context) for _ in range(2)]
    executors[0].add_node(camera)
    executors[1].add_node(controller)
    threads = [threading.Thread(target=executor.spin, daemon=True) for executor in executors]
    for thread in threads:
        thread.start()
    try:
        deadline = time.monotonic() + 5.
        while (not camera.automatic.client.server_is_ready() or camera.frames == 0
               or not client.service_is_ready()) and time.monotonic() < deadline:
            time.sleep(.01)
        assert camera.automatic.start()[0]
        deadline = time.monotonic() + 5.
        while camera.automatic.active and time.monotonic() < deadline:
            time.sleep(.01)
        assert camera.automatic.complete, camera.automatic.message
        assert len(camera._calibration_samples) == 5
        assert len(history) == 10 and camera.frames >= 5
    finally:
        for executor in executors:
            executor.shutdown(timeout_sec=1.)
        for thread in threads:
            thread.join(timeout=1.)
        server.destroy()
        camera.destroy_node()
        controller.destroy_node()
        rclpy.shutdown(context=context)
