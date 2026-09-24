"""Direct maintenance replay, fresh observations, cancellation and Save As."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock
import json
import threading
import time

import numpy as np
import pytest
from dobot_msgs_v4.msg import RobotStatus
from dobot_msgs_v4.srv import CP, MovJ, Stop
from rclpy.clock import ClockType
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import String

from camera_calibration_gui import automatic_capture as automatic_module
from camera_calibration_gui import maintenance_robot as robot_module
from camera_calibration_gui.automatic_capture import AutomaticCapture
from camera_calibration_gui.calibration_core import output_path_for_mode
from camera_calibration_gui.main import CalibrationNode, CalibrationWindow, TransformStamped
from camera_calibration_gui.maintenance_robot import (
    Cr10Model, MaintenanceRobot, RobotSample, arrived, idle, stationary)


def ros_time(seconds):
    return Time(seconds=seconds, clock_type=ClockType.ROS_TIME)


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(automatic_module, "MaintenanceRobot", MagicMock())
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


def pending_capture(node):
    request = dict(event=threading.Event(), success=False, message="", index=1,
                   not_before=ros_time(9.8).nanoseconds, joints=(0.,) * 6,
                   deadline=time.monotonic() + 10., last_stamp=None,
                   run_id=node.automatic.run_id)
    node.automatic.pending = request
    return request


def test_capture_collects_fresh_observation_and_preserves_recipe(node):
    recipe = node.automatic.recipe
    request = pending_capture(node)
    node.automatic._tick()
    assert request["event"].is_set() and request["success"]
    assert node.automatic.captured == 1
    assert len(node._calibration_samples) == 1
    assert node.automatic.recipe is recipe


def test_stop_cancels_pending_sample_and_sends_direct_stop(node):
    request = pending_capture(node)
    node.automatic.stop()
    node.automatic._tick()
    assert request["event"].is_set() and not request["success"]
    assert node._calibration_samples == []
    node.automatic.robot.request_stop.assert_called_once_with(fresh=True)


def test_old_frame_wait_is_bounded(node):
    request = pending_capture(node)
    node._latest_pose_stamp = ros_time(9.7).nanoseconds
    node.automatic._tick()
    assert not request["event"].is_set()
    request["deadline"] = 0.
    node.automatic._tick()
    assert request["event"].is_set() and not request["success"]
    assert "10 seconds" in request["message"]
    assert not node._calibration_samples


def test_rejected_start_preserves_loaded_samples(node):
    node.automatic.active = False
    original = [object()] * 5
    node._calibration_samples = original
    node.automatic.robot.prepare.side_effect = RuntimeError("Dobot unavailable")
    assert node.automatic.start() == (False, "Dobot unavailable")
    assert node._calibration_samples is original
    assert not node.automatic.started
    node.automatic.robot.command.assert_not_called()


def test_save_as_prefills_existing_rule_and_passes_edited_name_and_timestamp(node, monkeypatch):
    from camera_calibration_gui.main import QtWidgets

    node._calibration_mode = "camera_to_hand"
    node.save = MagicMock(return_value=(True, "Saved"))
    window = SimpleNamespace(_node=node, _show_save_result=MagicMock())
    dialog = MagicMock(return_value=("new_station.yaml", True))
    monkeypatch.setattr(QtWidgets.QInputDialog, "getText", dialog)
    CalibrationWindow._save_as(window)
    args, kwargs = node.save.call_args
    assert args == ("new_station.yaml",)
    expected = output_path_for_mode(node._calibration_mode, created_at=kwargs["created_at"])
    assert dialog.call_args.args[-1] == expected.name
    window._show_save_result.assert_called_once_with(True, "Saved")
    dialog.return_value = ("ignored.yaml", False)
    node.save.reset_mock()
    CalibrationWindow._save_as(window)
    node.save.assert_not_called()


@pytest.mark.parametrize("name", ["", "../source.yaml", "/tmp/source.yaml", "a\\b.yaml",
                                  ".hidden.yaml", "bad.txt", " space.yaml", "a\n.yaml"])
def test_save_as_cannot_escape_calibration_directory(tmp_path, name):
    with pytest.raises(ValueError):
        output_path_for_mode("camera_to_hand", root=tmp_path, filename=name)


def test_save_as_accepts_custom_basename_and_appends_yaml(tmp_path):
    assert output_path_for_mode("camera_to_hand", root=tmp_path, filename="new station") == (
        tmp_path / "calibration/new station.yaml")


def feedback(**changes):
    data = dict(robot_mode=5, controller_timer=1, currentCommandId=0, RunningStatus=0,
                isRunQueuedCmd=0, EnableStatus=1, ErrorStatus=0, CollisionStates=0,
                userCoordinate=0, toolCoordinate=0, digital_input_bits=0, digital_outputs=0,
                q_actual=[0.] * 6, tool_vector_actual=[0.] * 6)
    data.update(changes)
    return data


@pytest.fixture
def robot(monkeypatch):
    node = MagicMock()
    node._fatal_error = None
    result = MaintenanceRobot(node)
    monkeypatch.setattr(result, "check_owners", lambda: None)
    monkeypatch.setattr(result, "_service_owner", lambda _name: None)
    result._on_status(RobotStatus(is_connected=True, is_enable=True))
    result._on_feed(String(data=json.dumps(feedback())))
    result.output_bits = 0
    return result


@pytest.mark.parametrize("changes", [{"ErrorStatus": 1}, {"CollisionStates": 1},
                                     {"EnableStatus": 0}, {"userCoordinate": 1},
                                     {"toolCoordinate": 1}, {"digital_input_bits": 1},
                                     {"digital_outputs": 1}])
def test_guard_rejects_faults_and_changed_outputs(robot, changes):
    robot._on_feed(String(data=json.dumps(feedback(**changes))))
    with pytest.raises(RuntimeError):
        robot.guard()


def test_guard_rejects_stale_frozen_or_malformed_feedback(robot):
    robot.progress_time = time.monotonic() - 2.
    with pytest.raises(RuntimeError, match="not advancing"):
        robot.guard()
    robot._on_feed(String(data=json.dumps(feedback(controller_timer=2))))
    assert idle(robot.guard())
    robot._on_feed(String(data='{"robot_mode":5}'))
    with pytest.raises(RuntimeError, match="missing"):
        robot.guard()


def test_arrival_requires_tcp_joints_and_two_advancing_samples():
    first = RobotSample(1, feedback(), True)
    second = RobotSample(2, feedback(controller_timer=2), True)
    assert arrived(first, (0.,) * 6, np.eye(4)) and stationary(first, second)
    assert not stationary(first, first)
    assert not stationary(first, RobotSample(2, feedback(), True))
    wrong_tcp = RobotSample(2, feedback(tool_vector_actual=[10., 0., 0., 0., 0., 0.]), True)
    assert not arrived(wrong_tcp, (0.,) * 6, np.eye(4))
    assert not stationary(first, wrong_tcp)
    wrong_joints = RobotSample(2, feedback(q_actual=[2.] * 6), True)
    assert not arrived(wrong_joints, (0.,) * 6, np.eye(4))


def test_cr10_targets_validate_before_motion():
    model = Cr10Model(Path(__file__).resolve().parents[2]
                      / "DOBOT_6Axis_ROS2_V4/cra_description/urdf/cr10_robot.xacro")
    matrix = model.forward((0.,) * 6)
    assert np.allclose(matrix[3], [0., 0., 0., 1.])
    assert np.allclose(matrix[:3, :3] @ matrix[:3, :3].T, np.eye(3))
    for joints in ((0.,) * 5, (float("nan"),) * 6, (100.,) * 6):
        with pytest.raises(ValueError):
            model.forward(joints)


def test_late_motion_acceptance_is_stopped_and_previous_response_blocks(robot):
    future = Future()
    robot.clients["MovJ"].call_async.return_value = future
    robot.cancel.set()
    record = dict(name="MovJ", future=future, abandoned=True)
    robot.pending.append(record)
    robot.request_stop = MagicMock()
    future.set_result(MovJ.Response(res=0, robot_return="{2}"))
    robot._late_reply(record)
    robot._late_reply(record)
    robot.request_stop.assert_called_once_with(fresh=True)
    robot.cancel.clear()
    robot.pending.append(dict(name="CP", future=Future()))
    with pytest.raises(RuntimeError, match="unanswered"):
        robot.command("CP", CP.Request(r=100))


def test_new_explicit_stop_is_independent_of_old_result_and_confirms_physically(robot):
    first, second = Future(), Future()
    robot.clients["Stop"].call_async.side_effect = [first, second]
    robot.request_stop()
    robot.request_stop(fresh=True)
    first.set_result(Stop.Response(res=0))
    robot._check_stop()
    assert "accepted_sequence" not in robot.stop_attempt
    second.set_result(Stop.Response(res=0))
    robot._check_stop()
    assert not robot.stop_attempt.get("confirmed")
    for tick in (2, 3):
        robot._on_feed(String(data=json.dumps(feedback(controller_timer=tick))))
        robot._check_stop()
    assert robot.stop_attempt["confirmed"]


def test_motion_cannot_finish_on_old_queue_id_or_unchanged_feedback(robot, monkeypatch):
    robot.command = MagicMock(return_value=MovJ.Response(res=0, robot_return="{7}"))
    ticks = iter([
        RobotSample(1, feedback(), True),  # Initial and command acceptance.
        RobotSample(1, feedback(), True),
        RobotSample(2, feedback(controller_timer=2), True),  # Old queue ID.
        RobotSample(3, feedback(controller_timer=3, currentCommandId=7), True),
        RobotSample(3, feedback(controller_timer=3, currentCommandId=7), True),
        RobotSample(4, feedback(controller_timer=4, currentCommandId=7), True),
    ])
    robot.guard = lambda: next(ticks)
    sample = robot.move((0.,) * 6, np.eye(4), monitor=lambda: None)
    assert sample.sequence == 4
    request = robot.command.call_args.args[1]
    assert request.mode and request.param_value == ["user=0", "tool=0", "v=20", "a=20"]


@pytest.mark.parametrize("stop_during_solve", [False, True])
def test_real_ros_direct_replay_and_stop_with_two_camera_threads(monkeypatch, stop_during_solve):
    """Isolated fake Dobot services: no controller, camera hardware or robot connection."""
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    from camera_calibration_gui.calibration_core import rotation_matrix_to_rpy_deg

    monkeypatch.setenv("ROS_DOMAIN_ID", "232")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setattr(robot_module, "configured_bringup", lambda _root: "calibration_test_dobot")
    context = Context()
    rclpy.init(context=context)
    model = Cr10Model(Path(__file__).resolve().parents[2]
                      / "DOBOT_6Axis_ROS2_V4/cra_description/urdf/cr10_robot.xacro")

    class Camera(Node):
        def __init__(self):
            super().__init__("camera_calibration", context=context)
            self._lock = threading.RLock()
            self._event_logger = MagicMock()
            self._fatal_error = None
            self._calibration_samples = []
            self._camera_matrix = np.eye(3)
            self._latest_pose_stamp = self._latest_valid_rgb_stamp = None
            self.automatic = AutomaticCapture(self)
            self.automatic.recipe = SimpleNamespace(samples=[SimpleNamespace(
                joint_positions_rad=(index * .01, 0., 0., 0., 0., 0.)) for index in range(6)])
            self.solving = threading.Event()
            self.frames = 0
            self.create_timer(.01, self.sensor)

        def sensor(self):
            self.frames += 1
            self._latest_pose_stamp = self.get_clock().now().nanoseconds
            self._latest_valid_rgb_stamp = self._latest_pose_stamp

        def reset_samples(self, **_kwargs):
            self._calibration_samples.clear()

        def capture_sample(self, *, not_before_ns, **_kwargs):
            if self._latest_pose_stamp <= not_before_ns:
                return False, "Waiting for post-arrival image"
            self._calibration_samples.append(self._latest_pose_stamp)
            if len(self._calibration_samples) == 5:
                self.solving.set()
                time.sleep(.65)  # Serial image callbacks pause; robot feedback must keep running.
            return True, "Fresh sample"

    class Dobot(Node):
        def __init__(self):
            super().__init__("calibration_test_dobot", context=context)
            self.group = ReentrantCallbackGroup()
            self.data = feedback()
            self.cp = []
            self.moves = []
            self.stops = 0
            self.lock = threading.Lock()
            self.feed_pub = self.create_publisher(String, robot_module.FEED_TOPIC, 10)
            self.status_pub = self.create_publisher(RobotStatus, robot_module.STATUS_TOPIC, 10)
            self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
            for kind, callback in ((CP, self.cp_command), (MovJ, self.move), (Stop, self.stop)):
                self.create_service(kind, robot_module.SERVICE_PREFIX + kind.__name__, callback,
                                    callback_group=self.group)
            self.create_timer(.02, self.publish, callback_group=self.group)

        def cp_command(self, request, response):
            self.cp.append(request.r)
            response.res = 0
            return response

        def move(self, request, response):
            with self.lock:
                self.moves.append(request)
                joints = [getattr(request, key) for key in "abcdef"]
                matrix = model.forward(np.deg2rad(joints))
                self.data.update(
                    currentCommandId=len(self.moves), q_actual=joints,
                    tool_vector_actual=list(matrix[:3, 3] * 1000.)
                    + list(rotation_matrix_to_rpy_deg(matrix[:3, :3])))
                response.res = 0
                response.robot_return = "{" + str(len(self.moves)) + "}"
            return response

        def stop(self, _request, response):
            self.stops += 1
            response.res = 0
            return response

        def publish(self):
            with self.lock:
                self.data["controller_timer"] += 1
                self.feed_pub.publish(String(data=json.dumps(self.data)))
            self.status_pub.publish(RobotStatus(is_connected=True, is_enable=True))
            self.joint_pub.publish(JointState())

    camera, dobot = Camera(), Dobot()
    executors = [MultiThreadedExecutor(num_threads=2, context=context) for _ in range(2)]
    executors[0].add_node(camera)
    executors[1].add_node(dobot)
    threads = [threading.Thread(target=executor.spin, daemon=True) for executor in executors]
    for thread in threads:
        thread.start()
    try:
        deadline = time.monotonic() + 5.
        while (not all(client.service_is_ready()
                       for client in camera.automatic.robot.clients.values())
               or camera.automatic.robot.feed is None or camera.automatic.robot.status is None
               or camera.frames == 0) and time.monotonic() < deadline:
            time.sleep(.01)
        success, message = camera.automatic.start()
        assert success, message
        if stop_during_solve:
            assert camera.solving.wait(4.), camera.automatic.message
            before = camera.automatic.robot.sequence
            camera.automatic.stop()
            time.sleep(.1)
            assert dobot.stops >= 1
            assert camera.automatic.robot.sequence > before
        deadline = time.monotonic() + 5.
        while camera.automatic.active and time.monotonic() < deadline:
            time.sleep(.01)
        assert not camera.automatic.active, camera.automatic.message
        assert camera.automatic.complete is not stop_during_solve, camera.automatic.message
        assert len(dobot.moves) == (5 if stop_during_solve else 6)
        assert dobot.cp == [100]
        for index, request in enumerate(dobot.moves):
            assert request.mode and request.a == pytest.approx(np.rad2deg(index * .01))
        if stop_during_solve:
            assert camera.automatic.robot.stop_attempt["confirmed"]
        else:
            assert len(camera._calibration_samples) == 6 and dobot.stops == 0
    finally:
        camera.automatic.close()
        for executor in executors:
            executor.shutdown(timeout_sec=2.)
        for thread in threads:
            thread.join(timeout=1.)
        camera.destroy_node()
        dobot.destroy_node()
        rclpy.shutdown(context=context)


@pytest.mark.parametrize("bad_frame", ["malformed", "clock_reset", "transient_fault"])
def test_feedback_failure_stays_latched_until_another_explicit_start(robot, bad_frame):
    robot.watching = True
    if bad_frame == "malformed":
        message = "invalid JSON"
    else:
        message = json.dumps(feedback(**(
            {"controller_timer": 0} if bad_frame == "clock_reset" else {"ErrorStatus": 1})))
    robot._on_feed(String(data=message))
    robot._on_feed(String(data=json.dumps(feedback(controller_timer=2))))
    with pytest.raises(RuntimeError):
        robot.guard()


def test_response_timeout_does_not_retry_or_allow_another_command(robot, monkeypatch):
    monkeypatch.setattr(robot_module, "RESPONSE_TIMEOUT", 0.)
    robot.clients["MovJ"].call_async.return_value = Future()
    with pytest.raises(RuntimeError, match="timed out"):
        robot.command("MovJ", MovJ.Request())
    with pytest.raises(RuntimeError, match="unanswered"):
        robot.command("CP", CP.Request(r=100))
    robot.clients["MovJ"].call_async.assert_called_once()


def test_competing_command_apps_or_publishers_block_replay(robot):
    node = robot.node
    robot.bringup = "dobot"
    node.get_name.return_value = "camera_calibration"
    node.get_namespace.return_value = "/"
    node.get_node_names_and_namespaces.return_value = [
        ("camera_calibration", "/"), ("dobot", "/"), ("robot_controller", "/")]
    with pytest.raises(RuntimeError, match="competing"):
        MaintenanceRobot.check_owners(robot)
    node.get_node_names_and_namespaces.return_value.pop()
    node.get_publishers_info_by_topic.return_value = []
    with pytest.raises(RuntimeError, match="sole canonical publisher"):
        MaintenanceRobot.check_owners(robot)
    publisher = SimpleNamespace(node_name="dobot", node_namespace="/")
    node.get_publishers_info_by_topic.return_value = [publisher]
    MaintenanceRobot.check_owners(robot)
    node.get_publishers_info_by_topic.return_value = [publisher, publisher]
    with pytest.raises(RuntimeError, match="sole canonical publisher"):
        MaintenanceRobot.check_owners(robot)


def test_late_stop_keeps_gui_stop_available_after_operation_thread_exits(node):
    node.automatic.active = False
    node.automatic.robot.stop_attempt = {"future": Future(), "deadline": time.monotonic() + 5.}
    node.automatic._tick()
    assert node.automatic.stopping and node._automatic_busy()
    node.automatic.robot.stop_attempt["confirmed"] = True
    node.automatic._tick()
    assert not node.automatic.stopping
