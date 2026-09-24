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
from dobot_msgs_v4.srv import CP, EnableRobot, MovJ, Stop, StopDrag
from rclpy.clock import ClockType
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import String

from camera_calibration_gui import automatic_capture as automatic_module
from camera_calibration_gui import maintenance_robot as robot_module
from camera_calibration_gui.automatic_capture import AutomaticCapture
from camera_calibration_gui.calibration_core import JOINT_NAMES, output_path_for_mode
from camera_calibration_gui.main import CalibrationNode, CalibrationWindow, TransformStamped
from camera_calibration_gui.maintenance_robot import (
    Cr10JointLimits, MaintenanceRobot, RobotSample, idle, joints_at_target, stationary)


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
    request = dict(event=threading.Event(), success=False, message="", index=1, attempt=1,
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
    node.create_client.side_effect = lambda *_args, **_kwargs: MagicMock()
    node._latest_joint_positions = (0.,) * 6
    node._latest_joint_state_stamp = ros_time(9.9)
    node.get_clock.return_value.now.return_value = ros_time(10.)
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


def test_saved_joints_ignore_absolute_tcp_but_stability_detects_motion():
    first = RobotSample(1, feedback(), True)
    second = RobotSample(2, feedback(controller_timer=2), True)
    assert joints_at_target(first, (0.,) * 6) and stationary(first, second)
    assert not stationary(first, first)
    assert not stationary(first, RobotSample(2, feedback(), True))
    wrong_tcp = RobotSample(2, feedback(tool_vector_actual=[10., 0., 0., 0., 0., 0.]), True)
    assert joints_at_target(wrong_tcp, (0.,) * 6)
    assert not stationary(first, wrong_tcp)
    wrong_joints = RobotSample(2, feedback(q_actual=[2.] * 6), True)
    assert not joints_at_target(wrong_joints, (0.,) * 6)


def test_cr10_targets_validate_before_motion():
    limits = Cr10JointLimits(Path(__file__).resolve().parents[2]
                             / "DOBOT_6Axis_ROS2_V4/cra_description/urdf/cr10_robot.xacro")
    assert limits.validate((0.,) * 6) == (0.,) * 6
    for joints in ((0.,) * 5, (float("nan"),) * 6, (100.,) * 6):
        with pytest.raises(ValueError):
            limits.validate(joints)


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
    sample = robot.move((0.,) * 6, monitor=lambda: None)
    assert sample.sequence == 4
    request = robot.command.call_args.args[1]
    assert request.mode and request.param_value == ["user=0", "tool=0", "v=20", "a=20"]


@pytest.mark.parametrize("scenario", ["normal", "stop_during_solve", "camera_gap",
                                      "disabled", "drag_disabled", "drag_enabled"])
def test_real_ros_direct_replay_and_stop_with_two_camera_threads(monkeypatch, scenario):
    """Isolated fake Dobot services: no controller, camera hardware or robot connection."""
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    monkeypatch.setenv("ROS_DOMAIN_ID", "232")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setattr(robot_module, "configured_bringup", lambda _root: "calibration_test_dobot")
    context = Context()
    rclpy.init(context=context)
    stop_during_solve = scenario == "stop_during_solve"

    class Camera(Node):
        def __init__(self):
            super().__init__("camera_calibration", context=context)
            self._lock = threading.RLock()
            self._event_logger = MagicMock()
            self._fatal_error = None
            self._calibration_samples = []
            self._camera_matrix = np.eye(3)
            self._latest_pose_stamp = self._latest_valid_rgb_stamp = None
            self._latest_joint_positions = self._latest_joint_state_stamp = None
            self.create_subscription(JointState, "/joint_states",
                                     lambda msg: CalibrationNode._on_joint_state(self, msg), 10)
            self.automatic = AutomaticCapture(self)
            self.automatic.recipe = SimpleNamespace(samples=[SimpleNamespace(
                joint_positions_rad=(index * .01, 0., 0., 0., 0., 0.)) for index in range(6)])
            self.solving = threading.Event()
            self.frames = 0
            self.gap_until = 0.
            self.create_timer(.01, self.sensor)

        def sensor(self):
            if time.monotonic() < self.gap_until:
                return
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
            if scenario == "disabled":
                self.data.update(robot_mode=4, EnableStatus=0)
            elif scenario.startswith("drag_"):
                self.data.update(robot_mode=6)
            self.startup_commands = []
            self.cp = []
            self.moves = []
            self.stops = 0
            self.moving_until = 0.
            self.lock = threading.Lock()
            self.feed_pub = self.create_publisher(String, robot_module.FEED_TOPIC, 10)
            self.status_pub = self.create_publisher(RobotStatus, robot_module.STATUS_TOPIC, 10)
            self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
            for kind, callback in ((CP, self.cp_command), (MovJ, self.move), (Stop, self.stop),
                                   (StopDrag, self.stop_drag), (EnableRobot, self.enable)):
                self.create_service(kind, robot_module.SERVICE_PREFIX + kind.__name__, callback,
                                    callback_group=self.group)
            self.create_timer(.02, self.publish, callback_group=self.group)

        def cp_command(self, request, response):
            assert self.data["robot_mode"] == 5 and self.data["EnableStatus"] == 1
            self.cp.append(request.r)
            response.res = 0
            return response

        def stop_drag(self, _request, response):
            with self.lock:
                assert self.data["robot_mode"] == 6
                self.startup_commands.append("StopDrag")
                disabled = scenario == "drag_disabled"
                self.data.update(robot_mode=4 if disabled else 5, EnableStatus=int(not disabled))
            response.res = 0
            return response

        def enable(self, _request, response):
            with self.lock:
                assert self.data["robot_mode"] == 4
                self.startup_commands.append("EnableRobot")
                self.data.update(robot_mode=5, EnableStatus=1)
            response.res = 0
            return response

        def move(self, request, response):
            with self.lock:
                self.moves.append(request)
                joints = [getattr(request, key) for key in "abcdef"]
                self.data.update(
                    currentCommandId=len(self.moves), q_actual=joints,
                    tool_vector_actual=[747.562 + joints[0], 219.073, 159.800,
                                        -175.635, 11.458, -108.158])
                if scenario == "camera_gap" and len(self.moves) == 2:
                    camera.gap_until = time.monotonic() + .9
                    self.moving_until = time.monotonic() + 1.5
                    self.data.update(robot_mode=7, RunningStatus=1, isRunQueuedCmd=1)
                response.res = 0
                response.robot_return = "{" + str(len(self.moves)) + "}"
            return response

        def stop(self, _request, response):
            with self.lock:
                self.stops += 1
                self.moving_until = 0.
                self.data.update(robot_mode=5, RunningStatus=0, isRunQueuedCmd=0)
            response.res = 0
            return response

        def publish(self):
            with self.lock:
                if self.moving_until and time.monotonic() >= self.moving_until:
                    self.data.update(robot_mode=5, RunningStatus=0, isRunQueuedCmd=0)
                    self.moving_until = 0.
                self.data["controller_timer"] += 1
                self.feed_pub.publish(String(data=json.dumps(self.data)))
            self.status_pub.publish(RobotStatus(
                is_connected=True, is_enable=self.data["robot_mode"] == 5))
            joints = JointState()
            joints.header.stamp = self.get_clock().now().to_msg()
            joints.name = list(JOINT_NAMES)
            joints.position = list(np.deg2rad(self.data["q_actual"]))
            self.joint_pub.publish(joints)

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
               or camera.frames == 0 or camera._latest_joint_positions is None
               ) and time.monotonic() < deadline:
            time.sleep(.01)
        success, message = camera.automatic.start()
        assert success, message
        if stop_during_solve:
            assert camera.solving.wait(10.), camera.automatic.message
            before = camera.automatic.robot.sequence
            camera.automatic.stop()
            time.sleep(.1)
            assert dobot.stops >= 1
            assert camera.automatic.robot.sequence > before
        deadline = time.monotonic() + 15.
        while camera.automatic.active and time.monotonic() < deadline:
            time.sleep(.01)
        assert not camera.automatic.active, camera.automatic.message
        assert camera.automatic.complete is not stop_during_solve, camera.automatic.message
        expected_positions = ([0, 1, 1, 2, 3, 4, 5] if scenario == "camera_gap"
                              else list(range(5 if stop_during_solve else 6)))
        assert len(dobot.moves) == len(expected_positions)
        assert dobot.cp == [100]
        expected_startup = {"disabled": ["EnableRobot"], "drag_enabled": ["StopDrag"],
                            "drag_disabled": ["StopDrag", "EnableRobot"]}
        assert dobot.startup_commands == expected_startup.get(scenario, [])
        for index, request in zip(expected_positions, dobot.moves):
            assert request.mode and request.a == pytest.approx(np.rad2deg(index * .01))
        if stop_during_solve:
            assert camera.automatic.robot.stop_attempt["confirmed"]
        else:
            assert len(camera._calibration_samples) == 6
            assert dobot.stops == (1 if scenario == "camera_gap" else 0)
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


@pytest.mark.parametrize("continue_batch", [True, False])
def test_only_three_failed_captures_prompt_and_continue_preserves_samples(node, continue_batch):
    automatic = node.automatic
    automatic.robot.operator_cancel = threading.Event()
    calls = []

    def guard(*_args):
        if automatic.robot.operator_cancel.is_set():
            raise robot_module.ReplayStopped("Operator stopped")

    automatic.robot.guard.side_effect = guard
    automatic.robot.hold.side_effect = guard

    def capture(index, attempt, *_args):
        calls.append((index, attempt))
        if index == 2 and calls.count((2, 1)) < 2:
            assert automatic.retry_prompt is None
            return dict(success=False, retryable=True, message="Board obscured")
        node._calibration_samples.append(index)
        automatic.captured += 1
        return dict(success=True)

    automatic._stable_capture = capture
    worker = threading.Thread(target=automatic._run, args=([(0.,) * 6] * 5,))
    worker.start()
    try:
        deadline = time.monotonic() + 2.
        while automatic.retry_prompt is None and worker.is_alive() and time.monotonic() < deadline:
            time.sleep(.005)
        assert automatic.retry_prompt is not None, automatic.message
        assert calls == [(1, 1), (2, 1), (2, 2), (2, 3)]
        assert node._calibration_samples == [1]
        assert automatic.robot.move.call_count == 2
        time.sleep(.04)
        assert len(calls) == 4  # No fourth attempt without explicit Continue.
        token = automatic.retry_prompt["token"]
        automatic.respond_retry("stale prompt", True)
        assert not automatic.retry_prompt["event"].is_set()
        automatic.respond_retry(token, continue_batch)
        worker.join(timeout=2.)
        assert not worker.is_alive()
        assert automatic.complete is continue_batch, automatic.message
        assert automatic.robot.move.call_count == (5 if continue_batch else 2)
        assert node._calibration_samples == ([1, 2, 3, 4, 5] if continue_batch else [1])
        if continue_batch:
            assert calls[4] == (2, 1)
    finally:
        automatic.stop()
        worker.join(timeout=2.)


def test_three_motion_timeouts_each_confirm_stop_before_retry_or_prompt(node):
    automatic = node.automatic
    attempts = []

    def move(*_args, **_kwargs):
        attempts.append("move")
        if len(attempts) <= 3:
            raise robot_module.MotionArrivalTimeout("Wrong endpoint")
        return object()

    automatic.robot.move.side_effect = move
    automatic.robot.stop_attempt = dict(confirmed=True)

    def capture(*_args):
        automatic.captured += 1
        return dict(success=True)

    automatic._stable_capture = capture
    worker = threading.Thread(target=automatic._run, args=([(0.,) * 6],))
    worker.start()
    try:
        deadline = time.monotonic() + 2.
        while automatic.retry_prompt is None and worker.is_alive() and time.monotonic() < deadline:
            time.sleep(.005)
        assert automatic.retry_prompt is not None, automatic.message
        assert automatic.retry_prompt["phase"] == "motion"
        assert len(attempts) == 3
        assert automatic.robot.wait_stop.call_count == 3
        assert automatic.robot.resume_after_stop.call_count == 2
        automatic.respond_retry(automatic.retry_prompt["token"], True)
        worker.join(timeout=2.)
        assert not worker.is_alive() and automatic.complete, automatic.message
        assert len(attempts) == 4
        assert automatic.robot.resume_after_stop.call_count == 3
    finally:
        automatic.stop()
        worker.join(timeout=2.)


def test_stability_requires_full_second_and_advancing_feedback(node, monkeypatch):
    clock = SimpleNamespace(now=0.)
    monkeypatch.setattr(automatic_module, "time", SimpleNamespace(monotonic=lambda: clock.now))
    calls = []
    anchor = RobotSample(1, feedback(), True)

    def hold(*_args):
        calls.append(clock.now)
        # At exactly one second, repeat an old sample; it cannot finish the hold.
        if clock.now == 1.:
            return anchor
        return RobotSample(len(calls) + 1, feedback(controller_timer=len(calls) + 1), True)

    node.automatic.robot.hold.side_effect = hold
    node.automatic.robot.cancel.wait.side_effect = (
        lambda _dt: setattr(clock, "now", clock.now + .25))
    node.automatic._wait_stable(anchor, (0.,) * 6)
    assert clock.now == 1.25 and calls[0] == 0.
    assert len(calls) == 6
    assert node.automatic.pending is None


def test_movement_during_stability_aborts_without_a_capture(node):
    node.automatic.robot.hold.side_effect = RuntimeError("Robot moved")
    with pytest.raises(RuntimeError, match="Robot moved"):
        node.automatic._stable_capture(1, 1, object(), (0.,) * 6)
    assert node.automatic.pending is None and not node._calibration_samples


@pytest.mark.parametrize("blocker", ["operator_stop", "new_stop", "unanswered", "fault"])
def test_motion_retry_cannot_clear_stop_or_fault_guards(robot, blocker):
    attempt = dict(confirmed=True)
    robot.stop_attempt = attempt
    robot.cancel.set()
    if blocker == "operator_stop":
        robot.operator_cancel.set()
    elif blocker == "new_stop":
        robot.stop_attempt = dict(confirmed=True)
    elif blocker == "unanswered":
        robot.pending.append(dict(future=Future()))
    else:
        robot.feedback_failure = "Robot fault"
    with pytest.raises(RuntimeError):
        robot.resume_after_stop(attempt)
    assert robot.cancel.is_set()


def test_confirmed_motion_retry_resumes_without_resetting_outputs(robot):
    attempt = dict(confirmed=True)
    robot.stop_attempt = attempt
    robot.cancel.set()
    robot.resume_after_stop(attempt)
    assert not robot.cancel.is_set() and robot.stop_attempt is None
    assert robot.output_bits == 0
    for client in robot.clients.values():
        client.call_async.assert_not_called()


def test_arrival_timeout_reports_which_robot_gates_failed(robot, monkeypatch):
    monkeypatch.setattr(robot_module, "MOTION_PROGRESS_TIMEOUT", 0.)
    robot.command = MagicMock(return_value=MovJ.Response(res=0, robot_return="{7}"))
    with pytest.raises(robot_module.MotionArrivalTimeout, match="queue ID 0/7.*joint error"):
        robot.move((0.,) * 6, monitor=lambda: None)


@pytest.mark.parametrize("answer", ["continue", "stop", "close"])
def test_retry_prompt_is_nonmodal_and_close_means_stop(node, answer):
    from camera_calibration_gui.main import QtCore, QtWidgets

    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    node.load_last_session = lambda: None
    window = CalibrationWindow(node)
    window._timer.stop()
    prompt = dict(token="retry", event=threading.Event(), accepted=False, index=2, attempt=3,
                  phase="capture", reason="Board obscured")
    node.automatic.retry_prompt = prompt
    try:
        window._refresh_retry_prompt()
        dialog = window._retry_dialog[1]
        assert dialog.windowModality() == QtCore.Qt.NonModal
        assert "all three attempts failed" in dialog.text()
        if answer == "close":
            dialog.close()
        else:
            dialog.button(QtWidgets.QMessageBox.Yes if answer == "continue"
                          else QtWidgets.QMessageBox.No).click()
        application.processEvents()
        assert prompt["event"].is_set()
        assert prompt["accepted"] is (answer == "continue")
        window._refresh_retry_prompt()
        assert window._retry_dialog is None
        if answer != "continue":
            node.automatic.robot.request_stop.assert_called_once_with(fresh=True)
    finally:
        node.automatic.retry_prompt = None
        window._refresh_retry_prompt()
        window.close()
        application.processEvents()


@pytest.mark.parametrize("failure", ["no_info", "no_image", "stale", "future"])
def test_camera_readiness_reason_is_specific_and_recoverable(node, failure):
    if failure == "no_info":
        node._camera_matrix = None
        expected = "CameraInfo"
    elif failure == "no_image":
        node._latest_valid_rgb_stamp = None
        expected = "valid RGB input"
    else:
        node._latest_valid_rgb_stamp = ros_time(9. if failure == "stale" else 11.).nanoseconds
        expected = "input age"
    with pytest.raises(automatic_module.CameraNotReady, match=expected):
        node.automatic._require_camera()


def test_native_worker_failure_is_not_a_recoverable_camera_input_condition(node):
    node._fatal_error = "Native worker exited"
    with pytest.raises(RuntimeError, match="Native worker") as failure:
        node.automatic._require_camera()
    assert not isinstance(failure.value, automatic_module.CameraNotReady)


@pytest.mark.parametrize("phase", ["before_motion", "during_motion", "after_motion_retry"])
def test_all_camera_readiness_paths_reach_three_attempt_prompt(node, phase):
    automatic = node.automatic
    automatic.robot.operator_cancel = threading.Event()
    checks, moves = [], []

    def monitor(*_args):
        if automatic.robot.operator_cancel.is_set():
            raise robot_module.ReplayStopped("Operator stopped")

    automatic.robot.guard.side_effect = monitor
    automatic.robot.hold_idle.side_effect = monitor
    automatic.robot.hold_stopped.side_effect = monitor

    def wait_camera(_monitor):
        checks.append(True)
        if phase == "before_motion" and len(checks) <= 3:
            raise automatic_module.CameraNotReady("CameraInfo missing")
        if phase == "after_motion_retry" and 2 <= len(checks) <= 3:
            raise automatic_module.CameraNotReady("RGB input aged")

    def move(*_args, **_kwargs):
        moves.append(True)
        if phase == "during_motion" and len(moves) <= 3:
            raise automatic_module.CameraNotReady("RGB input aged")
        if phase == "after_motion_retry" and len(moves) == 1:
            raise robot_module.MotionArrivalTimeout("Queue ID not confirmed")
        return object()

    def capture(*_args):
        automatic.captured += 1
        return dict(success=True)

    automatic._wait_camera = wait_camera
    automatic.robot.move.side_effect = move
    automatic._stable_capture = capture
    worker = threading.Thread(target=automatic._run, args=([(0.,) * 6],))
    worker.start()
    try:
        deadline = time.monotonic() + 2.
        while automatic.retry_prompt is None and worker.is_alive() and time.monotonic() < deadline:
            time.sleep(.005)
        assert automatic.retry_prompt is not None, automatic.message
        assert automatic.retry_prompt["phase"] == "camera"
        assert automatic.retry_prompt["attempt"] == 3
        assert automatic.active and not automatic.complete
        assert len(checks) == 3
        expected_moves = {"before_motion": 0, "during_motion": 3, "after_motion_retry": 1}
        assert len(moves) == expected_moves[phase]
        assert automatic.robot.wait_stop.call_count == len(moves)
        time.sleep(.04)
        assert len(checks) == 3
        automatic.respond_retry(automatic.retry_prompt["token"], True)
        worker.join(timeout=2.)
        assert not worker.is_alive() and automatic.complete, automatic.message
        assert len(checks) == 4 and automatic.captured == 1
    finally:
        automatic.stop()
        worker.join(timeout=2.)


def test_transient_camera_gap_recovers_within_same_attempt(node, monkeypatch):
    clock = SimpleNamespace(now=0.)
    monkeypatch.setattr(automatic_module, "time", SimpleNamespace(monotonic=lambda: clock.now))
    node._latest_valid_rgb_stamp = None

    def wait(_dt):
        clock.now += .1
        if clock.now >= .3:
            node._latest_valid_rgb_stamp = ros_time(9.9).nanoseconds

    node.automatic.robot.cancel.wait.side_effect = wait
    monitor = MagicMock()
    node.automatic._wait_camera(monitor)
    assert .3 <= clock.now < 2.
    assert node.automatic.retry_prompt is None
    assert monitor.call_count >= 4
    node.automatic.robot.move.assert_not_called()


def readiness_rig(robot, monkeypatch, mode, drag_exit=5):
    """Advance real guarded readiness through fake replies and canonical feedback."""
    rig = SimpleNamespace(now=time.monotonic(), calls=[], behavior={}, futures={},
                          data=feedback(robot_mode=mode, EnableStatus=int(mode != 4)))
    monkeypatch.setattr(robot_module, "time", SimpleNamespace(monotonic=lambda: rig.now))
    monkeypatch.setattr(robot_module, "configured_bringup", lambda _root: "dobot")

    def publish():
        rig.data["controller_timer"] += 1
        robot._on_status(RobotStatus(is_connected=True, is_enable=rig.data["robot_mode"] == 5))
        robot._on_feed(String(data=json.dumps(rig.data)))

    def advance(_duration):
        rig.now += .1
        publish()

    def send(name, kind, _request):
        rig.calls.append(name)
        future = Future()
        rig.futures[name] = future
        behavior = rig.behavior.get(name, "success")
        if behavior == "unanswered":
            return future
        future.set_result(kind.Response(res=-1 if behavior == "rejected" else 0))
        if behavior == "success":
            target_mode = drag_exit if name == "StopDrag" else 5
            rig.data.update(robot_mode=target_mode, EnableStatus=int(target_mode != 4))
        return future

    for name, kind in (("StopDrag", StopDrag), ("EnableRobot", EnableRobot)):
        robot.clients[name].call_async.side_effect = (
            lambda request, name=name, kind=kind: send(name, kind, request))
    monkeypatch.setattr(robot.cancel, "wait", advance)
    rig.publish, rig.advance = publish, advance
    rig.samples = [SimpleNamespace(joint_positions_rad=(0.,) * 6) for _ in range(5)]
    publish()
    return rig


@pytest.mark.parametrize("mode,drag_exit,expected", [
    (5, 5, []), (4, 5, ["EnableRobot"]), (6, 5, ["StopDrag"]),
    (6, 4, ["StopDrag", "EnableRobot"])])
def test_robot_prepares_itself_once_and_requires_advancing_idle_feedback(
        robot, monkeypatch, mode, drag_exit, expected):
    rig = readiness_rig(robot, monkeypatch, mode, drag_exit)
    targets = robot.prepare(rig.samples)
    assert not rig.calls  # Preflight and construction themselves never command hardware.
    assert targets == [(0.,) * 6] * 5
    with pytest.raises(RuntimeError, match="readiness state"):
        robot.command("CP", CP.Request(r=100))
    initial_sequence = robot.sequence
    robot.ensure_ready(MagicMock())
    assert rig.calls == expected and idle(robot.guard())
    assert robot.sequence >= initial_sequence + 2
    assert not robot.startup_modes
    for name, kind in (("EnableRobot", EnableRobot), ("StopDrag", StopDrag)):
        with pytest.raises(RuntimeError, match="explicit Start"):
            robot.command(name, kind.Request())
    robot.clients["MovJ"].call_async.assert_not_called()
    assert robot.output_bits == 0


@pytest.mark.parametrize("failure", ["missing", "stale", "future", "invalid", "zero_stamp"])
def test_readiness_requires_valid_live_joint_stream_before_any_command(robot, monkeypatch, failure):
    rig = readiness_rig(robot, monkeypatch, 4)
    if failure == "missing":
        robot.node._latest_joint_positions = None
    elif failure == "invalid":
        robot.node._latest_joint_positions = (float("nan"),) * 6
    else:
        seconds = {"stale": 8., "future": 11., "zero_stamp": 0.}[failure]
        robot.node._latest_joint_state_stamp = ros_time(seconds)
    with pytest.raises(RuntimeError, match="joint_states"):
        robot.prepare(rig.samples)
    assert not rig.calls and not robot.watching


@pytest.mark.parametrize("changes", [
    {"ErrorStatus": 1}, {"CollisionStates": 1}, {"robot_mode": 7}, {"robot_mode": 10},
    {"RunningStatus": 1}, {"isRunQueuedCmd": 1}, {"userCoordinate": 1},
    {"toolCoordinate": 1}, {"digital_input_bits": 1}, {"digital_outputs": 4097}])
def test_readiness_rejects_faults_motion_context_and_unsafe_io(robot, monkeypatch, changes):
    rig = readiness_rig(robot, monkeypatch, 4)
    rig.data.update(changes)
    rig.publish()
    with pytest.raises(RuntimeError):
        robot.prepare(rig.samples)
    assert not rig.calls and not robot.watching


@pytest.mark.parametrize("name,mode", [("EnableRobot", 4), ("StopDrag", 6)])
@pytest.mark.parametrize("behavior", ["rejected", "unanswered", "no_transition"])
def test_failed_readiness_stops_without_retry_settings_or_motion(
        node, robot, monkeypatch, name, mode, behavior):
    rig = readiness_rig(robot, monkeypatch, mode)
    rig.behavior[name] = behavior
    targets = robot.prepare(rig.samples)
    robot.request_stop = MagicMock()
    robot.wait_stop = MagicMock()
    node.automatic.robot = robot
    node.automatic._run(targets)
    assert rig.calls == [name]
    assert not node.automatic.complete and not node.automatic.active
    assert "Stop confirmed" in node.automatic.message
    robot.request_stop.assert_called_once()
    robot.clients["CP"].call_async.assert_not_called()
    robot.clients["MovJ"].call_async.assert_not_called()


@pytest.mark.parametrize("name,mode", [("EnableRobot", 4), ("StopDrag", 6)])
@pytest.mark.parametrize("when", ["response", "confirmation"])
def test_operator_stop_preempts_readiness_and_late_acceptance_is_contained(
        node, robot, monkeypatch, name, mode, when):
    rig = readiness_rig(robot, monkeypatch, mode)
    if when == "response":
        rig.behavior[name] = "unanswered"
    targets = robot.prepare(rig.samples)
    robot.request_stop = MagicMock()
    robot.wait_stop = MagicMock()

    def stop(_duration):
        rig.advance(.1)
        robot.operator_cancel.set()

    monkeypatch.setattr(robot.cancel, "wait", stop)
    node.automatic.robot = robot
    node.automatic._run(targets)
    assert rig.calls == [name] and not node.automatic.complete
    robot.clients["CP"].call_async.assert_not_called()
    robot.clients["MovJ"].call_async.assert_not_called()
    if when == "response":
        kind = EnableRobot if name == "EnableRobot" else StopDrag
        rig.futures[name].set_result(kind.Response(res=0))
        assert robot.request_stop.call_count == 2
        assert robot.request_stop.call_args.kwargs == {"fresh": True}


@pytest.mark.parametrize("failure", ["joints", "feed", "fault", "outputs", "disconnect"])
def test_readiness_keeps_feedback_and_latched_safety_guards(robot, monkeypatch, failure):
    rig = readiness_rig(robot, monkeypatch, 4)
    robot.prepare(rig.samples)

    def fail(_duration):
        rig.advance(.1)
        if failure == "joints":
            robot.node._latest_joint_state_stamp = ros_time(8.)
        elif failure == "feed":
            robot.progress_time = rig.now - 2.
        elif failure == "disconnect":
            robot._on_status(RobotStatus(is_connected=False))
            rig.publish()  # A brief disconnection must stay latched.
        else:
            field = "ErrorStatus" if failure == "fault" else "digital_outputs"
            rig.data[field] = 1
            rig.publish()
            rig.data[field] = 0
            rig.publish()

    monkeypatch.setattr(robot.cancel, "wait", fail)
    with pytest.raises(RuntimeError):
        robot.ensure_ready(MagicMock())
    assert rig.calls == ["EnableRobot"]


def test_stop_during_preflight_cannot_be_erased(robot, monkeypatch):
    rig = readiness_rig(robot, monkeypatch, 4)
    attempt = {"confirmed": True}
    monkeypatch.setattr(robot, "check_owners", lambda: setattr(robot, "stop_attempt", attempt))
    robot.operator_cancel.set()
    with pytest.raises(robot_module.ReplayStopped, match="Stop interrupted"):
        robot.prepare(rig.samples)
    assert robot.stop_attempt is attempt and robot.operator_cancel.is_set()
    assert not rig.calls


def test_start_confirmation_explains_enable_and_drag_and_cancel_is_read_only(node, monkeypatch):
    from camera_calibration_gui.main import QtWidgets

    window = SimpleNamespace(_node=node, _log=MagicMock())
    question = MagicMock(return_value=QtWidgets.QMessageBox.No)
    monkeypatch.setattr(QtWidgets.QMessageBox, "question", question)
    node.automatic.start = MagicMock(return_value=(True, "Preparing robot"))
    CalibrationWindow._start_automatic(window)
    node.automatic.start.assert_not_called()
    prompt = question.call_args.args[2]
    assert "exit drag mode" in prompt and "enable the robot if disabled" in prompt
    assert question.call_args.args[-1] == QtWidgets.QMessageBox.No
    question.return_value = QtWidgets.QMessageBox.Yes
    CalibrationWindow._start_automatic(window)
    node.automatic.start.assert_called_once()


@pytest.mark.parametrize("mode", [4, 6])
def test_readiness_never_reenables_or_exits_drag_mid_replay(robot, monkeypatch, mode):
    rig = readiness_rig(robot, monkeypatch, 5)
    robot.prepare(rig.samples)
    robot.ensure_ready(MagicMock())
    rig.data.update(robot_mode=mode, EnableStatus=int(mode != 4))
    rig.publish()
    rig.data.update(robot_mode=5, EnableStatus=1)
    rig.publish()
    with pytest.raises(RuntimeError, match="state or I/O changed"):
        robot.guard()
    assert not rig.calls
