"""Calibration action client and nonblocking fresh-sample handshake; no Dobot clients."""

import time
import uuid

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.time import Time
from robot_controller_interfaces.action import ReplayCalibration
from robot_controller_interfaces.msg import CalibrationPosition
from robot_controller_interfaces.srv import CaptureCalibration, Command


class AutomaticCapture:
    def __init__(self, node):
        self.node = node
        self.recipe = None
        self.source_path = None
        self.active = False
        self.capturing = False
        self.complete = False
        self.started = False
        self.run_id = ""
        self.goal = None
        self.stopping = False
        self.pending = None
        self.captured = 0
        self.last_feedback = None
        self.watchdog_stopped = False
        self.prepared = 0
        self.message = "Load a calibration to reuse its robot positions."
        self.client = ActionClient(node, ReplayCalibration, "/robot_controller/replay_calibration")
        # The async service releases its executor thread while waiting for the
        # serialized sensor/timer group; TF retains the second executor thread.
        self.service = node.create_service(
            CaptureCalibration, "/camera_calibration/capture_automatic", self._service,
            callback_group=ReentrantCallbackGroup())
        self.stop_client = node.create_client(Command, "/robot_controller/stop")
        self.timer = node.create_timer(0.02, self._tick)

    def set_recipe(self, artifact, path):
        with self.node._lock:
            if self.active or self.capturing:
                raise RuntimeError("Stop automatic capture before loading another calibration")
            self.recipe, self.source_path = artifact, path
            self.started = self.complete = False
            self.message = f"Loaded route: {len(artifact.samples)} positions from {path.name}."

    def clear_recipe(self):
        if self.active or self.capturing:
            raise RuntimeError("Stop automatic capture before changing settings")
        self.recipe = self.source_path = None
        self.started = self.complete = False
        self.message = "Load a calibration to reuse its robot positions."

    def start(self):
        with self.node._lock:
            if self.active or self.capturing or self.recipe is None:
                return False, "Load a calibration before starting automatic capture."
            if not self.client.server_is_ready():
                return False, "Robot controller unavailable; start robot_controller first."
            if self.node._fatal_error is not None:
                return False, self.node._fatal_error
            self.run_id = uuid.uuid4().hex
            self.active, self.complete, self.stopping = True, False, False
            self.last_feedback = time.monotonic()
            self.watchdog_stopped = False
            self.goal = None
            self.captured = self.prepared = 0
            request = ReplayCalibration.Goal(
                run_id=self.run_id, positions=[
                    CalibrationPosition(joints_rad=list(sample.joint_positions_rad))
                    for sample in self.recipe.samples])
            self.message = "Requesting replay; robot must be enabled, idle, user/tool 0, DI1 LOW."
            try:
                token = self.run_id
                future = self.client.send_goal_async(
                    request, feedback_callback=lambda update: self._feedback(token, update))
                future.add_done_callback(self._accepted)
            except Exception as exc:
                self.active = False
                return False, str(exc)
            return True, self.message

    def _accepted(self, future):
        try:
            with self.node._lock:
                self.goal = future.result()
                if not self.goal.accepted:
                    self.active = False
                    self.message = (
                        "Replay rejected. Check controller log: enabled/idle robot, user/tool 0, "
                        "DI1 LOW, no held item or competing operation, valid CR10 joints.")
                    return
                if self.stopping:
                    self.goal.cancel_goal_async()
                self.goal.get_result_async().add_done_callback(self._finished)
        except Exception as exc:
            self.message = f"Replay action failed: {exc}"
            self.stop()

    def _feedback(self, token, feedback):
        update = feedback.feedback
        with self.node._lock:
            if token != self.run_id or not self.active:
                return
            self.last_feedback = time.monotonic()
            if not self.stopping:
                self.message = (f"Position {update.position_index}/{update.position_total}: "
                                f"{update.phase.lower()} — {update.message}")

    def _finished(self, future):
        with self.node._lock:
            try:
                reply = future.result()
                result = reply.result
                self.complete = (not self.stopping and reply.status == GoalStatus.STATUS_SUCCEEDED
                                 and result.outcome == result.SUCCESS
                                 and result.captured_count == self.captured
                                 and self.captured == len(self.recipe.samples))
                self.message = ("Complete. Save as New Calibration when ready. " if self.complete
                                else "Automatic capture ended without completing the route. ")
                self.message += result.message
            except Exception as exc:
                self.complete = False
                self.message = f"Replay result unavailable: {exc}. Use controller Stop."
                self.stop()
            finally:
                self.active = False
                self._finish_pending(False, self.message)
                self.node._event_logger.record(
                    "INFO" if self.complete else "WARNING", "automatic_capture_finished",
                    self.message, run_id=self.run_id, captured=self.captured)

    def _finish_pending(self, success, message):
        pending, self.pending = self.pending, None
        if pending is not None:
            pending[1].set_result(CaptureCalibration.Response(success=success, message=message))

    async def _service(self, request, response):
        with self.node._lock:
            if (not self.active or self.stopping or request.run_id != self.run_id
                    or request.position_index != self.captured + 1
                    or self.recipe is None or request.position_index > len(self.recipe.samples)
                    or self.node._fatal_error is not None):
                response.message = "Inactive, stopped, failed or out-of-order calibration run"
                return response
            if request.phase == request.PREPARE:
                if self.pending is not None or self.prepared == request.position_index:
                    response.message = "Duplicate preparation request"
                    return response
                stamp = self.node._latest_valid_rgb_stamp
                if (stamp is None or self.node._camera_matrix is None
                        or not 0 <= (self.node.get_clock().now().nanoseconds
                                     - stamp) / 1e9 <= 0.5):
                    response.message = "A live valid RGB stream and CameraInfo are required"
                    return response
                if request.position_index == 1:
                    self.node.reset_samples(_automatic=True)
                    self.started = True
                self.prepared = request.position_index
                response.success = True
                response.message = "Ready for next saved position"
                return response
            stamp = Time.from_msg(request.not_before).nanoseconds
            if (request.phase != request.CAPTURE or stamp <= 0 or self.pending is not None
                    or self.prepared != request.position_index
                    or stamp > self.node.get_clock().now().nanoseconds):
                response.message = "Invalid or duplicate capture request"
                return response
            future = Future()
            self.pending = (request, future, time.monotonic() + 10.)
            self.last_attempt_stamp = None
        return await future

    def _tick(self):
        with self.node._lock:
            if not self.active:
                return
            if self.node._fatal_error is not None:
                if not self.stopping:
                    self.stop()
                return
            if (not self.stopping and not self.watchdog_stopped and (
                    not self.client.server_is_ready()
                    or self.last_feedback is not None
                    and time.monotonic() - self.last_feedback > 5.)):
                self.watchdog_stopped = True
                self.stop()
                self.message = (
                    "Controller unresponsive; Stop requested, physical stop unconfirmed.")
                return
            if self.pending is None:
                return
            request, _future, deadline = self.pending
            if self.stopping:
                self._finish_pending(False, "Operator stopped automatic capture")
                return
            if time.monotonic() >= deadline:
                self._finish_pending(False, "No acceptable fresh sample within 10 seconds: "
                                     + self.message)
                return
            stamp = self.node._latest_pose_stamp
            if stamp is None or stamp == self.last_attempt_stamp:
                return
            self.last_attempt_stamp = stamp
            target = self.recipe.samples[request.position_index - 1].joint_positions_rad
            self.capturing = True
        # Do not hold the node lock over TF acquisition or OpenCV solving.
        before = len(self.node._calibration_samples)
        try:
            success, message = self.node.capture_sample(
                not_before_ns=Time.from_msg(request.not_before).nanoseconds,
                expected_joints=target, run_id=request.run_id)
        except Exception as exc:
            self.stop()
            self.message = f"Automatic capture failed: {exc}"
            return
        finally:
            with self.node._lock:
                self.capturing = False
        with self.node._lock:
            if self.pending is None or self.stopping:
                return
            if success:
                self.captured += 1
                self._finish_pending(True, message)
            elif len(self.node._calibration_samples) != before or self.node._fatal_error:
                self._finish_pending(False, message)
            else:
                self.message = (f"Position {request.position_index}/{len(self.recipe.samples)}: "
                                f"waiting for fresh sample — {message}")

    def stop(self):
        with self.node._lock:
            self.stopping = True
            self.complete = False
            self._finish_pending(False, "Automatic capture stopped")
            self.message = "Stop requested; waiting for controller confirmation."
            if self.goal is not None and self.goal.accepted:
                try:
                    self.goal.cancel_goal_async()
                except Exception as exc:
                    self.message += f" Action cancellation failed: {exc}."
            if self.stop_client.service_is_ready():
                try:
                    self.stop_client.call_async(Command.Request())
                except Exception as exc:
                    self.message = f"STOP UNCONFIRMED: {exc}"
            else:
                self.message = "STOP UNCONFIRMED: controller Stop service is unavailable."
