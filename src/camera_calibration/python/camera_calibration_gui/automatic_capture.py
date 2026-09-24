"""Attended calibration replay using direct Dobot maintenance commands."""

import threading
import time
import uuid

from dobot_msgs_v4.srv import CP

from .maintenance_robot import MaintenanceRobot, MotionArrivalTimeout, ReplayStopped


CAPTURE_STABILITY_SEC = 1.0
MAX_POSITION_ATTEMPTS = 3


class AutomaticCapture:
    def __init__(self, node):
        self.node = node
        self.robot = MaintenanceRobot(node)
        self.recipe = self.source_path = None
        self.active = self.capturing = self.complete = self.started = self.stopping = False
        self.run_id = ""
        self.pending = self.thread = self.retry_prompt = None
        self.captured = 0
        self.message = "Load a calibration to reuse its robot positions."
        # RGB/joints and capture remain serialized. Dobot feedback and Stop use
        # the robot's independent group on the existing two-thread ROS executor.
        self.timer = node.create_timer(.02, self._tick)

    def set_recipe(self, artifact, path):
        with self.node._lock:
            self.clear_recipe()
            self.recipe, self.source_path = artifact, path
            self.message = f"Loaded route: {len(artifact.samples)} positions from {path.name}."

    def clear_recipe(self):
        if self.active or self.capturing or self.stopping:
            raise RuntimeError("Stop automatic capture before changing settings")
        self.recipe = self.source_path = None
        self.started = self.complete = False
        self.message = "Load a calibration to reuse its robot positions."

    def _require_camera(self):
        with self.node._lock:
            stamp = self.node._latest_valid_rgb_stamp
            if (self.node._fatal_error is not None or stamp is None
                    or self.node._camera_matrix is None
                    or not 0 <= (self.node.get_clock().now().nanoseconds - stamp) / 1e9 <= .5):
                raise RuntimeError("A live valid RGB stream and CameraInfo are required")

    def start(self):
        with self.node._lock:
            if self.active or self.capturing or self.stopping or self.recipe is None:
                return False, "Load a calibration and confirm Stop before starting another run."
            self.active = True
        try:
            self._require_camera()
            targets = self.robot.prepare(self.recipe.samples)
        except Exception as exc:
            self.active = False
            return False, str(exc)
        with self.node._lock:
            self.run_id = uuid.uuid4().hex
            self.complete = self.stopping = False
            self.captured = 0
            self.message = "Starting direct maintenance replay at 20% joint speed/acceleration."
            self.node.reset_samples(_automatic=True)
            self.started = True
            self.thread = threading.Thread(target=self._run, args=(targets,), daemon=True)
            self.thread.start()
        return True, self.message

    def _wait_camera(self, anchor, target):
        # Solving occupies the serial sensor callback. Wait for a new valid
        # callback before the next move while continuing robot supervision.
        deadline = time.monotonic() + 2.
        while True:
            self.robot.guard()
            if anchor is not None:
                self.robot.hold(anchor, *target)
            try:
                self._require_camera()
                return
            except RuntimeError:
                if time.monotonic() >= deadline or self.node._fatal_error is not None:
                    raise
                self.robot.cancel.wait(.01)

    def _stable_capture(self, index, attempt, anchor, joints, matrix):
        prefix = (f"Position {index}/{len(self.recipe.samples)}, "
                  f"attempt {attempt}/{MAX_POSITION_ATTEMPTS}")
        self.message = f"{prefix}: holding stationary for 1 second."
        self._wait_stable(anchor, joints, matrix)
        with self.node._lock:
            self.robot.guard()
            request = dict(
                event=threading.Event(), success=False, message="", index=index, attempt=attempt,
                not_before=self.node.get_clock().now().nanoseconds, joints=joints,
                deadline=time.monotonic() + 10., last_stamp=None, run_id=self.run_id,
                retryable=False)
            self.pending = request
            self.message = f"{prefix}: waiting for a fresh sample."
        while not request["event"].wait(.01):
            self.robot.hold(anchor, joints, matrix)
            # Solving blocks serial RGB callbacks, but never robot monitoring.
            if time.monotonic() > request["deadline"] + 20.:
                raise RuntimeError("Automatic sample processing timed out")
        self.robot.hold(anchor, joints, matrix)
        return request

    def _wait_stable(self, anchor, joints, matrix):
        started = time.monotonic()
        while True:
            sample = self.robot.hold(anchor, joints, matrix)
            if (time.monotonic() - started >= CAPTURE_STABILITY_SEC
                    and sample.sequence > anchor.sequence
                    and sample.feed["controller_timer"] > anchor.feed["controller_timer"]):
                break
            self.robot.cancel.wait(.01)

    def _retry_or_prompt(self, index, attempt, reason, phase, monitor):
        monitor()
        self.node._event_logger.record(
            "WARNING", "automatic_attempt_failed", reason, position=index,
            attempt=attempt, phase=phase)
        if attempt < MAX_POSITION_ATTEMPTS:
            self.message = (f"Position {index}: retrying automatically, "
                            f"attempt {attempt + 1}/{MAX_POSITION_ATTEMPTS}.")
            return attempt + 1
        prompt = dict(token=uuid.uuid4().hex, event=threading.Event(), accepted=False,
                      index=index, attempt=attempt, reason=reason, phase=phase)
        with self.node._lock:
            self.retry_prompt = prompt
            self.message = (f"Position {index}, attempt {attempt}/{MAX_POSITION_ATTEMPTS} failed. "
                            "Waiting for Continue or Stop.")
        self.node._event_logger.record(
            "WARNING", "automatic_retry_waiting", reason, position=index, attempt=attempt,
            phase=phase)
        try:
            while not prompt["event"].wait(.02):
                monitor()
            monitor()
            if not prompt["accepted"]:
                raise ReplayStopped("Operator stopped automatic capture")
            return 1
        finally:
            with self.node._lock:
                if self.retry_prompt is prompt:
                    self.retry_prompt = None

    def respond_retry(self, token, continue_capture):
        with self.node._lock:
            prompt = self.retry_prompt
            if (prompt is None or prompt["token"] != token or not self.active
                    or self.stopping or prompt["event"].is_set()):
                return
            if continue_capture:
                prompt["accepted"] = True
                prompt["event"].set()
                self.node._event_logger.record(
                    "INFO", "automatic_retry_continued", "Operator selected Continue",
                    position=prompt["index"], attempt=1)
                return
        self.stop()

    def _run(self, targets):
        try:
            self.robot.command("CP", CP.Request(r=100))
            anchor = previous_target = None
            for index, (joints, matrix) in enumerate(targets, 1):
                self._wait_camera(anchor, previous_target)
                needs_move = True
                attempt = 1
                while True:
                    if needs_move:
                        self.message = (f"Position {index}/{len(targets)}, "
                                        f"attempt {attempt}/{MAX_POSITION_ATTEMPTS}: moving.")
                        try:
                            anchor = self.robot.move(joints, matrix, monitor=self._require_camera)
                        except MotionArrivalTimeout as exc:
                            self.stopping = True
                            self.robot.request_stop()
                            self.robot.wait_stop()
                            stopped_attempt = self.robot.stop_attempt
                            stopped_anchor = self.robot.stopped_retry_snapshot(stopped_attempt)
                            self.stopping = False
                            attempt = self._retry_or_prompt(
                                index, attempt, str(exc), "motion",
                                lambda: self.robot.hold_stopped(stopped_anchor, stopped_attempt))
                            self.robot.resume_after_stop(stopped_attempt)
                            self._require_camera()
                            continue
                        needs_move = False
                    result = self._stable_capture(index, attempt, anchor, joints, matrix)
                    if result["success"]:
                        break
                    if not result["retryable"]:
                        raise RuntimeError(result["message"])
                    attempt = self._retry_or_prompt(
                        index, attempt, result["message"], "capture",
                        lambda: self.robot.hold(anchor, joints, matrix))
                previous_target = joints, matrix
            with self.node._lock:
                self.robot.guard()
                self.complete = self.captured == len(targets)
                if not self.complete:
                    raise RuntimeError("Automatic capture ended with missing samples")
                self.message = ("Complete. Robot remains at the last position. "
                                "Save as New Calibration.")
        except Exception as exc:
            self.complete = False
            self.stopping = True
            self.robot.request_stop()
            self.message = f"{exc}. Waiting for physical Stop confirmation."
            try:
                self.robot.wait_stop()
                self.stopping = False
                self.message = f"Automatic capture ended: {exc}. Robot Stop confirmed."
            except Exception as stop_error:
                self.message = f"{exc}. STOP UNCONFIRMED: {stop_error}. Use Stop again."
        finally:
            self.robot.end()
            with self.node._lock:
                self.retry_prompt = None
                self.active = False
                self._finish_pending(False, self.message)
                self.node._event_logger.record(
                    "INFO" if self.complete else "WARNING", "automatic_capture_finished",
                    self.message, run_id=self.run_id, captured=self.captured)

    def _finish_pending(self, success, message, *, retryable=False):
        pending, self.pending = self.pending, None
        if pending is not None:
            pending.update(success=success, message=message, retryable=retryable)
            pending["event"].set()

    def _tick(self):
        with self.node._lock:
            if not self.active:
                with self.robot.condition:
                    attempt = self.robot.stop_attempt or {}
                    if attempt and not attempt.get("confirmed"):
                        self.stopping = True
                        self.complete = False
                    if self.stopping:
                        if attempt.get("confirmed"):
                            self.stopping = False
                            self.message = "Robot Stop confirmed. Reload or restart the route."
                        elif attempt.get("error"):
                            self.message = f"STOP UNCONFIRMED: {attempt['error']}. Use Stop again."
                        else:
                            self.message = "Waiting for physical Stop confirmation."
                return
            pending = self.pending
            if pending is None or self.stopping:
                return
            if time.monotonic() >= pending["deadline"]:
                self._finish_pending(False, "No acceptable fresh sample within 10 seconds: "
                                     + self.message, retryable=True)
                return
            stamp = self.node._latest_pose_stamp
            if stamp is None or stamp == pending["last_stamp"]:
                return
            pending["last_stamp"] = stamp
            self.capturing = True
        before = len(self.node._calibration_samples)
        try:
            success, message = self.node.capture_sample(
                not_before_ns=pending["not_before"], expected_joints=pending["joints"],
                run_id=pending["run_id"])
        except Exception as exc:
            success, message = False, f"Automatic capture failed: {exc}"
            self.stop()
        finally:
            with self.node._lock:
                self.capturing = False
        with self.node._lock:
            if self.pending is not pending or self.stopping:
                return
            if success:
                self.captured += 1
                self._finish_pending(True, message)
            elif len(self.node._calibration_samples) != before or self.node._fatal_error:
                self._finish_pending(False, message)
            else:
                self.message = (f"Position {pending['index']}/{len(self.recipe.samples)}, "
                                f"attempt {pending['attempt']}/{MAX_POSITION_ATTEMPTS}: "
                                f"waiting for fresh sample — {message}")

    def stop(self):
        # Never wait for RGB processing, motion responses or enabled-state checks.
        self.robot.operator_cancel.set()
        self.robot.request_stop(fresh=True)
        with self.node._lock:
            if self.retry_prompt is not None:
                self.retry_prompt["event"].set()
                self.retry_prompt = None
            self.stopping = True
            self.complete = False
            self._finish_pending(False, "Operator stopped automatic capture")
            self.message = "Direct Dobot Stop requested; waiting for physical confirmation."

    def close(self):
        if self.active or self.stopping:
            self.stop()
            try:
                self.robot.wait_stop()
            except Exception as exc:
                self.node._event_logger.record(
                    "ERROR", "replay_shutdown_stop_unconfirmed", str(exc))
        if self.thread is not None:
            self.thread.join(timeout=8.)
