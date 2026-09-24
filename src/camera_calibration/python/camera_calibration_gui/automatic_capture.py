"""Attended calibration replay using direct Dobot maintenance commands."""

import threading
import time
import uuid

from dobot_msgs_v4.srv import CP

from .maintenance_robot import MaintenanceRobot


class AutomaticCapture:
    def __init__(self, node):
        self.node = node
        self.robot = MaintenanceRobot(node)
        self.recipe = self.source_path = None
        self.active = self.capturing = self.complete = self.started = self.stopping = False
        self.run_id = ""
        self.pending = self.thread = None
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

    def _run(self, targets):
        try:
            self.robot.command("CP", CP.Request(r=100))
            anchor = previous_target = None
            for index, (joints, matrix) in enumerate(targets, 1):
                # A solve occupies the serial sensor callback. Allow its next
                # independently valid frame to arrive before dispatching motion.
                deadline = time.monotonic() + 2.
                while True:
                    self.robot.guard()
                    if anchor is not None:
                        self.robot.hold(anchor, *previous_target)
                    try:
                        self._require_camera()
                        break
                    except RuntimeError:
                        if time.monotonic() >= deadline or self.node._fatal_error is not None:
                            raise
                        self.robot.cancel.wait(.01)
                self.message = f"Position {index}/{len(targets)}: moving to saved joints."
                anchor = self.robot.move(joints, matrix, monitor=self._require_camera)
                previous_target = joints, matrix
                with self.node._lock:
                    self.robot.guard()
                    request = dict(
                        event=threading.Event(), success=False, message="", index=index,
                        not_before=self.node.get_clock().now().nanoseconds, joints=joints,
                        deadline=time.monotonic() + 10., last_stamp=None, run_id=self.run_id)
                    self.pending = request
                    self.message = f"Position {index}/{len(targets)}: waiting for a fresh sample."
                while not request["event"].wait(.01):
                    self.robot.hold(anchor, joints, matrix)
                    # Solving blocks serial RGB callbacks, but never robot monitoring.
                    limit = request["deadline"] + (20. if self.capturing else 0.)
                    if time.monotonic() > limit:
                        raise RuntimeError("Automatic sample processing timed out")
                self.robot.hold(anchor, joints, matrix)
                if not request["success"]:
                    raise RuntimeError(request["message"])
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
                self.active = False
                self._finish_pending(False, self.message)
                self.node._event_logger.record(
                    "INFO" if self.complete else "WARNING", "automatic_capture_finished",
                    self.message, run_id=self.run_id, captured=self.captured)

    def _finish_pending(self, success, message):
        pending, self.pending = self.pending, None
        if pending is not None:
            pending.update(success=success, message=message)
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
                                     + self.message)
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
                self.message = (f"Position {pending['index']}/{len(self.recipe.samples)}: "
                                f"waiting for fresh sample — {message}")

    def stop(self):
        # Never wait for RGB processing, motion responses or enabled-state checks.
        self.robot.request_stop(fresh=True)
        with self.node._lock:
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
