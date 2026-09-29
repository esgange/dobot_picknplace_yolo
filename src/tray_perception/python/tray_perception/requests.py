"""One fresh, hash-bound tray observation pipeline for ROS and local simulation."""

import copy
import json
import os
from pathlib import Path
import threading
import time
import traceback
import uuid

import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from tf2_ros import TransformException
from camera_calibration_gui.calibration_core import quaternion_to_rotation_matrix

from item_perception_yolo.item_detector import encode_rgb_png
from item_perception_yolo.item_teach_core import file_sha256
from tray_perception_interfaces.srv import GetTrayPose

from .core import load_profile, ORIGIN_CONVENTION
from .documents import detection_profile, load_document
from .placement import sampling_from_message
from .contract import SERVICE_NAME


REQUEST_TIMEOUT = 10.0


def pose_extents(selected):
    try:
        corners = np.asarray(selected["corners_base_m"], dtype=float)
        if corners.shape != (4, 3) or not np.isfinite(corners).all():
            raise ValueError("Invalid tray corners")
        axes = corners[[1, 3]] - corners[0]
        lengths = np.linalg.norm(axes, axis=1)
        rotation = quaternion_to_rotation_matrix(*selected["quaternion"])
        if (np.any(lengths <= 0) or not np.allclose(corners[0], selected["position"], atol=1e-6)
                or not np.allclose(axes / lengths[:, None], rotation[:, :2].T, atol=1e-5)
                or not np.allclose(corners[2], corners[0] + axes.sum(axis=0), atol=1e-6)
                or not np.allclose(lengths,
                                   [selected["width_mm"] / 1000, selected["length_mm"] / 1000],
                                   atol=1e-6)):
            raise ValueError("Tray corners, axes or dimensions disagree")
        return tuple(float(value) for value in lengths)
    except (KeyError, ValueError, TypeError) as exc:
        raise RuntimeError(f"Invalid native tray extent evidence: {exc}") from exc


def preview_settings(settings, metadata):
    value = copy.deepcopy(settings)
    value.pop("name")
    value["accepted_class_ids"] = value["yolo"]["class_ids"][:]
    value["yolo"]["class_ids"] = sorted(int(key) for key in metadata["classes"])
    return value


def fingerprints(paths):
    return tuple((str(path), path.stat().st_ino, path.stat().st_size, path.stat().st_mtime_ns)
                 for path in paths)


def save_debug(root, identifier, view):
    directory = Path(root) / "debug/tray_img"
    directory.mkdir(parents=True, exist_ok=True)
    result = {"requested": True, "rgb_path": "", "depth_path": "", "error": ""}
    rgb = view["rgb"]
    for kind, data in (("rgb", view["overlay"]), ("depth", view["depth_overlay"])):
        if not data:
            result["error"] = "Registered depth unavailable; saved RGB only"
            continue
        target = directory / f"tray_{identifier}_{kind}.png"
        temporary = directory / f".{target.name}.tmp"
        try:
            temporary.write_bytes(encode_rgb_png(data, rgb["width"], rgb["height"]))
            os.replace(temporary, target)
            result[f"{kind}_path"] = str(target)
        finally:
            temporary.unlink(missing_ok=True)
    return result


class TrayRequests:
    def __init__(self, node):
        self.node = node
        self.lock = threading.RLock()
        self.request_lock = threading.Lock()
        self.service = self.binding = None
        self.epoch = 0
        self.next_check = 0.
        self.retired = []
        self.callbacks = 0
        self.last_reply = 0.
        self.status = "Disarmed — no tray pose service"

    @property
    def busy(self):
        return self.request_lock.locked()

    def disarm(self, reason="Disarmed"):
        with self.lock:
            self.epoch += 1
            service, self.service = self.service, None
            self.binding = None
            self.status = reason
            if service is not None:
                self.retired.append(service)
        if service is not None:
            self.node.events.record("INFO", "tray_disarmed", reason)

    def _retire(self):
        # rclpy sends the reply after our callback returns. Destroying that service
        # inside the callback can raise InvalidHandle in the executor. Revoke it
        # logically at once, then retire its ROS handle after the reply handoff.
        with self.lock:
            if self.callbacks or time.monotonic() - self.last_reply < .2:
                return
            for service in self.retired:
                self.node.destroy_service(service)
            self.retired.clear()

    def prepare(self, path, settings, expected_digest):
        node = self.node
        if not node.yolo_enabled or node.native.failed or node.fatal_error:
            raise ValueError("Enable YOLO with valid saved settings before triggering or arming")
        if node.model is None:
            raise ValueError("Load the saved tray model before triggering or arming")
        if node.deployment:
            profile = load_profile(path, node.root, deployment=True)
        else:
            document = load_document(path, node.root)
            profile = detection_profile(document, node.model["task"])
            if (document["artifact_type"] == "tray_teach_draft" and node.camera is not None
                    and document["form"]["draft"]["camera_prefix"].strip() !=
                    node.camera.settings.camera_prefix):
                raise ValueError("Saved draft camera prefix does not match its calibration")
        digest = file_sha256(Path(path))
        if digest != expected_digest:
            raise ValueError("Saved Tray Teach YAML changed; explicitly reload it")
        node.validate_sources()
        if profile["settings"] != settings or profile["reference_plane"] != node.plane:
            raise ValueError("Save or load the exact current tray detection settings and plane")
        camera = {"filename": node.camera.path.name, "sha256": node.camera.sha256}
        if (profile["camera_calibration"] != camera or node.model is None
                or node.model["sha256"] != profile["model"]["sha256"]
                or file_sha256(Path(node.model["path"])) != profile["model"]["sha256"]
                or node.model["task"] != settings["model_task"]
                or any(str(i) not in node.model_metadata["classes"]
                       for i in settings["yolo"]["class_ids"])):
            raise ValueError("Saved tray model or camera does not match the loaded sources")
        paths = (Path(path), Path(path).with_suffix(".pt"),
                 Path(node.model["path"]), node.camera.path)
        return {"path": Path(path), "digest": digest, "profile": profile,
                "settings": copy.deepcopy(settings), "paths": paths,
                "fingerprints": fingerprints(paths)}

    def _sole_provider(self):
        node = self.node
        own = (node.get_name(), node.get_namespace())
        identities = node.get_node_names_and_namespaces()
        if identities.count(own) > 1:
            raise ValueError("Duplicate tray detector node identity")
        for name, namespace in identities:
            if (name, namespace) != own and any(
                    service == SERVICE_NAME for service, _ in
                    node.get_service_names_and_types_by_node(name, namespace)):
                raise ValueError(f"Another node already provides {SERVICE_NAME}")

    def arm(self, path, settings, expected_digest):
        self.disarm()
        epoch = self.epoch
        retirement_deadline = time.monotonic() + 1.
        while self.retired:
            self._retire()
            if not self.retired:
                break
            if time.monotonic() >= retirement_deadline:
                raise ValueError("Previous tray service reply is still retiring; retry arming")
            time.sleep(.01)
        binding = self.prepare(path, settings, expected_digest)
        self._sole_provider()
        view = self.node.snapshot()
        self.node._check_snapshot(view)
        with self.lock:
            if self.epoch != epoch:
                raise ValueError("Tray inputs changed while arming; try again when ready")
            self.binding = binding
            self.service = self.node.create_service(
                GetTrayPose, SERVICE_NAME,
                lambda request, response: self.handle(request, response, provider_epoch=epoch),
                callback_group=ReentrantCallbackGroup())
            self.status = f"Armed: {binding['path'].name} | SHA-256 {binding['digest']}"
        self.node.events.record("INFO", "tray_armed", self.status)

    def tick(self):
        self._retire()
        if self.service is None or time.monotonic() < self.next_check:
            return
        self.next_check = time.monotonic() + 1.
        try:
            binding = self.binding
            if binding is None:
                return
            self.node.validate_sources()
            if (fingerprints(binding["paths"]) != binding["fingerprints"]
                    or file_sha256(binding["path"]) != binding["digest"]):
                raise ValueError("Bound tray files changed; reload/restart before arming")
            self._sole_provider()
        except (ValueError, OSError, RuntimeError) as exc:
            self.node.invalidate(str(exc))
            if self.node.deployment:
                self.node.fatal_error = str(exc)

    def _check(self, epoch, generation, deadline, *, simulated):
        if (self.epoch != epoch or self.node.generation != generation
                or not self.node.yolo_enabled or self.node.native.closed
                or (not simulated and self.service is None)):
            raise ValueError("Tray request cancelled by disarming, source/settings change or exit")
        if time.monotonic() >= deadline:
            raise TimeoutError("Tray request exceeded its configured deadline")

    def _fresh_view(self, start_ns, started, deadline, check, sampling=None):
        reason = "Waiting for RGB captured after this trigger"
        while time.monotonic() < deadline:
            check()
            try:
                view = (self.node.snapshot() if sampling is None else
                        self.node.snapshot(depth_required=True, quality=sampling))
                rgb = view["rgb"]
                if rgb["stamp_ns"] > start_ns and rgb["received_at"] >= started:
                    if sampling is not None and (
                            view["depth"]["stamp_ns"] <= start_ns
                            or view["depth"]["received_at"] < started):
                        reason = "Waiting for registered depth captured after this trigger"
                        time.sleep(.01)
                        continue
                    self.node._check_snapshot(view)
                    if view["camera_context"]["camera"] != self.node.plane["camera"]:
                        raise ValueError("CameraInfo differs from saved reference plane")
                    return view
            except (ValueError, TransformException) as exc:
                reason = str(exc)
            time.sleep(.01)
        raise TimeoutError(f"No fresh calibrated tray observation: {reason}")

    def handle(self, request, response, *, provider_epoch=None):
        with self.lock:
            self.callbacks += 1
        try:
            if provider_epoch is not None and provider_epoch != self.epoch:
                return GetTrayPose.Response(status="ERROR", message="Tray provider was disarmed")
            response, _ = self._batch(request, response)
            return response
        finally:
            with self.lock:
                self.callbacks -= 1
                self.last_reply = time.monotonic()

    def validate_view(self, view):
        binding = view["trigger_binding"]
        if (view["trigger_epoch"] != self.epoch or not self.node.yolo_enabled
                or self.node.native.failed or self.node.native.closed or self.node.fatal_error
                or fingerprints(binding["paths"]) != binding["fingerprints"]
                or file_sha256(binding["path"]) != binding["digest"]):
            raise ValueError("Simulated tray profile or source identity changed")
        self.node._check_snapshot(view)

    def simulate(self, path, settings, expected_digest, requested_at):
        request = GetTrayPose.Request(profile_sha256=expected_digest)
        response, view = self._batch(request, GetTrayPose.Response(),
                                     simulation=(path, settings), requested_at=requested_at)
        return {"response": response, "view": view}

    def _batch(self, request, response, *, simulation=None, requested_at=None):
        if not self.request_lock.acquire(blocking=False):
            response.status, response.message = "BUSY", "One tray request is already active"
            return response, None
        start_ns, started = requested_at or (self.node.get_clock().now().nanoseconds,
                                             time.monotonic())
        deadline = started + REQUEST_TIMEOUT
        epoch, generation = self.epoch, self.node.generation
        acquired, view = False, None
        simulated = simulation is not None
        request_id = uuid.uuid4().hex
        phase = "validate_request"

        def check():
            self._check(epoch, generation, deadline, simulated=simulated)

        try:
            self.node.events.record(
                "INFO", "tray_request_started", "Simulated" if simulated else SERVICE_NAME,
                request_id=request_id, profile_sha256=request.profile_sha256,
                sample_placement_depth=bool(request.sample_placement_depth))
            sampling = (sampling_from_message(request.placement)
                        if request.sample_placement_depth else None)
            if sampling is not None:
                deadline = started + sampling["request_timeout_sec"]
            check()
            binding = self.binding
            if not simulated and (binding is None or request.profile_sha256 != binding["digest"]):
                raise ValueError("Requested Tray Teach profile SHA-256 mismatch")
            phase = "wait_for_preview"
            while not acquired:
                check()
                acquired = self.node.work_lock.acquire(
                    timeout=min(.02, max(.001, deadline - time.monotonic())))
            check()
            path, settings = simulation if simulated else (binding["path"], binding["settings"])
            phase = "validate_sources"
            verified = self.prepare(path, settings, request.profile_sha256)
            check()
            phase = "fresh_observation"
            view = self._fresh_view(start_ns, started, deadline, check, sampling)
            phase = "inference"
            result_view = self.node.preview(
                preview_settings(settings, self.node.model_metadata), generation=generation,
                view=view, visualize=simulated or request.save_debug_images, deadline=deadline,
                returned_only=simulated,
                **({"depth_quality": sampling} if sampling is not None else {}))
            check()
            self.prepare(path, settings, request.profile_sha256)
            if not simulated:
                self._sole_provider()
            check()
            result = result_view["result"]
            selected = result["selected"]
            response.batch_id = uuid.uuid4().hex
            response.header.frame_id = "base_link"
            response.header.stamp = Time(nanoseconds=view["rgb"]["stamp_ns"]).to_msg()
            response.detected_count = result["count"]
            response.valid_count = sum(d["valid"] for d in result["detections"])
            response.found = selected is not None
            if selected is not None:
                pose = response.tray
                pose.id = f"{response.batch_id}:{selected['source_index']}"
                pose.class_id, pose.class_name = selected["class_id"], selected["class_name"]
                pose.confidence = float(selected["confidence"])
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(
                    float, selected["position"])
                q = pose.pose.orientation
                q.x, q.y, q.z, q.w = map(float, selected["quaternion"])
                pose.length, pose.width = selected["length_mm"] / 1000, selected["width_mm"] / 1000
                pose.extent_x, pose.extent_y = pose_extents(selected)
                pose.center_distance_px = float(selected["center_distance_px"])
                if sampling is not None:
                    phase = "placement_depth"
                    if view["depth"] is None:
                        raise ValueError("Placement requires synchronized registered depth")
                    measured, data = self.node.native.call({
                        "operation": "tray_placement_depth", "width": view["rgb"]["width"],
                        "height": view["rgb"]["height"], "selected": selected,
                        "camera_context": view["camera_context"], "sampling": sampling},
                        view["depth"]["depth"], timeout=max(.001, deadline - time.monotonic()))
                    check()
                    if (not data and set(measured) == {"state", "error"}
                            and type(measured["error"]) is str and measured["error"]):
                        raise ValueError(measured["error"])
                    if (data or set(measured) != {
                            "state", "surface_base", "accepted_samples",
                            "total_samples", "median_mm", "sigma_mm"}
                            or len(measured["surface_base"]) != 3
                            or not np.isfinite(measured["surface_base"] + [
                                measured["median_mm"], measured["sigma_mm"]]).all()
                            or not sampling["depth_min_mm"] <= measured["median_mm"]
                            <= sampling["depth_max_mm"] or measured["sigma_mm"] < 0
                            or not measured["total_samples"] >= measured["accepted_samples"]
                            >= sampling["minimum_depth_samples"]
                            or measured["accepted_samples"] / measured["total_samples"]
                            < sampling["minimum_depth_fraction"]):
                        raise RuntimeError("Invalid native placement-depth reply")
                    sampled = response.placement
                    sampled.surface_base.x, sampled.surface_base.y, sampled.surface_base.z = map(
                        float, measured["surface_base"])
                    for key in ("accepted_samples", "total_samples", "median_mm", "sigma_mm"):
                        setattr(sampled, key, measured[key])
                    sampled.depth_header.frame_id = "base_link"
                    sampled.depth_header.stamp = Time(
                        nanoseconds=view["depth"]["stamp_ns"]).to_msg()
                    sampled.valid = True
                    self.prepare(path, settings, request.profile_sha256)
                    self._sole_provider()
            debug = {"requested": bool(request.save_debug_images), "rgb_path": "",
                     "depth_path": "", "error": ""}
            if request.save_debug_images:
                try:
                    debug = save_debug(self.node.root, response.batch_id, result_view)
                except Exception as exc:
                    debug["error"] = str(exc)
                check()
                self.prepare(path, settings, request.profile_sha256)
            response.diagnostics_json = json.dumps({
                "profile_sha256": verified["digest"], "model_sha256": self.node.model["sha256"],
                "camera_sha256": self.node.camera.sha256, "origin_convention": ORIGIN_CONVENTION,
                "snapshot_context": view["camera_context"],
                "reference_plane": verified["profile"]["reference_plane"],
                "placement_sampling": sampling,
                "inference_ms": result["inference_ms"], "detections": result["detections"],
                "debug_capture": debug}, allow_nan=False)
            check()
            response.success = True
            response.status = "OK" if response.found else "NO_VALID_TRAY"
            response.message = "One tray pose returned" if response.found else "No eligible tray"
            view = {**result_view, "trigger_binding": verified, "trigger_epoch": epoch}
            self.status = f"{response.status}: {response.message} | {response.batch_id}"
            event = "tray_simulated" if simulated else "tray_pose_response"
            self.node.events.record("INFO", event,
                                    self.status, profile_sha256=request.profile_sha256,
                                    request_id=request_id,
                                    source_stamp_ns=view["rgb"]["stamp_ns"],
                                    detected_count=response.detected_count,
                                    valid_count=response.valid_count,
                                    returned_tray=None if selected is None else {
                                        key: selected[key] for key in (
                                            "source_index", "class_name", "confidence",
                                            "position", "quaternion", "length_mm", "width_mm",
                                            "mask_clean") if key in selected},
                                    rejections=[{key: d[key] for key in (
                                        "source_index", "reason", "length_mm", "width_mm",
                                        "mask_clean")
                                        if key in d} for d in result["detections"]
                                        if not d["valid"]],
                                    elapsed_sec=time.monotonic() - started)
        except Exception as exc:
            response = GetTrayPose.Response(success=False, found=False,
                                            status="ERROR", message=str(exc))
            view = None
            self.status = f"ERROR: {exc}"
            self.node.events.record(
                "ERROR", "tray_request_failed", str(exc), request_id=request_id, phase=phase,
                elapsed_sec=time.monotonic() - started, traceback=traceback.format_exc())
            terminal = isinstance(exc, RuntimeError) and not self.node.native.closed
            if self.node.native.failed or terminal:
                self.node.fatal_error = str(exc)
                self.disarm(str(exc))
        finally:
            if acquired:
                self.node.work_lock.release()
            self.request_lock.release()
        return response, view
