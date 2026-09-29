"""One fresh, strictly bound tray-and-depth observation for a Place action."""

import json
import math
import time

import numpy as np
from camera_calibration_gui.calibration_core import quaternion_to_rotation_matrix
from tray_perception.core import ORIGIN_CONVENTION
from tray_perception.placement import sampling_from_item
from tray_perception_interfaces.msg import PlacementDepthRequest
from tray_perception_interfaces.srv import GetTrayPose

from .errors import FeedbackFailure
from .motion import rigid_matrix


SERVICE = "/tray_detect/get_tray_pose"
PROVIDERS = {("tray_teach", "/"), ("tray_detect", "/")}


def validate_result(result, config, sampling, start_ns, now_ns):
    if result is None or not result.success or not result.found or not result.placement.valid:
        reason = "no response" if result is None else f"{result.status}: {result.message}"
        raise FeedbackFailure(f"No usable tray placement depth: {reason}")
    try:
        evidence = json.loads(result.diagnostics_json)
        expected = {"profile_sha256": config.tray.sha256,
                    "model_sha256": config.tray.profile["model"]["sha256"],
                    "camera_sha256": config.tray.camera_sha256,
                    "origin_convention": ORIGIN_CONVENTION,
                    "reference_plane": config.tray.profile["reference_plane"],
                    "placement_sampling": sampling}
        for key, value in expected.items():
            if evidence.get(key) != value:
                raise ValueError(f"tray/controller {key} mismatch")
        sample = result.placement
        headers = (result.header, sample.depth_header)
        stamps = [h.stamp.sec * 1_000_000_000 + h.stamp.nanosec for h in headers]
        if (any(h.frame_id != "base_link" for h in headers)
                or any(not start_ns < stamp <= now_ns for stamp in stamps)
                or abs(stamps[0] - stamps[1]) / 1e9 > sampling["sync_tolerance_sec"]):
            raise ValueError("invalid frame, cached/future observation or unsynchronized depth")
        pose = result.tray
        p, q, surface = pose.pose.position, pose.pose.orientation, sample.surface_base
        numeric = (p.x, p.y, p.z, q.x, q.y, q.z, q.w, surface.x, surface.y, surface.z,
                   pose.extent_x, pose.extent_y, pose.length, pose.width, pose.confidence,
                   sample.median_mm, sample.sigma_mm)
        settings = config.tray.profile["settings"]
        if (not all(math.isfinite(v) for v in numeric) or not result.batch_id or not pose.id
                or pose.class_id not in settings["yolo"]["class_ids"]
                or not settings["yolo"]["confidence"] <= pose.confidence <= 1
                or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 1e-5
                or not 0 < sampling["x_mm"] / 1000 < pose.extent_x
                or not 0 < sampling["y_mm"] / 1000 < pose.extent_y
                or not np.allclose([pose.extent_x, pose.extent_y],
                                   [pose.width, pose.length], atol=1e-6)
                or not sample.total_samples >= sample.accepted_samples
                >= sampling["minimum_depth_samples"]
                or sample.accepted_samples / sample.total_samples
                < sampling["minimum_depth_fraction"]
                or not sampling["depth_min_mm"] <= sample.median_mm <= sampling["depth_max_mm"]
                or sample.sigma_mm < 0):
            raise ValueError("malformed tray pose or insufficient depth evidence")
        rotation = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        plane = rigid_matrix(config.tray.profile["reference_plane"]["base_from_plane"],
                             "Tray reference plane")
        if (abs(np.dot(np.array([p.x, p.y, p.z]) - plane[:3, 3], plane[:3, 2])) > 1e-5
                or abs(abs(np.dot(rotation[:, 2], plane[:3, 2])) - 1.) > 1e-5):
            raise ValueError("returned tray pose is not on the bound reference plane")
        target = np.array([p.x, p.y, p.z]) + rotation @ np.array([
            sampling["x_mm"] / 1000, sampling["y_mm"] / 1000, 0.])
        if not np.allclose(target[:2], [surface.x, surface.y], atol=1e-6):
            raise ValueError("placement X/Y differs from the requested tray offsets")
        return np.array([surface.x, surface.y, surface.z]), evidence
    except (ValueError, TypeError, KeyError, AttributeError, ZeroDivisionError) as exc:
        raise FeedbackFailure(f"Invalid tray placement response: {exc}") from exc


class TrayClient:
    def __init__(self, node):
        self.node = node
        self.client = node.create_client(GetTrayPose, SERVICE)
        self.pending = None

    def close(self):
        self.node.destroy_client(self.client)

    def check_owner(self):
        providers = self.node._service_providers(SERVICE)
        if len(providers) != 1 or providers[0] not in PROVIDERS:
            raise FeedbackFailure("Placement requires exactly one armed tray_teach or "
                                  "headless tray_detect provider in the root namespace")

    def request(self, config, x_mm, y_mm):
        node = self.node
        config.validate_sources(node.root)
        # ROS services cannot cancel server execution. Drain an interrupted
        # request before asking for another observation, discarding its old result.
        if self.pending is not None:
            retiring, deadline = self.pending
            while not retiring.done():
                node.wait_for_resume()
                node._preflight_item_state(True)
                if time.monotonic() > deadline:
                    raise FeedbackFailure("Previous tray request has not finished; retry after it retires")
                node.wait_control(.02)
            self.pending = None
        self.check_owner()
        if not self.client.service_is_ready():
            raise FeedbackFailure("Tray pose service is unavailable; arm Tray Teach first")
        sampling = sampling_from_item(config.profile, x_mm, y_mm)
        request = GetTrayPose.Request(profile_sha256=config.tray.sha256,
                                     sample_placement_depth=True,
                                     placement=PlacementDepthRequest(**sampling))
        start = node.get_clock().now().nanoseconds
        deadline = time.monotonic() + sampling["request_timeout_sec"] + 1
        node.operation_progress("TRAY_DEPTH", "Requesting fresh tray pose and placement depth")
        future = self.client.call_async(request)
        self.pending = (future, deadline)
        try:
            while not future.done():
                node.wait_for_resume()
                node._preflight_item_state(True)
                if time.monotonic() > deadline:
                    raise FeedbackFailure("Tray depth request timed out; no automatic retry")
                node.wait_control(.02)
            node.wait_for_resume()
            config.validate_sources(node.root)
            self.check_owner()
            point, evidence = validate_result(
                future.result(), config, sampling, start, node.get_clock().now().nanoseconds)
            node.events.record("INFO", "placement_depth", "Fresh tray depth accepted",
                               surface_base_m=point.tolist(), sampling=sampling,
                               tray_sha256=evidence["profile_sha256"])
            return point
        finally:
            if future.done():
                self.pending = None
