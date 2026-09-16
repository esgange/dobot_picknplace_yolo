"""Strict client-side validation for one fresh Item Detect candidate batch."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time

from item_perception_interfaces.srv import GetItemPoses

from .errors import FeedbackFailure, OperationCanceled


CANDIDATE_SERVICE = "/item_detect/get_item_poses"
CANONICAL_CANDIDATE_PROVIDERS = frozenset({
    ("item_detect", "/"),
    ("item_teach", "/"),
})


@dataclass(frozen=True)
class Candidate:
    identifier: str
    priority: int
    class_id: int
    confidence: float
    position_m: tuple
    quaternion: tuple


@dataclass(frozen=True)
class CandidateBatch:
    identifier: str
    observation_stamp_ns: int
    depth_stamp_ns: int
    candidates: tuple
    evidence: dict
    debug_message: str


class CandidateClient:
    def __init__(self, node, root, *, service=CANDIDATE_SERVICE):
        self.node = node
        self.root = Path(root).resolve()
        self.client = node.create_client(GetItemPoses, service)
        self.lock = threading.Lock()

    def close(self):
        self.node.destroy_client(self.client)

    def request(self, configuration, *, save_debug_images, cancel):
        if not self.lock.acquire(blocking=False):
            raise FeedbackFailure("A detector candidate request is already active")
        try:
            configuration.validate_sources(self.root)
            if configuration.selection is None:
                raise FeedbackFailure("Pick requires a configured Bin Teach")
            if not self.client.service_is_ready():
                raise FeedbackFailure("Canonical item_detect pose service is unavailable")
            self.node.check_detector_owner()
            profile = configuration.profile
            request = GetItemPoses.Request(
                max_candidates=configuration.pose_candidates,
                profile_sha256=configuration.profile_sha256,
                save_debug_images=bool(save_debug_images))
            self.node.events.record(
                "INFO", "candidate_request", "Requesting one fresh candidate batch",
                configuration_id=configuration.configuration_id,
                requested=configuration.pose_candidates,
                save_debug_images=bool(save_debug_images))
            future = self.client.call_async(request)
            deadline = time.monotonic() + profile["quality"]["request_timeout_sec"] + 1.0
            while not future.done():
                if cancel():
                    future.cancel()
                    raise OperationCanceled("Cancelled while awaiting item candidates")
                if time.monotonic() >= deadline:
                    future.cancel()
                    raise FeedbackFailure("Detector response timeout; no automatic retry")
                self.node.wait_control(0.02)
            result = future.result()
            configuration.validate_sources(self.root)
            if result is None or not result.success:
                status = "no response" if result is None else result.status
                message = "" if result is None else result.message
                raise FeedbackFailure(f"Detector {status}: {message}".strip())
            return self._validate_result(result, configuration, bool(save_debug_images))
        finally:
            self.lock.release()

    def _validate_result(self, result, configuration, save_debug_images):
        profile, selection = configuration.profile, configuration.selection
        try:
            evidence = json.loads(result.diagnostics_json)
        except (TypeError, json.JSONDecodeError) as exc:
            raise FeedbackFailure("Detector diagnostics are not valid JSON") from exc
        if result.header.frame_id != "platform_reference":
            raise FeedbackFailure("Detector result frame is not platform_reference")
        expected_evidence = {
            "profile_sha256": configuration.profile_sha256,
            "model_sha256": profile["model"]["sha256"],
            "camera_sha256": selection.station.camera.sha256,
            "platform_sha256": selection.station.platform.sha256,
            "bin_sha256": selection.bin.sha256,
        }
        for key, expected in expected_evidence.items():
            if evidence.get(key) != expected:
                raise FeedbackFailure(f"Detector/controller {key} mismatch")
        capture = evidence.get("debug_capture")
        if (type(capture) is not dict
                or set(capture) != {"requested", "rgb_path", "depth_path", "error"}
                or type(capture["requested"]) is not bool
                or any(type(capture[key]) is not str
                       for key in ("rgb_path", "depth_path", "error"))
                or capture["requested"] is not save_debug_images):
            raise FeedbackFailure("Detector debug-capture diagnostics are malformed")
        paths = (capture["rgb_path"], capture["depth_path"])
        debug_message = "OFF"
        if save_debug_images:
            if bool(capture["error"]) == bool(all(paths)):
                raise FeedbackFailure("Detector debug-capture result is inconsistent")
            if all(paths):
                directory = (self.root / "debug/pick_img").resolve()
                resolved = tuple(Path(path).resolve() for path in paths)
                if any(path.parent != directory or path.suffix != ".png" for path in resolved):
                    raise FeedbackFailure("Detector debug image escaped debug/pick_img")
                debug_message = "SAVED: " + " | ".join(map(str, resolved))
            else:
                debug_message = "SAVE WARNING: " + capture["error"]
        elif any(paths) or capture["error"]:
            raise FeedbackFailure("Detector saved unrequested debug images")
        now_ns = self.node.get_clock().now().nanoseconds
        stamps = tuple(value.sec * 1_000_000_000 + value.nanosec for value in
                       (result.header.stamp, result.depth_stamp))
        if any(stamp <= 0 or not 0 <= (now_ns - stamp) / 1e9 <=
               profile["quality"]["result_max_age_sec"] for stamp in stamps):
            raise FeedbackFailure("Returned detector observation is stale or future-dated")
        if abs(stamps[0] - stamps[1]) / 1e9 > profile["quality"]["sync_tolerance_sec"]:
            raise FeedbackFailure("Detector RGB/depth timestamps are not synchronized")
        if not result.batch_id or len(result.candidates) > configuration.pose_candidates:
            raise FeedbackFailure("Detector batch ID/count is invalid")
        found = []
        identifiers = set()
        previous_distance = -1.0
        for index, value in enumerate(result.candidates, 1):
            p, q = value.pose.position, value.pose.orientation
            numeric = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                       value.confidence, value.center_distance)
            if (not value.id or value.id in identifiers or value.priority != index
                    or value.class_id not in profile["yolo"]["class_ids"]
                    or not profile["yolo"]["confidence"] <= value.confidence <= 1.0
                    or value.center_distance < previous_distance
                    or not all(math.isfinite(number) for number in numeric)
                    or abs(sum(number * number for number in (q.x, q.y, q.z, q.w)) - 1.0)
                    > 1e-5):
                raise FeedbackFailure("Malformed, duplicate, or unordered detector candidate")
            identifiers.add(value.id)
            previous_distance = value.center_distance
            found.append(Candidate(
                value.id, value.priority, value.class_id, value.confidence,
                (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
        batch = CandidateBatch(result.batch_id, stamps[0], stamps[1], tuple(found),
                               evidence, debug_message)
        self.node.events.record(
            "INFO", "candidate_response", result.message,
            batch_id=batch.identifier, candidates=len(batch.candidates),
            configuration_id=configuration.configuration_id,
            debug_capture=debug_message)
        return batch
