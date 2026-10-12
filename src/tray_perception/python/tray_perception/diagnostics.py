"""Per-request tray performance and unlabelled review evidence; no native imports."""

from datetime import datetime, timedelta
import html
import json
import math
from pathlib import Path
import re
import time

from item_perception_yolo.daily_events import log_day, RETENTION_DAYS
from item_perception_yolo.item_detector import encode_rgb_png


SUCCESS_SAMPLE_INTERVAL = 10


def finite_json(value):
    if isinstance(value, dict):
        return {str(k): finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if value is None or type(value) in (str, bool, int, float):
        return value
    return str(value)


def prune_evidence(root, today):
    cutoff = today - timedelta(days=RETENTION_DAYS - 1)
    if not root.exists():
        return
    for day in root.iterdir():
        if day.is_symlink() or not day.is_dir():
            continue
        try:
            expired = datetime.strptime(day.name, "%Y-%m-%d").date() < cutoff
        except ValueError:
            continue
        if not expired:
            continue
        for request in day.iterdir():
            if (request.is_symlink() or not request.is_dir()
                    or not re.fullmatch(r"[a-f0-9]{32}", request.name)):
                continue
            for filename in ("raw.png", "overlay.svg", "detections.json"):
                target = request / filename
                if target.is_file() and not target.is_symlink():
                    target.unlink(missing_ok=True)
            if not any(request.iterdir()):
                request.rmdir()
        if not any(day.iterdir()):
            day.rmdir()


def save_evidence(root, request_id, view, detections, metadata):
    if view is None or not view.get("rgb"):
        return {"saved": False, "reason": "No fresh RGB observation available"}
    rgb = view["rgb"]
    directory = Path(root) / "logs/tray_perception/evidence" / str(log_day()) / request_id
    directory.mkdir(parents=True, exist_ok=False)
    raw = encode_rgb_png(rgb["rgb"], rgb["width"], rgb["height"])
    (directory / 'raw.png').write_bytes(raw)
    outlines = []
    for detection in detections:
        polygon = detection.get("polygon", [])
        if not polygon or any(len(p) != 2 or any(
                not isinstance(v, (float, int)) or not math.isfinite(v) for v in p)
                for p in polygon):
            continue
        color = "lime" if detection.get("valid") else "red"
        points = " ".join(f"{p[0]},{p[1]}" for p in polygon)
        label = html.escape(f"#{detection.get('source_index')} {detection.get('reason', '')}")
        outlines.append(f'<polygon points="{points}" fill="none" stroke="{color}" '
                        f'stroke-width="2"><title>{label}</title></polygon>')
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{rgb["width"]}" '
           f'height="{rgb["height"]}" viewBox="0 0 {rgb["width"]} {rgb["height"]}">'
           f'<image href="raw.png" width="{rgb["width"]}" height="{rgb["height"]}"/>'
           + ''.join(outlines) + '</svg>')
    (directory / 'overlay.svg').write_text(svg, encoding='utf-8')
    (directory / 'detections.json').write_text(json.dumps(finite_json({
        **metadata, "detections": detections, "ground_truth": None}), indent=2), encoding='utf-8')
    return {"saved": True, "raw_path": str(directory / 'raw.png'),
            "overlay_path": str(directory / 'overlay.svg'),
            "detections_path": str(directory / 'detections.json')}


class TrayRequestDiagnostics:
    def __init__(self, node, request, request_id, started, simulated):
        self.node, self.request, self.request_id = node, request, request_id
        self.started, self.simulated = started, simulated
        self.phase = "validate_request"
        self.phase_started = time.monotonic()
        self.timings = {}
        self.settings = None
        self.sampling = None
        self.view = None
        self.depth = None

    def phase_changed(self, phase):
        now = time.monotonic()
        key = self.phase + "_ms"
        self.timings[key] = self.timings.get(key, 0.) + (now - self.phase_started) * 1000
        self.phase, self.phase_started = phase, now

    def finish(self, response, view, sample_success):
        self.phase_changed("diagnostics")
        view = view or self.view
        result = {} if view is None else view.get("result", view.get("diagnostic_result", {}))
        worker = {} if view is None else view.get(
            "diagnostic_timings_ms", result.get("timings_ms", {}))
        detections = result.get("detections", [])
        usable = bool(response.success and response.found and (
            not self.request.sample_placement_depth or response.placement.valid))
        status = response.status
        if usable:
            category = "usable"
        elif status == "NO_VALID_PLACEMENT_DEPTH" or self.phase_before_finish == "placement_depth":
            category = "placement_depth"
        elif status == "NO_VALID_TRAY":
            category = "no_detection" if result.get("count") == 0 else "tray_geometry_or_filter"
        else:
            category = self.phase_before_finish
        depth = dict(self.depth or {})
        if "accepted_samples" in depth and "total_samples" in depth:
            depth["valid_fraction"] = (depth["accepted_samples"] / depth["total_samples"]
                                       if depth["total_samples"] else 0.)
        rgb = {} if view is None else view.get("rgb", {})
        payload = finite_json({
            "request_id": self.request_id, "batch_id": response.batch_id,
            "source": "simulate" if self.simulated else "service",
            "status": status, "response_message": response.message, "usable": usable,
            "category": category, "detected_count": result.get("count"),
            "valid_count": sum(bool(d.get("valid")) for d in detections),
            "detections": detections, "placement_depth": depth,
            "placement_required": bool(self.request.sample_placement_depth),
            "source_stamp_ns": rgb.get("stamp_ns"),
            "depth_stamp_ns": None if view is None or view.get("depth") is None
            else view["depth"].get("stamp_ns"),
            "camera_resolution": [rgb.get("width"), rgb.get("height")],
            "profile_sha256": self.request.profile_sha256,
            "model_sha256": (self.node.model or {}).get("sha256"),
            "camera_sha256": getattr(self.node.camera, "sha256", None),
            "settings": self.settings, "sampling": self.sampling, "device": "cpu",
            "save_debug_images": bool(self.request.save_debug_images),
            "worker_timings_ms": worker,
        })
        evidence_started = time.monotonic()
        evidence = {"saved": False, "reason": "Successful request not sampled"}
        if not self.simulated and (not usable or sample_success):
            try:
                evidence = save_evidence(self.node.root, self.request_id, view, detections, payload)
            except (OSError, ValueError, TypeError) as exc:
                evidence = {"saved": False, "error": str(exc)}
        payload["evidence"] = evidence
        self.timings["evidence_save_ms"] = (time.monotonic() - evidence_started) * 1000
        payload["timings_ms"] = {**self.timings, "total_ms": (time.monotonic()-self.started)*1000}
        diagnostics = json.loads(response.diagnostics_json or '{}')
        diagnostics.update(request_id=self.request_id, timings_ms=payload["timings_ms"],
                           worker_timings_ms=worker, diagnostic_evidence=evidence,
                           rejection_category=category, depth_diagnostics=depth)
        response.diagnostics_json = json.dumps(finite_json(diagnostics), allow_nan=False)
        self.node.events.record("INFO" if usable else "WARNING", "tray_request_diagnostics",
                                "Tray request timing and unlabelled quality evidence", **payload)

    def complete(self, response, view, sample_success):
        self.phase_before_finish = self.phase
        self.finish(response, view, sample_success)
