"""Native-only CPU inference. Import exclusively from item_preview_worker."""

import hashlib
import importlib
import importlib.metadata
import math
import os
from pathlib import Path
import time

from .preview_protocol import receive_packet, send_packet


def initialize(runtime, manifest):
    if manifest["schema_version"] != 1:
        raise RuntimeError("Unsupported YOLO runtime manifest")
    for name, version in manifest["private_packages"].items():
        distribution = importlib.metadata.distribution(name)
        if distribution.version != version or not Path(distribution.locate_file("")).resolve(
        ).is_relative_to(runtime):
            raise RuntimeError(f"Private package runtime mismatch: {name}=={version}")
    for name, version in manifest["system_packages"].items():
        if importlib.metadata.version(name) != version:
            raise RuntimeError(f"Required preprovisioned dependency: {name}=={version}")
    import cv2
    import numpy as np
    import torch
    import torchvision
    import ultralytics
    if cv2.__version__ != "4.10.0" or np.__version__ != "1.26.4":
        raise RuntimeError("OpenCV/NumPy runtime mismatch")
    for module in (cv2, np, ultralytics):
        if not Path(module.__file__).resolve().is_relative_to(runtime):
            raise RuntimeError(f"Private module leakage: {module.__file__}")
    if torch.__version__ != "2.13.0+cu130" or torchvision.__version__ != "0.28.0+cu130":
        raise RuntimeError("Exact provisioned Torch/torchvision runtime mismatch")
    cv2.setNumThreads(1)
    cv2.ocl.setUseOpenCL(False)
    torch.set_num_threads(4)
    torch.set_num_interop_threads(1)
    ultralytics.settings.update({"sync": False})
    return cv2, np, torch, ultralytics


def _digest(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _fingerprint(path):
    info = path.stat()
    return info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns


def render_result(result, rgb, names, task, maximum, cv2, np):
    overlay = rgb.copy()
    boxes = result.obb if task == "obb" else result.boxes
    if boxes is None:
        raise RuntimeError("Detector result lacks the selected task output")
    classes = boxes.cls.cpu().numpy()
    scores = boxes.conf.cpu().numpy()
    if len(classes) > maximum or scores.shape != classes.shape:
        raise RuntimeError("Malformed detection classes/count")
    polygons = boxes.xyxyxyxy.cpu().numpy() if task == "obb" else None
    rects = boxes.xyxy.cpu().numpy() if task != "obb" else None
    if task == "segment" and len(classes):
        if result.masks is None or len(result.masks.xy) != len(classes):
            raise RuntimeError("Segmentation result is missing matching masks")
        from .item_geometry import shade_masks
        overlay = shade_masks(rgb, result.masks.xy, cv2, np)
    for i, (label, confidence) in enumerate(zip(classes, scores)):
        if (not math.isfinite(float(label)) or int(label) != label or int(label) not in names
                or not math.isfinite(float(confidence)) or not 0 <= confidence <= 1):
            raise RuntimeError("Malformed detection class/confidence")
        if polygons is not None:
            points = polygons[i]
            if points.shape != (4, 2) or not np.isfinite(points).all():
                raise RuntimeError("Malformed OBB geometry")
        else:
            rectangle = rects[i]
            if rectangle.shape != (4,) or not np.isfinite(rectangle).all():
                raise RuntimeError("Malformed detection rectangle")
    return overlay, len(classes)


def serve(input_stream, output_stream, runtime, manifest, scratch):
    try:
        cv2, np, torch, ultralytics = initialize(runtime, manifest)
        send_packet(output_stream, {
            "state": "ready", "pid": os.getpid(), "device": "cpu",
            "ultralytics": ultralytics.__version__, "module": ultralytics.__file__,
            "opencv": cv2.__version__, "numpy": np.__version__, "torch": torch.__version__,
            "opencv_threads": cv2.getNumThreads(), "opencl": cv2.ocl.useOpenCL(),
            "torch_threads": torch.get_num_threads(), "wheel_hashes": manifest["wheels"],
        })
        model = None
        selected = None
        fingerprint = None
        while True:
            request, data = receive_packet(input_stream)
            if request.get("operation") not in (
                    "detect", "inspect", "preview", "overlay_roi", "selected_pose"):
                raise RuntimeError("Unsupported preview operation")
            if cv2.getNumThreads() != 1 or cv2.ocl.useOpenCL() or torch.get_num_threads() != 4:
                raise RuntimeError("Native thread/OpenCL runtime drift")
            if request["operation"] == "selected_pose":
                from .item_geometry import selected_pose
                from .item_teach_core import validate_detection_settings
                width, height = request["width"], request["height"]
                if (type(width) is not int or type(height) is not int
                        or not 0 < width <= 4096 or not 0 < height <= 4096
                        or len(data) != width * height * 5):
                    raise RuntimeError("Malformed selected RGB/depth snapshot")
                validate_detection_settings(request["settings"], geometry_required=True)
                rgb_bytes = width * height * 3
                rgb = np.frombuffer(data[:rgb_bytes], np.uint8).reshape(height, width, 3)
                depth = np.frombuffer(data[rgb_bytes:], "<u2").reshape(height, width)
                _rgb, depth_view, candidates, rejected = selected_pose(
                    request["detection"], rgb, depth, request["context"],
                    request["settings"], cv2, np, display_detections=request["display_detections"])
                send_packet(output_stream, {
                    "state": "ok", "generation": request["generation"],
                    "width": width, "height": height,
                    "candidates": candidates, "rejected": rejected}, depth_view.tobytes())
                continue
            if request["operation"] == "overlay_roi":
                # Pure geometry: no model loading, prediction or production settings.
                from .item_geometry import draw_bin_roi
                width, height = request["width"], request["height"]
                if (type(width) is not int or type(height) is not int
                        or not 0 < width <= 4096 or not 0 < height <= 4096
                        or len(data) != width * height * 3):
                    raise RuntimeError("Malformed ROI RGB frame")
                overlay = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3).copy()
                status = draw_bin_roi(overlay, request["context"], request["error"], cv2, np)
                response = {"state": "ok", "generation": request["generation"],
                            "width": width, "height": height, "roi_overlay": status}
                send_packet(output_stream, response, overlay.tobytes())
                continue
            config = request["model"]
            from .item_preview import validate_preview_settings
            if request["operation"] in ("detect", "preview"):
                validate_preview_settings(config["task"], config["yolo"])
            path = Path(config["path"]).resolve(strict=True)
            key = (str(path), config["sha256"])
            if key != selected:
                if path.suffix != ".pt" or _digest(path) != config["sha256"]:
                    raise RuntimeError("Selected local model SHA-256 mismatch")
                before = _fingerprint(path)
                model = ultralytics.YOLO(str(path))
                if model.task not in ("detect", "segment", "obb"):
                    raise RuntimeError(f"Unsupported model task: {model.task}")
                if _fingerprint(path) != before or _digest(path) != config["sha256"]:
                    raise RuntimeError("Model changed while loading")
                selected, fingerprint = key, before
            if _fingerprint(path) != fingerprint:
                raise RuntimeError("Loaded model changed on disk; no silent reload")
            names = model.names
            if type(names) is not dict or any(type(v) is not str for v in names.values()):
                raise RuntimeError("Model lacks class-name metadata")
            if request["operation"] == "inspect":
                if data:
                    raise RuntimeError("Model inspection does not accept images")
                sources = {"segment": ["mask"], "obb": ["obb"], "detect": []}[model.task]
                send_packet(output_stream, {"state": "ok", "task": model.task,
                                            "sha256": config["sha256"], "classes": names,
                                            "geometry_sources": sources})
                continue
            if model.task != config["task"]:
                raise RuntimeError(f"Model task is {model.task}, selected {config['task']}")
            if any(class_id not in names for class_id in config["yolo"]["class_ids"]):
                raise RuntimeError(f"Selected class IDs not present in model; classes={names}")
            width, height = request["width"], request["height"]
            context = request.get("context")
            preview = request["operation"] == "preview"
            if preview and context is not None:
                raise RuntimeError("All-detections preview must not receive depth/pose context")
            has_depth = context is not None or (preview and request.get("preview_depth", False))
            expected_bytes = width * height * (5 if has_depth else 3)
            if (type(width) is not int or type(height) is not int or not 0 < width <= 4096
                    or not 0 < height <= 4096 or len(data) != expected_bytes):
                raise RuntimeError("Malformed RGB frame dimensions")
            rgb_bytes = width * height * 3
            rgb = np.frombuffer(data[:rgb_bytes], dtype=np.uint8).reshape(height, width, 3)
            bgr = np.ascontiguousarray(rgb[:, :, ::-1])
            settings = config["yolo"]
            start = time.monotonic()
            results = model.predict(
                source=bgr, device=torch.device("cpu"), imgsz=settings["image_size"],
                conf=settings["confidence"], iou=settings["iou"],
                max_det=settings["max_detections"], classes=settings["class_ids"],
                rect=False, batch=1, augment=False, agnostic_nms=False, nms=True, quantize=32,
                save=False, save_txt=False, save_crop=False, show=False, verbose=False,
                project=str(scratch), name="preview", exist_ok=True, stream=False,
            )
            if len(results) != 1 or tuple(results[0].orig_shape) != (height, width):
                raise RuntimeError("Detector result does not match the submitted frame")
            overlay, count = render_result(
                results[0], rgb, names, config["task"], settings["max_detections"], cv2, np,
            )
            available = []
            if results[0].masks is not None or model.task == "segment":
                available.append("mask")
            if results[0].obb is not None or model.task == "obb":
                available.append("obb")
            candidates, rejected, depth_view, detections = [], [], None, []
            roi_status = {"visible": False, "reason": "No calibrated bin ROI applied"}
            if preview:
                from .item_geometry import (
                    preview_detections, draw_pick_geometry, draw_bin_roi, classify_size,
                    render_depth, draw_depth_geometry,
                )
                source = request["geometry_source"]
                if source != "none" and source not in available:
                    raise RuntimeError("Selected preview geometry is unavailable")
                detections = preview_detections(
                    results[0], source, names, settings["max_detections"],
                    request["measurement_context"], request["measurement_error"], cv2, np,
                    diameter_mm=request["settings"]["pickdepth_radius"])
                for item in detections:
                    valid, reason = classify_size(item["measurement"],
                                                  request["settings"].get("geometry"))
                    item.update(size_valid=valid, size_reason=reason)
                    if source != "none":
                        color = ((0, 255, 0) if valid is True else
                                 (255, 0, 0) if valid is False else (180, 180, 180))
                        draw_pick_geometry(overlay, item["rectangle"], cv2, np, color=color)
                if request.get("preview_depth", False):
                    depth = np.frombuffer(data[rgb_bytes:], "<u2").reshape(height, width)
                    depth_view = render_depth(depth, request["settings"]["quality"], cv2, np)
                    draw_depth_geometry(depth_view, detections, source,
                                        request["depth_cameras"], request["measurement_context"],
                                        cv2, np)
                roi_status = draw_bin_roi(overlay, request["measurement_context"],
                                          request["measurement_error"], cv2, np)
            if context is not None:
                from .item_geometry import objects_from_result, generate_candidates, draw_bin_roi
                from .item_teach_core import validate_detection_settings
                validate_detection_settings(request["settings"], geometry_required=True)
                limit = request.get("candidate_limit")
                if limit is not None and (type(limit) is not int
                                          or not 1 <= limit <= settings["max_detections"]):
                    raise RuntimeError("Invalid requested candidate overlay count")
                objects = objects_from_result(results[0], request["settings"]["geometry_source"],
                                              names, settings["max_detections"], cv2, np)
                depth = np.frombuffer(data[rgb_bytes:], dtype="<u2").reshape(height, width)
                overlay, depth_view, candidates, rejected = generate_candidates(
                    objects, rgb, depth, context, request["settings"], cv2, np,
                    candidate_limit=limit)
                roi_status = draw_bin_roi(overlay, context, "", cv2, np)
            send_packet(output_stream, {
                "state": "ok", "generation": request["generation"], "width": width,
                "height": height, "count": count, "task": model.task,
                "inference_ms": round((time.monotonic() - start) * 1000, 1),
                "geometry_sources": available, "candidates": candidates, "rejected": rejected,
                "has_depth_view": depth_view is not None,
                "detections": detections,
                "roi_overlay": roi_status,
            }, overlay.tobytes() + (b"" if depth_view is None else depth_view.tobytes()))
    except EOFError:
        return
    except Exception as exc:
        send_packet(output_stream, {"state": "error", "error": f"{type(exc).__name__}: {exc}"})
        raise
