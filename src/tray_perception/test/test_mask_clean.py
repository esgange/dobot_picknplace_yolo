"""Synthetic geometry checks run with the actual pinned mask/OpenCV runtime."""

import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix
import pytest


def exercise_masks():
    import cv2
    import numpy as np
    import torch
    from ultralytics.engine.results import Results
    from tray_perception.mask_clean import clean_mask, clean_objects, validate_evidence

    rectangle = np.zeros((120, 160), np.uint8)
    rectangle[20:100, 20:100] = 1
    cleaned, original, evidence = clean_mask(rectangle, cv2, np)
    assert np.array_equal(cleaned, rectangle) and np.array_equal(original, rectangle)
    assert evidence["status"] == "unchanged" and evidence["retained_fraction"] == 1
    for bridge_width in (0, 1, 2):
        mask = rectangle.copy()
        mask[40:55, 130:145] = 1
        mask[45:45 + bridge_width, 100:130] = 1
        cleaned, _, evidence = clean_mask(mask, cv2, np)
        assert np.array_equal(cleaned, rectangle)
        assert evidence["components_before"] == (1 if bridge_width else 2)
        assert evidence["components_after"] == 2 and evidence["status"] == "cleaned"
        assert evidence["removed_pixels"] == 225 + 30 * bridge_width

    # Pixel area (including an inclusive 80% boundary), not contour vertex count.
    mask = np.zeros((120, 160), np.uint8)
    mask[10:30, 10:30] = 1
    mask[40:50, 40:50] = 1
    _, _, evidence = clean_mask(mask, cv2, np)
    assert evidence["status"] == "cleaned" and evidence["retained_fraction"] == .8
    mask[60, 60] = 1
    _, _, evidence = clean_mask(mask, cv2, np)
    assert evidence["status"] == "rejected" and "No dominant region" in evidence["reason"]
    mask[40:60, 40:60] = 1
    _, _, evidence = clean_mask(mask, cv2, np)
    assert evidence["status"] == "rejected"
    for width in (0, 1, 2):
        mask = np.zeros((120, 160), np.uint8)
        mask[30:30 + width, 10:100] = 1
        cleaned, _, evidence = clean_mask(mask, cv2, np)
        assert not cleaned.any() and evidence["status"] == "rejected"
    for invalid in (np.ones((3, 3, 1)), np.full((3, 3), np.nan), np.full((3, 3), .5)):
        with pytest.raises(RuntimeError, match="finite binary"):
            clean_mask(invalid, cv2, np)

    def result(masks, shape=(120, 160)):
        return Results(np.zeros((*shape, 3), np.uint8), "synthetic", {0: "tray", 1: "other"},
                       boxes=torch.tensor([[20, 20, 100, 100, .9, i % 2]
                                           for i in range(len(masks))]),
                       masks=torch.tensor(np.asarray(masks)))

    # An artificial polygon join cannot appear: the two raw regions stay separate.
    mask = rectangle.copy()
    mask[40:55, 130:145] = 1
    outputs = clean_objects(result([mask, rectangle]), {0: "tray", 1: "other"}, 100, cv2, np)
    assert [o["index"] for o in outputs] == [0, 1]
    assert [o["class_name"] for o in outputs] == ["tray", "other"]
    assert [o["class_id"] for o in outputs] == [0, 1]
    for obj in outputs:
        validate_evidence(obj["mask_clean"])
        assert np.array_equal(obj["polygon"].min(axis=0), [20, 20])
        assert np.array_equal(obj["polygon"].max(axis=0), [99, 99])

    ambiguous = np.zeros_like(rectangle)
    ambiguous[20:50, 20:50] = ambiguous[20:50, 100:130] = 1
    output = clean_objects(result([ambiguous]), {0: "tray"}, 1, cv2, np)[0]
    assert output["mask_clean"]["status"] == "rejected" and not len(output["polygon"])
    validate_evidence(output["mask_clean"])
    with pytest.raises(RuntimeError, match="count"):
        clean_objects(result([rectangle, rectangle]), {0: "tray", 1: "other"}, 1, cv2, np)
    malformed = result([rectangle])
    malformed.masks.data = torch.ones((2, 120, 160))
    with pytest.raises(RuntimeError, match="dimensions/count"):
        clean_objects(malformed, {0: "tray"}, 10, cv2, np)
    malformed.masks = None
    with pytest.raises(RuntimeError, match="absent"):
        clean_objects(malformed, {0: "tray"}, 10, cv2, np)

    # Correct letterbox inverse: 1280×720 RGB padded into a 640×640 mask.
    mask = np.zeros((640, 640), np.uint8)
    mask[200:301, 100:201] = 1
    output = clean_objects(result([mask], (720, 1280)), {0: "tray"}, 1, cv2, np)[0]
    assert np.array_equal(output["polygon"].min(axis=0), [200, 120])
    assert np.array_equal(output["polygon"].max(axis=0), [400, 320])


def exercise_pipeline():
    import cv2
    import numpy as np
    import torch
    from ultralytics.engine.results import Results
    from tray_perception import native

    rgb = np.zeros((480, 640, 3), np.uint8)
    mask = np.zeros((480, 640), np.uint8)
    mask[100:401, 100:301] = 1
    mask[200:220, 450:470] = 1
    mask[210, 301:450] = 1
    names = {0: "tray"}

    def result(value):
        return Results(rgb, "synthetic", names,
                       boxes=torch.tensor([[100, 100, 470, 401, .9, 0]]),
                       masks=torch.tensor(value[None]))

    camera = {"width": 640, "height": 480,
              "k": [500., 0., 320., 0., 500., 240., 0., 0., 1.],
              "d": [0.] * 5, "distortion_model": "plumb_bob"}
    context = {"camera": camera, "depth_camera": camera,
               "base_from_optical": np.eye(4).tolist()}
    plane_matrix = np.eye(4)
    plane_matrix[2, 3] = 1.
    plane = {"base_from_plane": plane_matrix.tolist(),
             "corners_base_m": [[-.5, -.4, 1.], [.5, -.4, 1.], [.5, .4, 1.], [-.5, .4, 1.]]}
    settings = {"accepted_class_ids": [0], "model_task": "segment", "geometry_source": "mask",
                "geometry": {"length_mm": 600., "width_mm": 400., "tolerance_mm": 1.},
                "yolo": {"class_ids": [0], "confidence": .5, "iou": .7,
                         "max_detections": 100, "image_size": 640}}
    request = {"settings": settings, "plane": plane, "camera_context": context}
    live, pixels = native.predict_trays(request, result(mask), rgb, names, cv2, np)
    simulated, _ = native.predict_trays({**request, "returned_only": True}, result(mask),
                                        rgb, names, cv2, np)
    assert simulated == live and live["count"] == 1
    chosen = live["selected"]
    assert chosen["valid"] and chosen["mask_clean"]["status"] == "cleaned"
    assert np.isclose(chosen["length_mm"], 600.) and np.isclose(chosen["width_mm"], 400.)
    assert np.frombuffer(pixels, np.uint8).reshape(rgb.shape)[215, 458].sum() == 0
    visual_request = {"width": 640, "height": 480, "camera_context": context,
                      "pixels": [], "cloud": False}
    data = rgb.tobytes() + np.full((480, 640), 800, "<u2").tobytes()
    _, clean_depth = native.tray_visuals(
        {**visual_request, "detections": live["detections"]}, data, cv2, np)
    _, base_depth = native.tray_visuals({**visual_request, "detections": []}, data, cv2, np)
    at = (215 * 640 + 458) * 3
    assert clean_depth[at:at + 3] == base_depth[at:at + 3]

    # A connected tail reaching the image edge must still refuse a tray pose.
    mask[:, 301:] = 0
    mask[200, :100] = 1
    clipped, _ = native.predict_trays(request, result(mask), rgb, names, cv2, np)
    assert clipped["selected"] is None
    evidence = clipped["detections"][0]
    assert evidence["mask_clean"]["source_touches_image_edge"]
    assert min(p[0] for p in evidence["polygon"]) == 100  # Tail was removed.
    assert "touches image edge" in evidence["reason"]
    # A detached small edge satellite is removable noise, not a clipped main tray.
    mask[200, :100] = 0
    mask[200:205, :5] = 1
    detached, _ = native.predict_trays(request, result(mask), rgb, names, cv2, np)
    assert detached["selected"] is not None

    # Two large blobs are rejected before rectangle fitting or any axes drawing.
    mask[100:401, 350:551] = 1
    rejected, pixels = native.predict_trays(
        {**request, "plane": None}, result(mask), rgb, names, cv2, np)
    assert rejected["selected"] is None
    assert rejected["detections"][0]["polygon"] == []
    assert rejected["detections"][0]["reason"].startswith("Detection Mask Clean:")
    assert not any(pixels)
    _, pixels = native.tray_visuals(
        {**visual_request, "detections": rejected["detections"]}, data, cv2, np)
    assert pixels == base_depth

    # OBB predictions keep their existing geometry and bypass mask cleanup.
    obb = Results(rgb, "synthetic", names,
                  obb=torch.tensor([[200, 250, 200, 300, 0, .9, 0]]))
    obb_settings = {**settings, "model_task": "obb", "geometry_source": "obb"}
    obb_result, _ = native.predict_trays(
        {**request, "settings": obb_settings}, obb, rgb, names, cv2, np)
    assert obb_result["selected"] is not None
    assert "mask_clean" not in obb_result["selected"]


@pytest.mark.parametrize("exercise", ["exercise_masks", "exercise_pipeline"])
def test_private_detection_mask_clean(exercise):
    runtime = Path(get_package_prefix("item_perception_yolo")) / \
        "lib/item_perception_yolo/yolo_runtime"
    command = ("import sys,runpy; sys.path.insert(0,sys.argv[1]); "
               "runpy.run_path(sys.argv[2])[sys.argv[3]]()")
    result = subprocess.run(["/usr/bin/python3", "-c", command, str(runtime), __file__, exercise],
                            env=dict(os.environ, OPENBLAS_NUM_THREADS="1"),
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
