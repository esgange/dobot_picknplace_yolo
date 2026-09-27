# Tray runtime attribution

Tray Perception reuses the pinned private YOLO/OpenCV runtime installed by
`item_perception_yolo`. It does not redistribute a second runtime or change its
upstream dependencies. Preserve the exact wheel metadata and all attribution in
[`item_perception_yolo/NOTICE.md`](../item_perception_yolo/NOTICE.md) and
`third_party/wheels/item-yolo-runtime-lock.json`.

The shared worker uses Ultralytics 8.4.150 (AGPL-3.0), OpenCV 4.10.0, NumPy
1.26.4 and the exact separately provisioned Torch/torchvision runtime from that
lock. This package's BSD-3-Clause license does not replace dependency licenses.
