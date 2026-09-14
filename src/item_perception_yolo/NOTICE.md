# Item inference runtime attribution

The item worker uses the exact wheels listed with SHA-256 hashes in
`third_party/wheels/item-yolo-runtime-lock.json`. Build extraction preserves
each wheel's upstream license/notice metadata; do not remove that metadata.

- Ultralytics 8.4.150: https://github.com/ultralytics/ultralytics — AGPL-3.0.
- Ultralytics THOP 2.1.6: https://github.com/ultralytics/thop — AGPL-3.0.
- NumPy 1.26.4: https://numpy.org/ — BSD-3-Clause, with bundled-library notices.
- opencv-python 4.10.0.84: https://github.com/opencv/opencv-python — wheel package
  MIT; OpenCV Apache-2.0; retain bundled third-party notices (including Qt).
- cloudpickle 3.1.1: https://github.com/cloudpipe/cloudpickle — BSD-3-Clause.
- Polars and polars-runtime-32 1.35.2: https://github.com/pola-rs/polars — MIT.
- nvidia-ml-py 12.560.30: https://pypi.org/project/nvidia-ml-py/ — BSD license.

The project's package license does not replace these dependencies' licenses.
Review applicable upstream distribution and source obligations, particularly
AGPL-licensed inference components, when distributing/deploying this runtime.

The fixed CPU worker also requires the exact separately provisioned system
packages enumerated in the lock, including Torch 2.13.0+cu130 and torchvision
0.28.0+cu130. Their dependency closure is not bundled here. No global Python
installation, automatic network installer, GPU selection or runtime fallback
is performed by the application or build. This is not a complete offline OS
or Python dependency image.
