# Offline Python wheels

This directory contains exact runtime artifacts required to build the workspace
without network access.

`opencv_python-4.10.0.84-cp37-abi3-manylinux_2_17_x86_64.manylinux2014_x86_64.whl`
is the package-private OpenCV runtime used by `camera_calibration` and the
separately isolated item inference worker on the
project's absolute Ubuntu 22.04 x86-64 / Python 3.10 platform. The package build
verifies its SHA-256 digest from `SHA256SUMS` before extracting it into that
package's install prefix. It is never installed globally and is never selected
as a fallback for another package.

The wheel is distributed by the OpenCV on Wheels project under the MIT license
and contains its own `LICENSE.txt` plus `LICENSE-3RD-PARTY.txt`. Those files are
preserved inside the wheel and the extracted package installation.

## Item inference runtime

`item-yolo-runtime-lock.json` lists the exact private inference wheels and
SHA-256 hashes used by `item_perception_yolo`. Root/package builds verify and
extract these offline into that package's private prefix without global pip
installation. `SHA256SUMS` also lists every wheel. See the package's
[`NOTICE.md`](../../src/item_perception_yolo/NOTICE.md) for attribution.

The lock separately lists exact preprovisioned system dependencies. Torch,
torchvision and their full OS/Python/CUDA dependency closure are NOT contained
in this wheel directory; stage them on the offline target separately. No
complete offline deployment milestone is claimed by the item inference work.
