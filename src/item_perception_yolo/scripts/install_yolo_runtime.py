"""Install the extracted native runtime as real files, never build-tree symlinks."""

from pathlib import Path
import shutil
import sys


REQUIRED_RUNTIME_FILES = (
    "runtime-lock.json",
    "cv2/__init__.py",
    "cv2/cv2.abi3.so",
)


def _validate_target(path):
    if path.name != "yolo_runtime" or path == Path(path.anchor):
        raise ValueError("Private runtime destination must end in yolo_runtime")


def install_runtime(source, destination):
    source = Path(source).resolve(strict=True)
    destination = Path(destination).absolute()
    _validate_target(destination)
    if source == destination:
        raise ValueError("Private runtime source and destination must differ")
    for relative in REQUIRED_RUNTIME_FILES:
        path = source / relative
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"Invalid extracted runtime file: {relative}")
    if destination.is_symlink() or destination.is_file():
        destination.unlink()
    elif destination.exists():
        shutil.rmtree(destination)
    shutil.copytree(
        source, destination, symlinks=False,
        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    linked = [str(path.relative_to(destination)) for path in destination.rglob("*")
              if path.is_symlink()]
    if linked:
        raise ValueError(f"Installed private runtime contains symlinks: {linked[:3]}")
    for relative in REQUIRED_RUNTIME_FILES:
        if not (destination / relative).is_file():
            raise ValueError(f"Installed runtime file is missing: {relative}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: install_yolo_runtime.py SOURCE DESTINATION")
    install_runtime(sys.argv[1], sys.argv[2])
