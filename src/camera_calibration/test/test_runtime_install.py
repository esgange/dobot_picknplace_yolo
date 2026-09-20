import importlib.util
from pathlib import Path

import pytest


def installer():
    path = Path(__file__).parents[1] / "scripts/install_opencv_runtime.py"
    spec = importlib.util.spec_from_file_location(
        "opencv_runtime_installer_fixture", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.install_runtime


def test_native_runtime_install_replaces_symlinks_with_private_copies(tmp_path):
    source = tmp_path / "build" / "opencv_runtime"
    (source / "cv2").mkdir(parents=True)
    dist_info = source / "opencv_python-4.10.0.84.dist-info"
    dist_info.mkdir()
    (source / "cv2/__init__.py").write_text("# pinned loader")
    (source / "cv2/cv2.abi3.so").write_bytes(b"native fixture")
    (dist_info / "LICENSE.txt").write_text("OpenCV license fixture")
    (dist_info / "LICENSE-3RD-PARTY.txt").write_text(
        "third-party license fixture")
    (source / "stale.pyc").write_bytes(b"ignored")
    destination = tmp_path / "install" / "opencv_runtime"
    (destination / "cv2").mkdir(parents=True)
    (destination / "cv2/__init__.py").symlink_to(
        source / "cv2/__init__.py")
    (destination / "stale.txt").write_text("must be removed")

    installer()(source, destination)

    assert (destination / "cv2/__init__.py").read_text() == "# pinned loader"
    assert (destination / "cv2/cv2.abi3.so").read_bytes() == b"native fixture"
    assert not (destination / "cv2/__init__.py").is_symlink()
    assert not any(path.is_symlink() for path in destination.rglob("*"))
    assert not (destination / "stale.txt").exists()
    assert not (destination / "stale.pyc").exists()


@pytest.mark.parametrize("name", ["runtime", "", "."])
def test_native_runtime_install_rejects_broad_destination(tmp_path, name):
    source = tmp_path / "source"
    source.mkdir()
    destination = tmp_path / name if name else tmp_path
    with pytest.raises(ValueError, match="must end in opencv_runtime"):
        installer()(source, destination)
