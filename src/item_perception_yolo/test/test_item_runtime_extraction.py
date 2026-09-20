import hashlib
import importlib.util
import json
from pathlib import Path
import zipfile

import pytest


def extractor():
    path = Path(__file__).parents[1] / "scripts/extract_yolo_runtime.py"
    spec = importlib.util.spec_from_file_location("extractor_fixture", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.extract


def installer():
    path = Path(__file__).parents[1] / "scripts/install_yolo_runtime.py"
    spec = importlib.util.spec_from_file_location("runtime_installer_fixture", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.install_runtime


def test_offline_extraction_missing_checksum_and_unsafe_member(tmp_path):
    wheel = tmp_path / "fixture.whl"
    with zipfile.ZipFile(wheel, "w") as archive:
        archive.writestr("runtime/LICENSE.txt", "Synthetic license fixture")
    manifest = {"schema_version": 1, "wheels": {wheel.name:
                hashlib.sha256(wheel.read_bytes()).hexdigest()}}
    lock = tmp_path / "item-yolo-runtime-lock.json"
    lock.write_text(json.dumps(manifest))
    extract = extractor()
    output = tmp_path / "extracted"
    extract(tmp_path, output)
    assert (output / "runtime/LICENSE.txt").read_text() == "Synthetic license fixture"
    wheel.write_bytes(wheel.read_bytes() + b"tamper")
    with pytest.raises(ValueError, match="checksum"):
        extract(tmp_path, tmp_path / "bad")
    assert not (tmp_path / "bad").exists()
    wheel.unlink()
    with pytest.raises(FileNotFoundError):
        extract(tmp_path, tmp_path / "missing")
    with zipfile.ZipFile(wheel, "w") as archive:
        archive.writestr("../escape", "not allowed")
    manifest["wheels"][wheel.name] = hashlib.sha256(wheel.read_bytes()).hexdigest()
    lock.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="Unsafe"):
        extract(tmp_path, tmp_path / "unsafe")
    assert not (tmp_path / "escape").exists()


def test_native_runtime_install_replaces_symlinks_with_private_copies(tmp_path):
    source = tmp_path / "build" / "yolo_runtime"
    (source / "cv2").mkdir(parents=True)
    (source / "runtime-lock.json").write_text('{"schema_version":1}')
    (source / "cv2/__init__.py").write_text("# pinned loader")
    (source / "cv2/cv2.abi3.so").write_bytes(b"native fixture")
    (source / "stale.pyc").write_bytes(b"ignored")
    destination = tmp_path / "install" / "yolo_runtime"
    (destination / "cv2").mkdir(parents=True)
    (destination / "cv2/__init__.py").symlink_to(source / "cv2/__init__.py")
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
    with pytest.raises(ValueError, match="must end in yolo_runtime"):
        installer()(source, destination)
