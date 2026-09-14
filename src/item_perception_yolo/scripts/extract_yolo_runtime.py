"""Build-time offline extraction of the exact private runtime and preserved licenses."""

import hashlib
import json
from pathlib import Path
import shutil
import sys
import zipfile


def extract(wheel_directory, output):
    manifest_path = wheel_directory / "item-yolo-runtime-lock.json"
    manifest = json.loads(manifest_path.read_text())
    if manifest["schema_version"] != 1:
        raise ValueError("Unsupported item runtime lock schema")
    for filename, expected in manifest["wheels"].items():
        if Path(filename).name != filename or not filename.endswith(".whl"):
            raise ValueError("Invalid locked wheel filename")
        path = wheel_directory / filename
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        if digest.hexdigest() != expected:
            raise ValueError(f"Wheel checksum mismatch: {filename}")
    output.mkdir(parents=True, exist_ok=True)
    for filename in manifest["wheels"]:
        with zipfile.ZipFile(wheel_directory / filename) as archive:
            for info in archive.infolist():
                if not (output / info.filename).resolve().is_relative_to(output.resolve()):
                    raise ValueError(f"Unsafe wheel member: {info.filename}")
            archive.extractall(output)
    shutil.copyfile(manifest_path, output / "runtime-lock.json")


if __name__ == "__main__":
    extract(Path(sys.argv[1]), Path(sys.argv[2]))
