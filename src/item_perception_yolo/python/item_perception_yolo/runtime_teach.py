"""Strict flat runtime teach catalog shared by headless production consumers."""

from dataclasses import dataclass
from pathlib import Path


RUNTIME_PREFIXES = {
    "item_teach": "item_teach_",
    "bin_teach": "bin_teach_",
    "tray_teach": "tray_teach_",
}


@dataclass(frozen=True)
class RuntimeTeachCatalog:
    directory: Path
    item_yaml: Path
    item_model: Path
    bin_yaml: Path


def _artifact_kind(name):
    matches = [kind for kind, prefix in RUNTIME_PREFIXES.items()
               if name.startswith(prefix)]
    if len(matches) != 1:
        raise ValueError(
            f"Unsupported runtime_teach filename prefix: {name}; expected "
            "item_teach_, bin_teach_, or reserved tray_teach_")
    return matches[0]


def _require_one(paths, label):
    if not paths:
        raise ValueError(f"Missing required {label} in runtime_teach/")
    if len(paths) > 1:
        filenames = ", ".join(path.name for path in paths)
        raise ValueError(
            f"Multiple {label} files in runtime_teach/: {filenames}")
    return paths[0]


def runtime_teach_catalog(root):
    """Select exactly one deployed Item pair and Bin YAML by filename prefix."""
    directory = Path(root).resolve() / "runtime_teach"
    if directory.is_symlink() or not directory.is_dir():
        raise ValueError("Required flat root runtime_teach/ directory is missing or symlinked")
    artifacts = {"item_teach": {".yaml": [], ".pt": []},
                 "bin_teach": {".yaml": []}}
    for path in sorted(directory.iterdir()):
        if path.name.startswith("."):
            continue
        if path.is_symlink() or not path.is_file():
            raise ValueError("runtime_teach/ requires regular files only; no partitions/symlinks")
        kind = _artifact_kind(path.name)
        if kind == "tray_teach":
            raise ValueError(
                f"tray_teach_ is reserved for a future artifact and is not supported: {path.name}")
        if path.suffix not in artifacts[kind]:
            allowed = ", ".join(sorted(artifacts[kind]))
            raise ValueError(
                f"Unsupported {kind}_ runtime file extension: {path.name}; expected {allowed}")
        artifacts[kind][path.suffix].append(path.resolve())
    item_yaml = _require_one(
        artifacts["item_teach"][".yaml"], "item_teach_ YAML")
    item_model = _require_one(
        artifacts["item_teach"][".pt"], "item_teach_ PT model")
    bin_yaml = _require_one(
        artifacts["bin_teach"][".yaml"], "bin_teach_ YAML")
    if item_yaml.stem != item_model.stem:
        raise ValueError("runtime_teach/ Item Teach YAML/PT must have the same filename stem")
    return RuntimeTeachCatalog(directory.resolve(), item_yaml, item_model, bin_yaml)
