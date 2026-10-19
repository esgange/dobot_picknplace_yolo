"""Metadata fences around explicitly validated, immutable perception sources."""

from pathlib import Path
import stat


def fingerprints(paths):
    result = []
    for path in paths:
        path = Path(path).expanduser().absolute()
        info = path.lstat()
        if not stat.S_ISREG(info.st_mode) or any(p.is_symlink() for p in path.parents):
            raise ValueError(f"Perception source must be an ordinary file: {path}")
        result.append((str(path), info.st_dev, info.st_ino, info.st_size,
                       info.st_mtime_ns, info.st_ctime_ns))
    return tuple(result)


def check(binding, label="item"):
    if fingerprints(binding["paths"]) != binding["fingerprints"]:
        raise ValueError(f"Bound {label} source changed; explicitly reload/restart before capture")
