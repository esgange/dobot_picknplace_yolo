"""Bounded native-worker caches of camera-only math, never depth or eligibility."""

from collections import OrderedDict


_RAYS = OrderedDict()
_MAPPINGS = OrderedDict()


def camera_key(camera):
    return (camera.get("width"), camera.get("height"), camera.get("distortion_model"),
            tuple(camera["k"]), tuple(camera["d"]))


def clear_projection_caches():
    _RAYS.clear()
    _MAPPINGS.clear()


def _remember(cache, key, value):
    value.setflags(write=False)
    cache[key] = value
    cache.move_to_end(key)
    while len(cache) > 2:
        cache.popitem(last=False)
    return value


def _indices(pixels, camera, np):
    width, height = camera.get("width"), camera.get("height")
    if not width or not height or not len(pixels):
        return None
    points = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    if (not np.isfinite(points).all() or not np.equal(points, np.floor(points)).all()
            or np.any(points < 0) or np.any(points[:, 0] >= width)
            or np.any(points[:, 1] >= height)):
        return None
    return points[:, 1].astype(int) * width + points[:, 0].astype(int)


def raw_rays(pixels, camera, cv2, np):
    xy = cv2.undistortPoints(np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2),
                             np.asarray(camera["k"]).reshape(3, 3),
                             np.asarray(camera["d"]))[:, 0, :]
    return np.column_stack((xy, np.ones(len(xy))))


def cached_rays(pixels, camera, cv2, np):
    key = camera_key(camera)
    indices = _indices(pixels, camera, np)
    if indices is None or (len(indices) < 128 and key not in _RAYS):
        return raw_rays(pixels, camera, cv2, np)
    if key not in _RAYS:
        yy, xx = np.indices((camera["height"], camera["width"]))
        _remember(_RAYS, key, raw_rays(np.column_stack((xx.ravel(), yy.ravel())),
                                       camera, cv2, np))
    _RAYS.move_to_end(key)
    return _RAYS[key][indices]


def cached_mapping(pixels, source, target, cv2, np):
    key = (camera_key(source), camera_key(target))
    indices = _indices(pixels, source, np)
    use_cache = indices is not None and (len(indices) >= 128 or key in _MAPPINGS)
    if use_cache and key in _MAPPINGS:
        _MAPPINGS.move_to_end(key)
        return _MAPPINGS[key][indices]
    if use_cache:
        yy, xx = np.indices((source["height"], source["width"]))
        pixels = np.column_stack((xx.ravel(), yy.ravel()))
    directions = cached_rays(pixels, source, cv2, np)
    result, _ = cv2.projectPoints(directions, np.zeros(3), np.zeros(3),
                                  np.asarray(target["k"]).reshape(3, 3), np.asarray(target["d"]))
    result = result[:, 0, :]
    if not np.isfinite(result).all() or np.any(np.abs(result) > 2_000_000_000):
        raise ValueError("RGB/depth ray projection exceeds safe coordinate range")
    return _remember(_MAPPINGS, key, result)[indices] if use_cache else result
