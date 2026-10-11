"""Read-only, platform-XY repick exclusion geometry and wire validation."""

import math


REPICK_CONTRACT = "repick_exclusions_v1"
REPICK_DEFAULTS = {"blaclist_trigger": 0, "blackist_radius": 0.0, "blackist_cooldown": 0}
MAX_REGIONS = 1000


def contains(region, position):
    return math.hypot(position[0] - region["x_m"], position[1] - region["y_m"]) <= (
        region["radius_m"] + 1e-9)


def blocking_region(regions, position):
    return next((region for region in regions if contains(region, position)), None)


def region_dict(region):
    return {key: getattr(region, key) for key in (
        "id", "x_m", "y_m", "radius_m", "attempts", "cooldown_remaining")}


def validate_regions(regions, settings):
    if type(regions) is not list or len(regions) > MAX_REGIONS:
        raise ValueError("Invalid repick exclusion list")
    identifiers = set()
    for region in regions:
        if (type(region) is not dict or set(region) != {
                "id", "x_m", "y_m", "radius_m", "attempts", "cooldown_remaining"}
                or type(region["id"]) is not int or region["id"] <= 0
                or region["id"] in identifiers
                or any(type(region[key]) not in (float, int) or not math.isfinite(region[key])
                       for key in ("x_m", "y_m", "radius_m"))
                or region["radius_m"] <= 0
                or region["radius_m"] != settings["blackist_radius"] / 1000.
                or not settings["blaclist_trigger"]
                or type(region["attempts"]) is not int
                or region["attempts"] != settings["blaclist_trigger"]
                or type(region["cooldown_remaining"]) is not int
                or not 1 <= region["cooldown_remaining"] <= settings["blackist_cooldown"]):
            raise ValueError("Malformed or profile-mismatched repick exclusion")
        identifiers.add(region["id"])
    return regions


def rejection_reason(region):
    return (f"Repick area {region['id']} blocked: radius={region['radius_m'] * 1000:g} mm, "
            f"attempts={region['attempts']}, "
            f"cooldown={region['cooldown_remaining']} other picks")
