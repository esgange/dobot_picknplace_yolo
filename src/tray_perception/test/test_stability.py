"""Pose-pair boundaries and controller-verifiable evidence, without ROS or hardware."""
import copy
import math

import pytest

from tray_perception.stability import compare, validate_evidence


def pose(stamp=101, x=0., angle=0., width=190., length=275., cls=0):
    radians = math.radians(angle) / 2
    return {"stamp_ns": stamp, "class_id": cls, "position": [x, 0., 0.],
            "quaternion": [0., 0., math.sin(radians), math.cos(radians)],
            "width_mm": width, "length_mm": length}


TAUGHT = {"width_mm": 190., "length_mm": 275., "tolerance_mm": 20.}


@pytest.mark.parametrize("width", [100., 190., 300.])
@pytest.mark.parametrize("factor,accepted", [(0., True), (1., True), (1.00001, False)])
def test_translation_scales_with_taught_width_and_includes_boundary(width, factor, accepted):
    taught = {**TAUGHT, "width_mm": width, "length_mm": width * 2}
    a = pose(width=width, length=width*2)
    b = {**a, "stamp_ns": 102, "position": [width * .05 / 1000 * factor, 0., 0.]}
    proof = compare(a, b, taught, {"x_mm": 20., "y_mm": 30.})
    assert proof["accepted"] is accepted
    assert proof["translation_limit_mm"] == pytest.approx(width * .05)
    assert proof["differences"]["placement_mm"] == pytest.approx(width * .05 * factor)


@pytest.mark.parametrize("angle,accepted", [(5., True), (5.00001, False), (90., False), (180., False)])
def test_rotation_boundary_and_axis_flips(angle, accepted):
    # Small tray keeps center translation below its limit for the 5-degree case.
    a, b = pose(width=190., length=190.), pose(102, angle=angle, width=190., length=190.)
    taught = {**TAUGHT, "length_mm": 190.}
    # Rotate about the center so only the separately checked origin moves.
    c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
    b["position"] = [(.095 - .095*(c-s))/2, (.095 - .095*(s+c))/2, 0.]
    assert compare(a, b, taught)["accepted"] is accepted


def test_center_and_requested_placement_are_checked_separately():
    # Both dimensions are individually legal; their accumulated center displacement is not.
    a, b = pose(width=170., length=255.), pose(102, width=210., length=295.)
    proof = compare(a, b, TAUGHT)
    assert proof["differences"]["origin_mm"] == 0.
    assert not proof["accepted"]
    # No extra inter-frame dimension gate: allowed changes with a stable center pass.
    assert compare(pose(width=185.), pose(102, width=195.), TAUGHT)["accepted"]
    a, b = pose(), pose(102, angle=3.)
    assert compare(a, b, TAUGHT)["accepted"]
    proof = compare(a, b, TAUGHT, {"x_mm": 180., "y_mm": 265.})
    assert proof["differences"]["placement_mm"] > 9.5 and not proof["accepted"]


def test_quaternion_sign_is_equivalent_and_classes_must_match():
    a, b = pose(), pose(102)
    b["quaternion"] = [-v for v in b["quaternion"]]
    assert compare(a, b, TAUGHT)["accepted"]
    assert not compare(a, {**b, "class_id": 1}, TAUGHT)["accepted"]
    # Identical center with a different origin/axis convention must fail.
    b.update(position=[.19, .275, 0.], quaternion=[0., 0., 1., 0.])
    assert not compare(a, b, TAUGHT)["accepted"]


@pytest.mark.parametrize("stamp", [100, 101])
def test_nonadvancing_rgb_rejected(stamp):
    with pytest.raises(ValueError, match="advancing"):
        compare(pose(), pose(stamp), TAUGHT)


def test_both_dimensions_require_taught_tolerance():
    with pytest.raises(ValueError, match="dimensions"):
        compare(pose(width=169.), pose(102), TAUGHT)


@pytest.mark.parametrize("tamper", ["absent", "old", "cached", "returned", "limit", "timing", "version_bool", "accept_int"])
def test_controller_recomputes_evidence(tamper):
    a, b = pose(), pose(102, x=.001)
    proof = compare(a, b, TAUGHT)
    proof["timings_ms"] = {"first_prediction": 100., "second_prediction": 101., "search": 205.}
    validate_evidence(proof, b, TAUGHT, None, 100, 103)
    broken, returned = copy.deepcopy(proof), copy.deepcopy(b)
    if tamper == "absent":
        broken = None
    elif tamper == "old":
        broken["schema_version"] = 0
    elif tamper == "version_bool":
        broken["schema_version"] = True
    elif tamper == "accept_int":
        broken["accepted"] = 1
    elif tamper == "cached":
        broken["first"]["stamp_ns"] = 100
    elif tamper == "returned":
        returned["position"][0] += .001
    elif tamper == "limit":
        broken["translation_limit_mm"] = 10.
    else:
        broken["timings_ms"]["search"] = float("nan")
    with pytest.raises(ValueError):
        validate_evidence(broken, returned, TAUGHT, None, 100, 103)
