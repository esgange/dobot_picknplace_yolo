"""Saved opt-in settings and strict, source-bound request exclusions."""

import copy
import json

import pytest

from item_perception_interfaces.msg import RepickRegion
from item_perception_interfaces.srv import GetItemPoses
from item_perception_yolo import item_detector as detector, item_teach_core as core
from item_perception_yolo.item_teach_recovery import recover_item_fields
from item_perception_yolo.repick import REPICK_CONTRACT, capture_note, validate_regions
from test_item_detector import service_node  # noqa: F401
from test_item_teach import settings, home, pair  # noqa: F401


def region():
    return {"id": 1, "x_m": .1, "y_m": .2, "radius_m": .03,
            "attempts": 3, "cooldown_remaining": 5}


def enabled():
    return {"pose_candidates": 3, "blaclist_trigger": 3,
            "blackist_radius": 30., "blackist_cooldown": 5, "trickle_depth": 0.}


def test_capture_note_distinguishes_disabled_empty_active_and_local_simulation():
    assert capture_note(core.REPICK_DEFAULTS, []) == "Repick OFF"
    assert capture_note(enabled(), []) == "Repick ON: 0 active areas"
    assert capture_note(enabled(), [region()]) == "Repick: 1 active (purple)"
    assert capture_note(enabled(), [], simulated=True) == "Repick: controller history not applied"


def test_new_profiles_start_disabled_and_recover_schema13_without_changing_file(pair):  # noqa: F811
    root, _, path, profile = pair
    assert profile["retry"] == {"pose_candidates": 3, **core.REPICK_DEFAULTS}
    import yaml
    profile["schema_version"] = 13
    profile["retry"] = {"pose_candidates": 3}
    path.write_text(yaml.safe_dump(profile))
    before = path.read_bytes()
    with pytest.raises(ValueError, match="exactly 16"):
        core.load_item_profile(path, root=root)
    draft = recover_item_fields(path, root=root)
    assert all(draft.values[k] == v for k, v in core.REPICK_DEFAULTS.items())
    assert path.read_bytes() == before


@pytest.mark.parametrize("key,value", [
    ("blaclist_trigger", -1), ("blaclist_trigger", 1.5), ("blaclist_trigger", True),
    ("blackist_cooldown", 0), ("blackist_cooldown", 1001), ("blackist_radius", 0),
    ("blackist_radius", float("nan")), ("blackist_radius", float("inf")),
])
def test_enabled_repick_requires_explicit_valid_values(settings, key, value):  # noqa: F811
    settings["retry"] = enabled()
    settings["retry"][key] = value
    with pytest.raises(ValueError):
        core.validate_settings(settings)


def test_enabled_settings_round_trip_exact_spelling(pair):  # noqa: F811
    root, model, _, profile = pair
    fields = core.settings_from_profile(profile)
    fields["retry"] = enabled()
    path, _ = core.save_item_profile(fields, profile["home"], model, root=root)
    loaded, _ = core.load_item_profile(path, root=root)
    assert loaded["retry"] == enabled()


@pytest.mark.parametrize("step", [-1., float("nan"), float("inf"), True, "10"])
def test_trickle_depth_requires_finite_nonnegative_number(settings, step):  # noqa: F811
    settings["retry"] = {**enabled(), "trickle_depth": step}
    with pytest.raises(ValueError, match="trickle_depth"):
        core.validate_settings(settings)


def test_positive_trickle_requires_enabled_blacklist(settings):  # noqa: F811
    settings["retry"]["trickle_depth"] = 10.
    with pytest.raises(ValueError, match="requires enabled repick"):
        core.validate_settings(settings)


def test_schema14_recovery_keeps_blacklist_values_and_proposes_zero_trickle(pair):  # noqa: F811
    import yaml
    root, _, path, profile = pair
    profile["schema_version"] = 14
    profile["retry"] = enabled()
    del profile["retry"]["trickle_depth"]
    path.write_text(yaml.safe_dump(profile))
    before = path.read_bytes()
    draft = recover_item_fields(path, root=root)
    assert draft.values["trickle_depth"] == 0.
    for key in ("blaclist_trigger", "blackist_radius", "blackist_cooldown"):
        assert draft.values[key] == enabled()[key]
    assert path.read_bytes() == before


def test_schema15_round_trip_and_missing_trickle_stays_invalid(pair):  # noqa: F811
    root, model, _, profile = pair
    fields = core.settings_from_profile(profile)
    fields["retry"] = {**enabled(), "trickle_depth": 10.5}
    path, saved = core.save_item_profile(fields, profile["home"], model, root=root)
    loaded, _ = core.load_item_profile(path, root=root)
    assert loaded["schema_version"] == 16 and loaded["retry"]["trickle_depth"] == 10.5
    del saved["retry"]["trickle_depth"]
    import yaml
    path.write_text(yaml.safe_dump(saved))
    with pytest.raises(ValueError, match="trickle_depth"):
        core.load_item_profile(path, root=root)
    draft = recover_item_fields(path, root=root)
    assert draft.values["trickle_depth"] is None


@pytest.mark.parametrize("damage", ["disabled", "radius", "nan", "duplicate", "count", "cooldown"])
def test_untrusted_exclusions_must_match_profile(damage):
    retry, regions = enabled(), [region()]
    if damage == "disabled":
        retry["blaclist_trigger"] = 0
    elif damage == "radius":
        regions[0]["radius_m"] = .04
    elif damage == "nan":
        regions[0]["x_m"] = float("nan")
    elif damage == "duplicate":
        regions.append(copy.deepcopy(regions[0]))
    elif damage == "count":
        regions[0]["attempts"] = 2
    else:
        regions[0]["cooldown_remaining"] = 6
    with pytest.raises(ValueError):
        validate_regions(regions, retry)


@pytest.mark.parametrize("damage", [None, "contract", "bin", "platform", "radius"])
def test_provider_echoes_bound_exclusions_or_rejects_before_inference(
        service_node, damage):  # noqa: F811
    node, _ = service_node
    from item_perception_yolo.item_preview import CaptureMailbox
    node.capture_preview = CaptureMailbox()
    node._validate_pose_profile.return_value[0]["retry"] = enabled()
    request = GetItemPoses.Request(
        max_candidates=3, profile_sha256="a" * 64,
        pose_convention=GetItemPoses.Request.POSE_CONVENTION,
        repick_contract=REPICK_CONTRACT, blacklist_revision=8,
        blacklist_bin_sha256="e" * 64, blacklist_platform_sha256="d" * 64,
        blacklist_regions=[RepickRegion(**region())])
    if damage == "contract":
        request.repick_contract = ""
    elif damage in ("bin", "platform"):
        setattr(request, f"blacklist_{damage}_sha256", "wrong")
    elif damage == "radius":
        request.blacklist_regions[0].radius_m = .05
    result = node.infer.return_value["metadata"]
    result["candidates"] = []
    result["rejected"] = [{"source_index": 0, "rejection_stage": "blacklist",
                           "reason": "Repick area 1 blocked"}]
    response = detector.ItemDetectNode._request(node, request, GetItemPoses.Response())
    if damage:
        assert not response.success
        node.infer.assert_not_called()
    else:
        assert response.success and response.status == "NO_UNBLOCKED_ITEMS"
        evidence = json.loads(response.diagnostics_json)
        assert evidence["repick"]["regions"] == [region()]
        assert evidence["repick"]["revision"] == 8
        assert node.infer.call_args.args[2]["repick_regions"] == [region()]
        assert node.capture_preview.take()["view"]["repick_note"] == "Repick: 1 active (purple)"
