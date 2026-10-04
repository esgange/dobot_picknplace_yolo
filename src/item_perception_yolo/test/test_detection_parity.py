"""Identical saved teaching/deployment inputs must produce identical pose batches."""

import copy
import shutil

import pytest
import yaml

from item_perception_yolo import item_detector as detector
from item_perception_yolo import item_teach_core as core
from test_item_detector import call, service_node  # noqa: F401
from test_item_teach import home, pair, settings  # noqa: F401


@pytest.mark.parametrize("valid_count", [0, 1, 3])
def test_saved_gui_and_headless_item_batches_match(
        pair, service_node, monkeypatch, valid_count):  # noqa: F811 - imported pytest fixtures
    root, _, path, profile = pair
    profile["model"]["declared_task"] = "segment"
    profile["geometry_source"] = "mask"
    path.write_text(yaml.safe_dump(profile))
    runtime = root / "runtime_teach"
    runtime.mkdir()
    for source in (path, path.with_suffix(".pt")):
        shutil.copy2(source, runtime / source.name)
    # Keep the real strict readers and source hashes; only sensors/inference are synthetic.
    monkeypatch.setattr(detector, "load_item_profile", lambda path, **kwargs:
                        core.load_item_profile(path, root=root,
                                               deployment=kwargs.get("deployment", False)))
    monkeypatch.setattr(detector, "file_sha256", core.file_sha256)
    node, candidate = service_node
    node.root = root
    node.settings = core.detection_settings(core.settings_from_profile(profile))
    node.model_config = {"sha256": profile["model"]["sha256"]}
    node.model_metadata = {"geometry_sources": ["mask"]}
    node.infer.return_value["metadata"]["candidates"] = [
        {**copy.deepcopy(candidate), "source_index": index} for index in range(valid_count)]
    responses = []
    for deployment in (False, True):
        node.deployment = deployment
        node.profile_path = runtime / path.name if deployment else path
        loaded, node.profile_digest = detector.ItemDetectNode._validate_pose_profile(
            node, node.profile_path)
        assert loaded == profile
        node.pose_candidates = loaded["retry"]["pose_candidates"]
        response = call(node, count=node.pose_candidates, digest=node.profile_digest)
        assert response.success
        assert response.status == {0: "NO_VALID_ITEMS", 1: "SHORTAGE", 3: "OK"}[valid_count]
        assert len(response.candidates) == valid_count
        responses.append(response)
    assert responses[0].batch_id != responses[1].batch_id
    # Request identity is deliberately unique; geometry, ranking, hashes and evidence match.
    for response in responses:
        response.batch_id = ""
        for candidate in response.candidates:
            candidate.id = candidate.id.split(":", 1)[1]
    assert responses[0] == responses[1]
    assert node._snapshot.call_count == node.infer.call_count == 2
    assert all(call.args[0] == 100_200_000_000 for call in node._snapshot.call_args_list)
    assert all(call.kwargs["candidate_limit"] == 3 for call in node.infer.call_args_list)
