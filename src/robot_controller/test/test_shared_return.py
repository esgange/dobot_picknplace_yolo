"""Paused Return Item is the command-level reference for automatic dropped returns."""

import numpy as np
import pytest

from robot_controller.errors import FeedbackFailure, OperationCanceled
from robot_controller.item_return import ReturnPickBridge
from robot_controller.kinematics import pose_values
from robot_controller.motion import candidate_transit, pick_targets
from robot_controller.pick_session import PickSession
from test_placement_queue import HELD, RELEASE, OPEN
from test_queued_return import return_rig


def drop_rig(height=.8, *, neutral=False, more=False):
    rig, source = return_rig(height)
    node = rig.node
    if more:
        next_pose = source[3].matrix.copy()
        next_pose[0, 3] += .15
        plan = pick_targets(node.configuration.home_matrix, next_pose,
                            node.configuration.profile, 2)
        node.managed.session = PickSession(["dropped", "next"], [source, plan])
        node.managed.session.set_state(1, "ACTIVE")
        node.managed.session.set_state(1, "HELD")
    node.managed.session.set_state(1, "DROPPED")
    node.holding_item = False
    node.managed.kind = None
    node.managed.executing = False
    node.expected_outputs = {ch: bool((0 if neutral else HELD) & (1 << (ch - 1)))
                             for ch in (1, 2, 13, 14)}
    rig.emit(outputs=0 if neutral else HELD, inputs=0, running=0)
    rig.requests.clear()
    rig.order.clear()
    return rig, source


def at_return_retract(node):
    pose = node.managed.session.attempts[0].plan[2].matrix.copy()
    pose[2, 3] = node.configuration.home_matrix[2, 3]
    return dict(outputs=0, inputs=0, running=0,
                tool_vector_actual=pose_values(pose), currentCommandId=3)


@pytest.mark.parametrize("height", [.3, .8, 1.0])
@pytest.mark.parametrize("neutral", [False, True])
def test_drop_return_matches_every_paused_return_command_except_home(height, neutral):
    reference, _ = return_rig(height)
    reference.steps = iter([dict(outputs=RELEASE, inputs=OPEN),
                            dict(outputs=0, inputs=0, home=True)])
    reference.node.managed._put_back(dropped=False)

    rig, source = drop_rig(height, neutral=neutral)
    rig.steps = iter([dict(outputs=RELEASE, inputs=OPEN), at_return_retract(rig.node)])
    rig.node.managed._put_back(dropped=True, home_if_exhausted=False)
    assert rig.requests == reference.requests[:-1]
    assert all(name != "DO" for name, _ in rig.requests)
    assert all(not command.mode for _, command in rig.requests)
    assert rig.node.managed.session.attempts[0].state == "DROPPED"
    assert rig.node.managed.session.held_index is None
    assert rig.node.managed.return_progress is None
    assert not rig.node.holding_item and not any(rig.node.expected_outputs.values())
    np.testing.assert_allclose(rig.requests[-2][1].c, source[2].matrix[2, 3] * 1000)


@pytest.mark.parametrize("neutral", [False, True])
def test_shared_return_and_next_pick_are_one_ordered_queue_with_no_home_or_midpoint_wait(neutral):
    rig, source = drop_rig(neutral=neutral, more=True)
    node = rig.node
    prefix, origin = node.managed._put_back(
        dropped=True, continue_candidates=True, home_if_exhausted=False)
    assert not rig.requests  # Prepare once; the next Pick submits the complete group.
    operation = node.managed.return_progress
    bridge = ReturnPickBridge(node, operation, origin)
    plan = node.managed.session.attempts[1].plan
    targets = (*prefix, candidate_transit(prefix[-1].matrix, plan),
               plan[1], plan[2], plan[3])
    rig.steps = iter([
        dict(outputs=RELEASE, inputs=OPEN, currentCommandId=2),
        dict(outputs=0, inputs=0, currentCommandId=3),
        dict(outputs=(1 << 12) | (1 << 13), inputs=1, currentCommandId=7),
    ])
    acquired, _ = rig.transport.move_batch(
        targets, stop_on_suction=True, return_terminal_pose=True,
        confirmed_start_pose=origin, placement_bridge=bridge)
    assert acquired and bridge.completed
    assert [name for name, _ in rig.requests] == [
        "MovL", "MovLIO", "MovLIO", "MovLIO", "MovL", "MovL", "MovLIO", "Stop"]
    assert rig.order[:len(targets)] == [name for name, _ in rig.requests[:-1]]
    assert all(not command.mode for _, command in rig.requests[:-1])
    assert [a.state for a in node.managed.session.attempts] == ["DROPPED", "ACTIVE"]
    assert node.managed.session.held_index is None and node.managed.return_progress is None
    np.testing.assert_array_equal(prefix[1].matrix, source[2].matrix)
    assert [event.vendor_value() for event in prefix[1].motion_io] == [
        "{0,90,2,0}", "{0,90,14,1}", "{0,90,13,0}", "{0,90,1,1}"]
    assert [event.vendor_value() for event in prefix[2].motion_io] == [
        "{1,0,2,0}", "{1,0,14,0}", "{1,0,1,0}", "{1,0,13,0}"]


def test_next_pick_cannot_acquire_old_di1_without_observed_neutral_release_boundary():
    rig, _ = drop_rig(more=True)
    node = rig.node
    prefix, origin = node.managed._put_back(dropped=True, continue_candidates=True)
    operation = node.managed.return_progress
    bridge = ReturnPickBridge(node, operation, origin)
    plan = node.managed.session.attempts[1].plan
    rig.steps = iter([dict(outputs=HELD, inputs=1, currentCommandId=7)])
    with pytest.raises(FeedbackFailure, match="without neutral"):
        rig.transport.move_batch(
            (*prefix, candidate_transit(prefix[-1].matrix, plan), plan[1], plan[2], plan[3]),
            stop_on_suction=True, confirmed_start_pose=origin, placement_bridge=bridge)
    assert not bridge.completed
    assert node.managed.session.held_index == 1
    assert node.managed.session.attempts[1].state == "PENDING"


def test_neutral_before_retract_was_issued_cannot_arm_the_next_pick():
    rig, _ = drop_rig(neutral=True, more=True)
    node = rig.node
    prefix, origin = node.managed._put_back(dropped=True, continue_candidates=True)
    bridge = ReturnPickBridge(node, node.managed.return_progress, origin)
    rig.emit(outputs=0, inputs=0)
    rig.steps = iter([dict(outputs=HELD, inputs=1, currentCommandId=7)])
    plan = node.managed.session.attempts[1].plan
    with pytest.raises(FeedbackFailure, match="without neutral"):
        rig.transport.move_batch(
            (*prefix, candidate_transit(prefix[-1].matrix, plan), plan[1], plan[2], plan[3]),
            stop_on_suction=True, confirmed_start_pose=origin, placement_bridge=bridge)
    assert not bridge.completed
    assert node.managed.session.held_index == 1


@pytest.mark.parametrize("stop_after", range(1, 8))
def test_stop_during_combined_return_pick_admission_retains_old_source(stop_after):
    rig, _ = drop_rig(more=True)
    node = rig.node
    prefix, origin = node.managed._put_back(dropped=True, continue_candidates=True)
    bridge = ReturnPickBridge(node, node.managed.return_progress, origin)
    plan = node.managed.session.attempts[1].plan
    canceled = [False]
    node.cancel_requested = lambda: canceled[0]

    def cancel(index):
        if index == stop_after:
            canceled[0] = True
    rig.on_request = cancel
    with pytest.raises(OperationCanceled):
        rig.transport.move_batch(
            (*prefix, candidate_transit(prefix[-1].matrix, plan), plan[1], plan[2], plan[3]),
            stop_on_suction=True, confirmed_start_pose=origin, placement_bridge=bridge)
    assert len([name for name, _ in rig.requests if name != "Stop"]) == stop_after
    assert rig.requests[-1][0] == "Stop"
    assert node.managed.session.held_index == 1
    assert [a.state for a in node.managed.session.attempts] == ["DROPPED", "PENDING"]
    assert node.managed.return_progress is bridge.operation
