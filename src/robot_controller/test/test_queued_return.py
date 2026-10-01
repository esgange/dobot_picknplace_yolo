"""Explicit Return Item exercises real queued transport with synthetic feedback only."""

from unittest.mock import Mock

import numpy as np
import pytest

from robot_controller.controller import RobotController
from robot_controller.errors import FeedbackFailure, OperationCanceled
from robot_controller.motion import pick_targets
from robot_controller.pick_session import PickSession
from robot_controller.recovery import HomeRecovery
from robot_controller.state_machine import ControllerStateMachine
from test_placement_queue import QueueRig, HELD, RELEASE, OPEN


def return_rig(height=.8):
    rig = QueueRig()
    node = rig.node
    node.placement = None
    node.configuration.home_joints = (.12, -.2, .3, .04, -.05, .06)
    rig.joint_poses[node.configuration.home_joints] = node.configuration.home_matrix.copy()
    node.active_action = "pick"
    node.machine = ControllerStateMachine(initial="PAUSED")
    node.managed.kind, node.managed.executing = "return", True
    source = np.eye(4)
    source[:3, 3] = [.12, -.23, .25]
    plan = pick_targets(node.configuration.home_matrix, source, node.configuration.profile, 1)
    node.managed.session = PickSession(["saved-item"], [plan])
    node.managed.session.set_state(1, "ACTIVE")
    node.managed.session.set_state(1, "HELD")

    def check_cancel():
        if node.cancel_requested():
            raise OperationCanceled("operator Stop")
    node.raise_if_cancelled = check_cancel
    node._preflight_item_state = lambda held: RobotController._preflight_item_state(node, held)
    current = node.configuration.tray.detect_matrix.copy()
    current[2, 3] = height
    rig.transport.current_pose = lambda: current.copy()
    rig.order.clear()
    return rig, plan


@pytest.mark.parametrize("height", [.3, .8, 1.0])
@pytest.mark.parametrize("approach_speed", [6, 17])
def test_return_queues_release_retract_and_exact_home_before_any_arrival(height, approach_speed):
    rig, source = return_rig(height)
    rig.node.configuration.profile["speed"]["approach_percent"] = approach_speed
    rig.steps = iter([
        dict(outputs=RELEASE, inputs=OPEN),
        dict(outputs=0, inputs=OPEN),
        dict(outputs=0, inputs=OPEN, home=True),
    ])
    rig.node.managed._put_back(dropped=False)
    offset = int(height < .795)
    assert [name for name, _ in rig.requests] == ["MovL"] * offset + [
        "MovL", "MovLIO", "MovLIO", "MovL"]
    assert rig.order[:4 + offset] == [name for name, _ in rig.requests]
    approach, release, retract, home = [request for _, request in rig.requests[offset:]]
    assert (release.a, release.b, release.c) == pytest.approx(source[2].matrix[:3, 3] * 1000)
    assert (approach.a, approach.b, approach.c) == pytest.approx((120., -230., 800.))
    assert (retract.a, retract.b, retract.c) == pytest.approx((120., -230., 800.))
    assert list(release.mdis) == [
        "{0,80,2,0}", "{0,80,14,1}", "{0,80,13,0}", "{0,80,1,1}"]
    assert list(retract.mdis) == [
        "{0,20,2,0}", "{0,20,14,0}", "{0,20,1,0}", "{0,20,13,0}"]
    assert not any(request.mode for _, request in rig.requests[:-1]) and home.mode
    assert (home.a, home.b, home.c, home.d, home.e, home.f) == pytest.approx(
        np.rad2deg(rig.node.configuration.home_joints))
    assert [list(request.param_value) for _, request in rig.requests] == [
        ["user=0", "tool=0", f"v={speed}", f"a={acceleration}"]
        for speed, acceleration in (
            [(100, 70)] * offset + [(100, 70), (approach_speed, 30), (100, 40), (100, 70)])]
    assert rig.node.managed.session.attempts[0].state == "RETURNED"
    assert rig.node.managed.session.held_index is None
    assert rig.node.managed.return_progress is None
    assert not rig.node.holding_item and not any(rig.node.expected_outputs.values())
    assert rig.node.monitor.snapshot().robot_enabled


@pytest.mark.parametrize("stop_after", [1, 2, 3, 4])
def test_direct_stop_during_return_admission_keeps_source_and_prevents_later_motion(stop_after):
    rig, _source = return_rig()
    canceled = [False]
    rig.node.cancel_requested = lambda: canceled[0]

    def cancel(index):
        if index == stop_after:
            canceled[0] = True
    rig.on_request = cancel
    with pytest.raises(OperationCanceled):
        rig.node.managed._put_back(dropped=False)
    assert len([name for name, _ in rig.requests if name != "Stop"]) == stop_after
    assert rig.requests[-1][0] == "Stop"
    assert rig.node.managed.session.held_index == 1
    assert rig.node.managed.session.attempts[0].state == "HELD"
    assert rig.node.managed.return_progress.phase != "DONE"


@pytest.mark.parametrize("outputs,inputs", [(0, 1), (RELEASE, 0), (HELD, 1)])
def test_home_without_neutral_released_feedback_cannot_complete_return(outputs, inputs):
    rig, _source = return_rig()
    rig.steps = iter([dict(outputs=outputs, inputs=inputs, home=True)])
    with pytest.raises(FeedbackFailure, match="final return outputs must be neutral"):
        rig.node.managed._put_back(dropped=False)
    assert rig.node.managed.session.held_index == 1
    assert rig.node.managed.session.attempts[0].state == "HELD"
    assert rig.node.managed.return_progress is not None


def test_return_can_complete_without_observing_intermediate_release_or_di12():
    rig, _source = return_rig()
    rig.steps = iter([dict(outputs=0, inputs=0, home=True)])
    rig.node.managed._put_back(dropped=False)
    assert rig.node.managed.session.attempts[0].state == "RETURNED"


def test_neutral_idle_at_retract_cannot_finish_before_taught_home_joints():
    rig, _source = return_rig()
    rig.steps = iter([
        dict(outputs=0, inputs=0, running=0, currentCommandId=4),
        dict(outputs=0, inputs=0, home=True),
    ])
    advance = rig.monitor.wait_next
    observations = []

    def wait(*args, **kwargs):
        observations.append(rig.node.managed.session.attempts[0].state)
        return advance(*args, **kwargs)
    rig.monitor.wait_next = wait
    rig.node.managed._put_back(dropped=False)
    assert observations == ["HELD", "HELD"]
    assert rig.node.managed.session.attempts[0].state == "RETURNED"


def test_stop_reconciles_timed_return_outputs_and_recover_cancels_without_replay():
    rig, _source = return_rig()
    canceled = [False]
    rig.node.cancel_requested = lambda: canceled[0]

    def release_then_stop(index):
        if index == 2:
            rig.emit(outputs=RELEASE, inputs=OPEN)
        if index == 3:
            canceled[0] = True
    rig.on_request = release_then_stop
    with pytest.raises(OperationCanceled):
        rig.node.managed._put_back(dropped=False)
    progress = rig.node.managed.return_progress

    def confirm(predicate, *_args, **_kwargs):
        first = rig.emit(outputs=0, inputs=0, running=0)
        predicate(first)
        assert predicate(rig.emit(outputs=0, inputs=0, running=0))
    rig.monitor.wait = confirm
    rig.transport.confirm_stop()
    assert progress.phase == "RELEASED"
    assert rig.node.managed.session.attempts[0].state == "HELD"
    before = len(rig.requests)
    recovery = HomeRecovery.cancel_action(rig.node)
    assert recovery.release_confirmed
    assert rig.node.managed.return_progress is None
    assert len(rig.requests) == before


def test_changed_sources_block_return_before_any_motion_or_outputs():
    rig, _source = return_rig()
    rig.node.configuration.validate_sources = Mock(side_effect=FeedbackFailure("source changed"))
    with pytest.raises(FeedbackFailure, match="source changed"):
        rig.node.managed._put_back(dropped=False)
    assert not rig.requests
