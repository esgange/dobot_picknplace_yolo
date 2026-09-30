"""Independent joint/status position confirmation with no robot or ROS node."""

from pathlib import Path
from types import SimpleNamespace
import threading
import time

import numpy as np
import pytest

from robot_controller.errors import FeedbackFailure, OperationCanceled
from robot_controller.controller import RobotController
from robot_controller.hardware import DobotTransport
from robot_controller.kinematics import Cr10Kinematics
from robot_controller.motion import Target
from test_feedback_v2 import feed, joint_message, primed_monitor
from test_transport_v2 import CompletedFuture, EventLog


MODEL = Path(__file__).resolve().parents[3] / (
    'src/DOBOT_6Axis_ROS2_V4/dobot_rviz/urdf/cr10_robot.urdf')


class PositionRig:
    def __init__(self):
        self.transport = object.__new__(DobotTransport)
        self.monitor = self.transport.monitor = primed_monitor()
        self.model = Cr10Kinematics(MODEL)
        self.transport.node = SimpleNamespace(
            kinematics=self.model, holding_item=False, expected_outputs={},
            raise_if_cancelled=lambda: None, cancel_requested=lambda: False)
        self.steps = iter(())
        self.waits = 0
        self.monitor.wait_next = self.advance

    def publish(self, *, joints=None, idle=True, joint_update=True, status_update=True,
                queue_idle=None):
        if joint_update:
            message = joint_message()
            message.position = list((0.,) * 6 if joints is None else joints)
            self.monitor.update_joints(message)
        if status_update:
            self.monitor.update_status(SimpleNamespace(is_connected=True, is_enable=idle))
        if queue_idle is not None:
            value = self.monitor.snapshot().feed.copy()
            value.update(isRunQueuedCmd=int(not queue_idle), RunningStatus=int(not queue_idle))
            value['controller_timer'] += 1
            self.monitor.update_feed(value)

    def advance(self, _sequence, _timeout, **kwargs):
        assert kwargs['position']
        self.waits += 1
        step = next(self.steps, None)
        if step is None:
            raise FeedbackFailure('No fresh joint/status evidence')
        self.publish(**step)
        return self.monitor.revision


def test_pick_initial_home_skips_queue_immediately_from_idle_and_joint_topics():
    rig = PositionRig()
    # Deliberately contradictory TCP telemetry must not cause a Home command/FK query.
    rig.monitor.update_feed(feed(tool_vector_actual=[999.] * 6))
    joints = (.1, -.2, .3, -.4, .5, -.6)
    rig.publish(joints=joints)
    node = SimpleNamespace(
        hardware=rig.transport, holding_item=False,
        configuration=SimpleNamespace(home_joints=joints),
        raise_if_cancelled=lambda: None, wait_for_resume=lambda: None,
        _preflight_item_state=lambda _holding: None,
        operation_progress=lambda *_args, **_kwargs: None)
    assert RobotController._execute_home(node) == ()
    assert rig.waits == 0
    assert rig.monitor.sequence == 2  # No additional FeedInfo or service round trip.


@pytest.mark.parametrize('joint', range(6))
def test_home_skip_observes_each_joint_immediately_without_new_feed(joint):
    rig = PositionRig()
    joints = [0.] * 6
    joints[joint] = np.deg2rad(1.01)
    rig.publish(joints=joints)
    assert not rig.transport.home_already_reached((0.,) * 6)
    joints[joint] = np.deg2rad(.99)
    rig.publish(joints=joints)
    assert rig.transport.home_already_reached((0.,) * 6)
    rig.publish(joint_update=False, idle=False)
    assert not rig.transport.home_already_reached((0.,) * 6)
    assert rig.waits == 0 and rig.monitor.sequence == 1


def test_stale_feedback_cannot_skip_home():
    rig = PositionRig()
    rig.monitor._monotonic = lambda: time.monotonic() + 2.
    with pytest.raises(FeedbackFailure, match='stale'):
        rig.transport.home_already_reached((0.,) * 6)


def test_origin_uses_joint_fk_and_next_idle_status_without_another_feed_tick():
    rig = PositionRig()
    rig.monitor.update_feed(feed(tool_vector_actual=[999.] * 6))
    joints = (.1, -.2, .3, -.4, .5, -.6)
    rig.steps = iter([dict(joints=joints)])
    assert rig.transport.current_pose() == pytest.approx(rig.model.forward(joints))
    assert rig.waits == 1
    assert rig.monitor.sequence == 2  # No dashboard query or new FeedInfo required.


@pytest.mark.parametrize('missing', ['joint_update', 'status_update'])
def test_origin_requires_both_streams_to_advance(missing):
    rig = PositionRig()
    rig.steps = iter([{missing: False}])
    with pytest.raises(FeedbackFailure, match='fresh joint/status'):
        rig.transport.current_pose()


def test_origin_waits_for_robot_status_idle_even_when_feed_claims_idle():
    rig = PositionRig()
    rig.steps = iter([dict(idle=False), dict(idle=True)])
    assert rig.transport.current_pose() == pytest.approx(rig.model.forward((0.,) * 6))
    assert rig.waits == 2


@pytest.mark.parametrize('field', [
    'userCoordinate', 'toolCoordinate', 'ErrorStatus', 'CollisionStates'])
def test_joint_origin_preserves_feed_fault_and_frame_gates(field):
    rig = PositionRig()
    rig.monitor.update_feed(feed(**{field: 1}))
    with pytest.raises(FeedbackFailure, match=field):
        rig.transport.current_pose()
    assert rig.waits == 0


def test_joint_origin_still_rejects_stale_feedback():
    rig = PositionRig()
    rig.monitor._monotonic = lambda: time.monotonic() + 2.
    with pytest.raises(FeedbackFailure, match='stale'):
        rig.transport.current_pose()


def test_cartesian_arrival_uses_canonical_fk_not_feed_tcp():
    rig = PositionRig()
    target = Target('cartesian', rig.model.forward((0.,) * 6), 100, 100)
    rig.monitor.update_feed(feed(tool_vector_actual=[999.] * 6))
    assert rig.transport._target_reached(target, rig.monitor.snapshot())
    rig.publish(joints=(.2, 0., 0., 0., 0., 0.))
    assert not rig.transport._target_reached(target, rig.monitor.snapshot())


def test_invalid_joint_fk_never_supplies_a_pose():
    rig = PositionRig()
    rig.transport.node.kinematics = SimpleNamespace(forward=lambda _j: np.full((4, 4), np.nan))
    with pytest.raises(FeedbackFailure, match='joint-derived'):
        rig.transport.pose_from_snapshot(rig.monitor.snapshot())


def test_home_reset_waits_for_io_queue_to_finish_without_claiming_position_loss():
    rig = PositionRig()
    rig.monitor.update_feed(feed(isRunQueuedCmd=1, RunningStatus=1))
    rig.steps = iter([dict(idle=False), dict(idle=True), dict(queue_idle=True)])
    sample = rig.transport.confirm_home((0.,) * 6)
    assert sample.robot_enabled and sample.joints == (0.,) * 6
    assert rig.waits == 3


def test_home_reset_reports_actual_joint_error_instead_of_claiming_home_was_lost():
    rig = PositionRig()
    rig.steps = iter([dict(joints=(.1,) * 6)])
    with pytest.raises(FeedbackFailure, match='maximum joint error=5.7296'):
        rig.transport.confirm_home((0.,) * 6)


def test_position_wait_is_direct_stop_aware():
    rig = PositionRig()

    def cancel():
        raise OperationCanceled('Stop')
    rig.transport.node.raise_if_cancelled = cancel
    with pytest.raises(OperationCanceled):
        rig.transport.current_pose()


@pytest.mark.parametrize('topic', ['joints', 'status'])
def test_position_wait_wakes_on_either_stream_without_feed_update(topic):
    monitor = primed_monitor()
    revision = monitor.revision

    def publish():
        time.sleep(.02)
        if topic == 'joints':
            monitor.update_joints(joint_message())
        else:
            monitor.update_status(SimpleNamespace(is_connected=True, is_enable=True))
    thread = threading.Thread(target=publish)
    thread.start()
    try:
        assert monitor.wait_next(revision, .3, position=True) > revision
        assert monitor.sequence == 1
    finally:
        thread.join()


def test_repeated_or_backward_joint_stamps_cannot_refresh_position():
    monitor = primed_monitor()
    before = monitor.snapshot()
    for stamp in (before.joint_stamp_ns, before.joint_stamp_ns - 1):
        message = joint_message(stamp)
        message.position[0] = 1.
        monitor.update_joints(message)
        assert monitor.snapshot().joints == before.joints
        assert monitor.revision == before.revision


@pytest.mark.parametrize('mode', [4, 5, 9, 10])
def test_stop_uses_distinct_stationary_joint_samples_even_in_disabled_fault_modes(mode):
    rig = PositionRig()
    rig.transport.node.events = EventLog()
    rig.monitor.update_feed(feed(robot_mode=mode, EnableStatus=int(mode == 5)))
    rig.monitor.update_status(SimpleNamespace(is_connected=True, is_enable=mode == 5))

    def wait(predicate, *_args, **_kwargs):
        first = rig.monitor.snapshot()
        assert not predicate(first)
        assert not predicate(first)  # Re-reading one joint stamp is not physical confirmation.
        message = joint_message()
        message.position[0] = .01
        rig.monitor.update_joints(message)
        assert not predicate(rig.monitor.snapshot())  # Joint motion with constant FeedInfo TCP.
        message = joint_message()
        message.position[0] = .01
        rig.monitor.update_joints(message)
        assert predicate(rig.monitor.snapshot())
        return rig.monitor.snapshot()

    rig.monitor.wait = wait
    rig.transport.confirm_stop(CompletedFuture())
    assert rig.monitor.sequence == 2  # Stop stationarity did not need another FeedInfo update.
