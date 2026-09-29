import json
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.action import GoalResponse
from rclpy.task import Future
from rclpy.time import Time
from robot_controller_interfaces.srv import Configure, Command
from robot_controller_interfaces.action import PlaceItem
from tray_perception_interfaces.srv import GetTrayPose

from robot_controller.controller import RobotController
from robot_controller.errors import FeedbackFailure, OperationCanceled, ManagedInterruption
from robot_controller.kinematics import pose_matrix, pose_values
from robot_controller.managed_control import ManagedControl
from robot_controller.pick_session import PickSession
from robot_controller.placement import PlacementOperation, place_targets, validate_target
from robot_controller.state_machine import ControllerStateMachine
from robot_controller.tray_client import validate_result, TrayClient
from robot_controller.ui_state import load_state, save_state
from tray_perception.core import ORIGIN_CONVENTION
from tray_perception.placement import sampling_from_item
from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS


def settings():
    return {"motion": {"standoff_height": 10., "prepick_height": 50., "retract_height": 20.},
            "speed": {"travel_percent": 80, "approach_percent": 10, "retract_percent": 20},
            "acceleration": {"travel_percent": 70, "approach_percent": 30, "retract_percent": 40},
            "timing": {"pick_settling": .2}, "pick_rotation": 90.,
            "geometry": {"pickdepth_radius": 30.}, "quality": dict(QUALITY_DEFAULTS)}


@pytest.mark.parametrize("angle", [-180., -90., 0., 90., 180.])
def test_place_uses_saved_tool_z_rotation_and_exact_depth_heights(angle):
    detect = pose_matrix([300, 200, 800, 175, 12, 28])
    home = pose_matrix([500, -150, 900, 170, 25, -60])
    plan = place_targets(detect, [.3, .2, .25], settings(), angle, home)
    assert [p.name for p in plan] == ["place_pre", "place_release", "place_retract", "place_home"]
    assert [p.matrix[2, 3] for p in plan] == pytest.approx([.31, .26, .31, .9])
    c, s = np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle))
    expected = detect[:3, :3] @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    assert all(np.allclose(p.matrix[:3, :3], expected) for p in plan[:3])
    assert all(np.allclose(p.matrix[:3, 2], detect[:3, 2]) for p in plan[:3])
    assert np.allclose(plan[-1].matrix, home)
    assert [p.speed_percent for p in plan] == [80, 10, 20, 80]
    assert [p.acceleration_percent for p in plan] == [70, 30, 40, 70]
    assert not plan[0].motion_io and not plan[3].motion_io
    assert [(e.percent, e.channel, e.active) for e in plan[1].motion_io] == [
        (80, 2, False), (80, 14, True), (80, 13, False), (80, 1, True)]
    assert [(e.percent, e.channel, e.active) for e in plan[2].motion_io] == [
        (50, 2, False), (50, 14, False), (50, 1, False), (50, 13, False)]


@pytest.mark.parametrize("values", [(0, 10, 0), (10, -1, 0), (10, 10, 181),
                                        (10, 10, -181), (10, float('nan'), 0),
                                        (True, 1, 0), (1, 1, float('inf'))])
def test_placement_rejects_invalid_inputs(values):
    with pytest.raises(ValueError):
        validate_target(*values)


class Hardware:
    def __init__(self, node):
        self.node = node
        self.current = node.configuration.tray.detect_matrix.copy()
        self.outputs = {1: False, 2: True, 13: True, 14: False}
        self.suction = True
        self.calls = []
        self.interrupt_at = None
        self.sequence = 1
        self.history = []

    def sample(self, **_kwargs):
        return SimpleNamespace(suction_present=self.suction, sequence=self.sequence,
                               feed={"digital_input_bits": int(self.suction) | (int(self.outputs[14]) << 11),
                                     "digital_outputs": sum(1 << (ch - 1) for ch, v in self.outputs.items() if v),
                                     "isRunQueuedCmd": 0, "RunningStatus": 0,
                                     "tool_vector_actual": pose_values(self.current)})

    def emit(self):
        self.sequence += 1
        sample = self.sample()
        self.history.append((self.sequence, self.sequence,
                             sample.feed["digital_outputs"], sample.feed["digital_input_bits"]))
        return sample

    def current_pose(self):
        return self.current.copy()

    def home_already_reached(self, _joints):
        return np.allclose(self.current, self.node.configuration.tray.detect_matrix)

    def move_batch(self, targets, *, batch_name, placement=None, forbid_suction=False,
                   confirmed_start_pose=None):
        self.calls.append(("move", tuple(p.name for p in targets), {"batch_name": batch_name}))
        for i, target in enumerate(targets):
            if placement:
                placement.issued(i)
            for event in target.motion_io:
                self.outputs[event.channel] = event.active
                if event.channel == 13 and not event.active:
                    self.suction = False
                if placement:
                    placement.observe(self.node, self.emit())
            self.current = target.matrix.copy()
            if self.interrupt_at == i:
                self.interrupt_at = None
                raise OperationCanceled("Stopped during placement queue")
        if placement:
            placement.complete(self.node, self.emit())

    def output(self, channel, active, **_kwargs):
        self.calls.append(("output", channel, active))
        self.outputs[channel] = active
        self.node.expected_outputs[channel] = active
        self.emit()

    def ensure_no_pending_response(self):
        pass

    def request_stop(self, _reason):
        return object()

    def confirm_stop(self, *_args, **_kwargs):
        pass


def operation_node():
    detect = pose_matrix([300, 200, 800, 180, 0, 0])
    node = SimpleNamespace(configuration=SimpleNamespace(
        profile=settings(), home_matrix=detect.copy(), tray=SimpleNamespace(detect_matrix=detect, detect_joints=(0.,) * 6),
        validate_sources=Mock()), root=None, headless=True, holding_item=True, expected_outputs={},
        events=Mock(), operation_progress=Mock(), wait_for_resume=Mock(),
        _preflight_item_state=Mock(), _execute_tray_position=Mock(),
        raise_if_cancelled=Mock(), cancel_requested=lambda: False,
        pause_requested=lambda: False, active_action="place", startup_complete=True,
        operation_lock=threading.Lock(), pause_event=threading.Event(),
        machine=ControllerStateMachine(initial="PLACING"))
    node._transition = lambda state, msg: node.machine.transition(state, msg)
    node.placement = PlacementOperation(30., 40., 90.)
    node.trays = SimpleNamespace(request=Mock(return_value=np.array([.3, .2, .25])))
    node.hardware = Hardware(node)
    node.monitor = SimpleNamespace(snapshot=node.hardware.sample,
        output_history=lambda seq: tuple(v for v in node.hardware.history if v[0] > seq))
    node.expected_outputs.update(node.hardware.outputs)
    node.managed = ManagedControl(node)
    node.managed.session = PickSession(["bottle"], [()])
    node.managed.session.set_state(1, "ACTIVE")
    node.managed.session.set_state(1, "HELD")
    return node


def test_place_sequence_queues_through_home_and_clears_held_context():
    node = operation_node()
    node.placement.run(node)
    assert node.hardware.calls == [("move", ("place_pre", "place_release", "place_retract", "place_home"),
                                    {"batch_name": "place_to_home"})]
    assert not node.holding_item and node.managed.session.held_index is None
    assert node.managed.session.attempts[0].state == "PLACED"
    assert node.placement.phase == "DONE"
    assert np.allclose(node.hardware.current, node.configuration.home_matrix)
    assert not any(node.hardware.outputs.values())


def test_missing_depth_never_admits_placement_or_releases_item():
    node = operation_node()
    node.trays.request.side_effect = FeedbackFailure("No depth")
    with pytest.raises(FeedbackFailure, match="No depth"):
        node.placement.run(node)
    assert not node.hardware.calls and node.holding_item


def test_source_change_after_detection_blocks_queue_and_preserves_grip():
    node = operation_node()
    node.configuration.validate_sources.side_effect = [None, ValueError("Teach file changed")]
    with pytest.raises(ValueError, match="Teach file changed"):
        node.placement.run(node)
    assert node.holding_item and not node.hardware.calls


@pytest.mark.parametrize("at", [1, 2])
def test_interrupted_release_or_retract_resumes_upward_and_home_without_release(at):
    node = operation_node()
    node.hardware.interrupt_at = at
    with pytest.raises(OperationCanceled):
        node.placement.run(node)
    assert node.placement.release_confirmed and node.placement.needs_recovery
    node.placement.run(node)
    expected = ("place_retract", "place_home") if at == 1 else ("place_home",)
    assert node.hardware.calls[-1][1] == expected
    assert all(call[2] is False for call in node.hardware.calls if call[0] == "output")
    assert node.placement.phase == "DONE"


def test_pause_after_release_stays_in_place_then_recovers_home():
    node = operation_node()
    node.hardware.interrupt_at = 1
    with pytest.raises(OperationCanceled):
        node.placement.run(node)
    node.machine.transition("PAUSING", "Pause")
    node.managed.kind = "pause"
    node.wait_control = lambda _seconds: node.managed.resume.set()
    before = len(node.hardware.calls)
    node.placement.handle_pause(node)
    assert len(node.hardware.calls) == before
    assert node.machine.state == "PLACING" and node.placement.phase == "RELEASED"
    node.placement.run(node)
    assert node.hardware.calls[-1][1][-1] == "place_home"


def test_ui_selection_migration_preserves_files_and_validated_placement(tmp_path):
    path = tmp_path / "last_session.json"
    original = '{"schema_version":1,"item":"item.yaml","bin":"bin.yaml"}'
    path.write_text(original)
    prefill = load_state(path)
    assert prefill["schema_version"] == 3 and prefill["placement"] is None
    assert path.read_text() == original
    save_state(path, "item.yaml", "bin.yaml", "tray.yaml", placement=(12., 34., -180.))
    assert load_state(path)["placement"] == [12., 34., -180.]
    save_state(path, "item.yaml", "bin.yaml", "tray.yaml")
    assert load_state(path)["placement"] == [12., 34., -180.]
    with pytest.raises(ValueError):
        save_state(path, "item.yaml", "bin.yaml", "tray.yaml", placement=(0., 1., 0.))


def response_fixture():
    profile = settings()
    tray = SimpleNamespace(sha256="tray", camera_sha256="camera",
                           profile={"model": {"sha256": "model"},
                                    "reference_plane": {"base_from_plane": np.eye(4).tolist()},
                                    "settings": {"yolo": {"class_ids": [0], "confidence": .5}}})
    config = SimpleNamespace(profile=profile, tray=tray)
    sampling = sampling_from_item(profile, 30., 40.)
    result = GetTrayPose.Response(success=True, found=True, batch_id="batch")
    result.header.frame_id = result.placement.depth_header.frame_id = "base_link"
    result.header.stamp.sec = result.placement.depth_header.stamp.sec = 2
    result.placement.valid = True
    result.placement.accepted_samples = result.placement.total_samples = 100
    result.placement.median_mm = 700.
    result.placement.surface_base.x, result.placement.surface_base.y = .13, .24
    result.placement.surface_base.z = .25
    result.tray.id, result.tray.confidence = "batch:0", .9
    result.tray.pose.position.x, result.tray.pose.position.y = .1, .2
    result.tray.pose.orientation.w = 1.
    result.tray.extent_x = result.tray.width = .2
    result.tray.extent_y = result.tray.length = .3
    result.diagnostics_json = json.dumps({"profile_sha256": "tray", "model_sha256": "model",
        "camera_sha256": "camera", "origin_convention": ORIGIN_CONVENTION,
        "reference_plane": tray.profile["reference_plane"], "placement_sampling": sampling})
    return result, config, sampling


def test_tray_depth_response_binds_pose_settings_sources_and_capture_time():
    result, config, sampling = response_fixture()
    point, _ = validate_result(result, config, sampling, 1_000_000_000, 3_000_000_000)
    assert point == pytest.approx([.13, .24, .25])


@pytest.mark.parametrize("damage", ["cached", "xy", "quaternion", "samples", "hash", "settings", "plane"])
def test_tray_depth_response_rejects_invalid_evidence(damage):
    result, config, sampling = response_fixture()
    if damage == "cached":
        result.header.stamp.sec = 1
    elif damage == "xy":
        result.placement.surface_base.x = .2
    elif damage == "quaternion":
        result.tray.pose.orientation.w = 2.
    elif damage == "samples":
        result.placement.accepted_samples = 2
    elif damage == "plane":
        result.tray.pose.position.z = .1
    else:
        evidence = json.loads(result.diagnostics_json)
        if damage == "hash":
            evidence["profile_sha256"] = "different"
        else:
            evidence["placement_sampling"]["diameter_mm"] = 300.
        result.diagnostics_json = json.dumps(evidence)
    with pytest.raises(FeedbackFailure):
        validate_result(result, config, sampling, 1_000_000_000, 3_000_000_000)


def test_place_goal_requires_startup_matching_configuration_and_held_source():
    node = operation_node()
    node._perception_ready = Mock(return_value=True)
    node.configuration.configuration_id = "bound"
    node._begin_operation = Mock()
    node.machine = ControllerStateMachine(initial="HOLDING")
    assert RobotController._reserve_goal(node, "place", "old") == GoalResponse.REJECT
    node.startup_complete = False
    assert RobotController._reserve_goal(node, "place", "bound") == GoalResponse.REJECT
    node.startup_complete = True
    assert RobotController._reserve_goal(node, "place", "bound") == GoalResponse.ACCEPT
    node.managed.session.set_state(1, "DROPPED")
    assert RobotController._reserve_goal(node, "place", "bound") == GoalResponse.REJECT


def test_generated_configure_includes_optional_tray_file():
    assert Configure.Request().tray_teach_file == ""


def test_place_action_finishes_ready_with_a_typed_result():
    node = operation_node()
    node._end_operation = Mock()
    goal = SimpleNamespace(request=PlaceItem.Goal(x_mm=30., y_mm=40., rotation_deg=-90.),
                           succeed=Mock())
    result = RobotController._execute_place_action(node, goal)
    assert result.outcome == PlaceItem.Result.SUCCESS and result.final_state == "READY"
    assert node.placement is None and not node.holding_item
    goal.succeed.assert_called_once()
    node._end_operation.assert_called_once()


def test_controller_recovery_finishes_placement_without_bin_putback_or_next_pick():
    node = operation_node()
    node.hardware.interrupt_at = 1
    with pytest.raises(OperationCanceled):
        node.placement.run(node)
    node.machine = ControllerStateMachine(initial="RECOVERY_REQUIRED")
    node.global_speed_percent = 50
    node._begin_operation = Mock()
    node._end_operation = Mock()
    node.hardware.recover = Mock()
    node.managed.recover_item_and_continue = Mock(side_effect=AssertionError("Wrong bin recovery"))
    response = RobotController._recover(node, Command.Request(), Command.Response())
    assert response.success and response.state == "READY"
    node.hardware.recover.assert_called_once_with(50, return_item=True)
    assert node.placement is None and node.hardware.calls[-1][1][-1] == "place_home"
    node.managed.recover_item_and_continue.assert_not_called()


@pytest.mark.parametrize('require_held_item', [True, False])
def test_paused_tray_request_retires_before_a_new_request_and_ignores_old_result(require_held_item):
    result, config, sampling = response_fixture()
    config.validate_sources = Mock()
    first, second = Future(), Future()
    second.set_result(result)
    client = SimpleNamespace(service_is_ready=lambda: True,
                             call_async=Mock(side_effect=[first, second]))
    times = iter([1., 1., 3.])
    node = SimpleNamespace(root=None, create_client=lambda *_: client,
        _service_providers=lambda _: [("tray_teach", "/")], events=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=next(times))),
        operation_progress=Mock(), _preflight_item_state=Mock(), monitor=Mock(),
        wait_for_resume=Mock(side_effect=ManagedInterruption("Pause")),
        wait_control=lambda _: first.set_result(GetTrayPose.Response(success=False)))
    observer = TrayClient(node)
    with pytest.raises(ManagedInterruption):
        observer.request(config, sampling["x_mm"], sampling["y_mm"],
                         require_held_item=require_held_item)
    assert not first.done() and client.call_async.call_count == 1
    node.wait_for_resume = Mock()
    point = observer.request(config, sampling["x_mm"], sampling["y_mm"],
                             require_held_item=require_held_item)
    assert point == pytest.approx([.13, .24, .25])
    assert client.call_async.call_count == 2 and observer.pending is None
    if require_held_item:
        node._preflight_item_state.assert_called_with(True)
    else:
        node._preflight_item_state.assert_not_called()
        node.monitor.snapshot.assert_called_with(require_enabled=True)


def test_continue_after_completed_home_never_reobserves_or_releases_again():
    node = operation_node()
    node.placement.run(node)
    calls = list(node.hardware.calls)
    node.placement.run(node)
    assert node.hardware.calls == calls
    node.trays.request.assert_called_once()
    assert node.placement.phase == 'DONE'
