"""Synthetic transport/streams only: never instantiate a real Dobot transport."""

import json
from datetime import datetime
import math
from pathlib import Path
import shutil
import threading
from types import MethodType, SimpleNamespace as NS
from unittest.mock import MagicMock

import numpy as np
import pytest
from rclpy.task import Future

from test_controller import pair as _pair_fixture
from robot_controller import controller, hardware, profiles, ui_state
from robot_controller.kinematics import Cr10Kinematics, pose_matrix, pose_values
from robot_controller.motion import PickExecutor, home_targets, pick_targets


pair = _pair_fixture  # Reuse the strict synthetic item writer.


def model():
    path = Path(__file__).resolve().parents[2] / "DOBOT_6Axis_ROS2_V4/cra_description"
    return Cr10Kinematics(path / "urdf/cr10_robot.xacro")


def settings():
    return {"motion": {"standoff_height": 90., "zheight_offset": 120.,
                       "prepick_height": 50., "retract_height": 100.},
            "gripper": {"use_grip": True, "grip_onpick": True},
            "timing": {"pick_settling": .2}}


def plan(index=1):
    return pick_targets(pose_matrix([100, 200, 800, 180, 0, 0]), [.3, .4, .1], settings(), index)


class FakeHardware:
    def __init__(self, acquisitions=(True,), *, failed=None, stopped_z=None):
        self.acquisitions = iter(acquisitions)
        self.failed = failed
        self.trace = []
        self.pose = np.eye(4)
        self.stopped_z = stopped_z

    def output(self, channel, active):
        self.trace.append(("DO", channel, active))

    def sensor(self, active, timeout, *, settling_sec):
        self.trace.append(("DI", active, timeout))
        return True if not active else next(self.acquisitions)

    def move(self, target, **kw):
        self.trace.append(("move", target.name, kw, float(target.matrix[2, 3])))
        if self.failed == target.name:
            raise ValueError("Injected hardware failure")
        self.pose = target.matrix.copy()
        if kw.get("stop_on_suction"):
            acquired = next(self.acquisitions)
            if acquired and self.stopped_z is not None:
                self.pose[2, 3] = self.stopped_z
            return acquired
        return False

    def current_pose(self):
        return self.pose.copy()


def execute(fake, cfg=None, plans=None, check=None):
    return PickExecutor(fake, finish_home=False).run(
        plans or [plan()], cfg or settings(), check=check or (lambda _: None),
        return_home=lambda **kw: fake.trace.append(("home", kw)))


def test_pose_units_and_canonical_fk_chain():
    robot = model()
    joints = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    expected = np.eye(4)
    for joint, angle in zip(robot.joints, joints):
        rotation = pose_matrix([0, 0, 0, 0, 0, math.degrees(angle)])
        expected = expected @ joint.origin @ rotation
    np.testing.assert_allclose(robot.forward(joints), expected, atol=1e-12)
    original = pose_matrix([100, 200, 300, 20, -30, 40])
    np.testing.assert_allclose(pose_matrix(pose_values(original)), original, atol=1e-12)
    assert len(robot.sha256) == 64
    for bad in ([0.] * 5, [math.nan] * 6, [100.] * 6):
        with pytest.raises(ValueError):
            robot.forward(bad)


def test_home_height_keeps_current_xy_and_orientation_then_exact_joints():
    current, home = pose_matrix([10, 20, 30, 5, 6, 7]), model().forward([.1] * 6)
    height, final = home_targets(current, home, [.1] * 6)
    assert height.relative_z and final.joints_rad == (.1,) * 6
    assert height.matrix[2, 3] == home[2, 3]
    np.testing.assert_allclose(height.matrix[:2, 3], current[:2, 3])
    np.testing.assert_allclose(height.matrix[:3, :3], current[:3, :3])
    np.testing.assert_allclose(final.matrix, home)


def test_pick_height_equations_and_home_attitude():
    targets = plan()
    np.testing.assert_allclose([t.matrix[2, 3] for t in targets], [.8, .31, .24, .19, .29, .31])
    for target in targets:
        np.testing.assert_allclose(target.matrix[:3, :3], targets[0].matrix[:3, :3])
        np.testing.assert_allclose(target.matrix[:2, 3], [.3, .4])
    cfg = settings()
    cfg["motion"]["zheight_offset"] = 99
    with pytest.raises(ValueError, match="zheight_offset"):
        pick_targets(np.eye(4), [.3, .4, .1], cfg, 1)
    with pytest.raises(ValueError, match="Home Z"):
        pick_targets(np.eye(4), [.3, .4, .1], settings(), 1)


@pytest.mark.parametrize("use_grip,close", [(False, False), (False, True),
                                            (True, False), (True, True)])
def test_finger_rules_and_success_holds_final_retract(use_grip, close):
    cfg, fake = settings(), FakeHardware()
    cfg["gripper"] = {"use_grip": use_grip, "grip_onpick": close}
    outcome = execute(fake, cfg)
    assert outcome["picked"] and outcome["holding_item"]
    finger = [v for v in fake.trace if v[0] == "DO" and v[1] in (2, 14)]
    expected = [("DO", 2, False), ("DO", 14, True)] if use_grip else []
    if use_grip and close:
        expected += [("DO", 14, False), ("DO", 2, True)]
    assert finger == expected
    assert fake.trace[-1][1] == "p1_final" and fake.trace[-1][2]["require_suction"]
    assert not any(v[0] == "home" for v in fake.trace)
    vacuum = [v for v in fake.trace if v[0] == "DO" and v[1] == 13]
    assert vacuum == [("DO", 13, False), ("DO", 13, True)]


def test_early_suction_never_descends_to_a_nominal_retract():
    fake = FakeHardware(stopped_z=.35)
    execute(fake)
    assert [v[3] for v in fake.trace if v[0] == "move"][-2:] == [.35, .35]


def test_missed_suction_settles_and_final_completion_precedes_next_candidate():
    fake = FakeHardware((False, False, True))
    checked = []
    outcome = execute(fake, plans=[plan(), plan(2)], check=checked.append)
    assert checked == [1, 1, 2, 2] and outcome["candidate"] == 2
    assert ("DI", True, .2) in fake.trace
    final = next(i for i, v in enumerate(fake.trace) if v[0:2] == ("move", "p1_final"))
    home = next(i for i, v in enumerate(fake.trace) if v[0] == "home")
    second = next(i for i, v in enumerate(fake.trace) if v[0:2] == ("move", "p2_transit"))
    assert final < home < second


@pytest.mark.parametrize("failed", ["p1_transit", "p1_pick", "p1_retract", "p1_final"])
def test_hardware_fault_is_not_retryable(failed):
    fake = FakeHardware(failed=failed)
    with pytest.raises(ValueError, match="Injected hardware"):
        execute(fake, plans=[plan(), plan(2)])
    assert not any(v[0:2] == ("move", "p2_transit") for v in fake.trace)


def test_exhausted_candidates_and_expiry_do_not_reacquire():
    fake = FakeHardware((False, False))
    outcome = execute(fake)
    assert not outcome["picked"] and not outcome["holding_item"]
    fake = FakeHardware((False, False))

    def check(index):
        if index == 2:
            raise ValueError("expired")
    with pytest.raises(ValueError, match="expired"):
        execute(fake, plans=[plan(), plan(2)], check=check)
    assert any(v[0:2] == ("move", "p1_final") for v in fake.trace)
    assert not any(v[0:2] == ("move", "p2_transit") for v in fake.trace)


def synthetic_transport(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(hardware.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(hardware.time, "sleep", lambda dt: clock.__setitem__(0, clock[0]+dt))
    robot = model()
    pose = robot.forward([.1] * 6)
    feed = {"digital_input_bits": 0, "digital_outputs": 0, "isRunQueuedCmd": 0,
            "RunningStatus": 0, "robot_mode": 5, "tool_vector_actual": pose_values(pose)}
    sequence = [0]

    def snapshot(**_):
        sequence[0] += 1
        return {"feed": feed, "enabled": feed["robot_mode"] == 5, "sequence": sequence[0]}
    node = NS(check_cancelled=MagicMock(), check_command_owner=MagicMock(),
              cancel=threading.Event(),
              feedback_snapshot=snapshot, current_joints=lambda: [.1] * 6, kinematics=robot,
              events=MagicMock(), set_execution_state=MagicMock())
    transport = object.__new__(hardware.DobotHardware)
    transport.node, transport.moving = node, False
    transport.suction_stop, transport.suction_interrupted = None, False
    names = ("Stop", "StopMoveJog", "DisableRobot", "EnableRobot", "SpeedFactor", "Tool",
             "SetTool", "CP", "DO", "GetPose", "InverseKin", "MovLIO", "RelMovLUser")
    transport.types = {name: NS(Request=lambda **fields: NS(**fields)) for name in names}
    transport.clients = {name: MagicMock() for name in names}
    for name, client in transport.clients.items():
        future = Future()
        raw = "0,{" + ",".join(map(str, pose_values(pose))) + "},GetPose(user=0,tool=0);"
        if name == "InverseKin":
            raw = "0,{" + ",".join(map(str, np.rad2deg([.1]*6))) + "},InverseKin();"
        future.set_result(NS(res=0, robot_return=raw))
        client.call_async.return_value = future
    return transport, feed, pose, clock


def test_initialization_order_and_only_first_two_best_effort(monkeypatch):
    transport, feed, _, _ = synthetic_transport(monkeypatch)
    calls = []

    def call(name, **fields):
        calls.append((name, fields))
        if name in ("StopMoveJog", "DisableRobot"):
            raise ValueError("optional unavailable")
    transport.call = call
    transport.initialize()
    assert [n for n, _ in calls] == [
        "StopMoveJog", "DisableRobot", "EnableRobot", "SpeedFactor", "Tool", "SetTool", "CP"]
    assert calls[3][1] == {"ratio": 100} and calls[5][1]["value"] == "{0,0,0,0,0,0}"
    assert transport.node.events.record.call_count == 2
    calls.clear()

    def strict_failure(name, **fields):
        calls.append(name)
        if name == "EnableRobot":
            raise ValueError("strict failed")
    transport.call = strict_failure
    with pytest.raises(ValueError, match="strict failed"):
        transport.initialize()
    assert calls == ["StopMoveJog", "DisableRobot", "EnableRobot"]


def test_home_transport_uses_relative_z_then_joint_movlio_and_confirms_completion(monkeypatch):
    transport, _, pose, clock = synthetic_transport(monkeypatch)
    for target in home_targets(pose, pose, [.1] * 6):
        transport.move(target)
    relative = transport.clients["RelMovLUser"].call_async.call_args.args[0]
    assert relative.a == relative.b == relative.c == 0
    final = transport.clients["MovLIO"].call_async.call_args.args[0]
    assert final.mode and final.mdis == [] and final.param_value == ["user=0", "tool=0"]
    assert final.a == pytest.approx(math.degrees(.1))
    assert clock[0] >= .6 and not transport.moving


def test_di1_before_descent_stops_without_dispatching_descent(monkeypatch):
    transport, feed, pose, _ = synthetic_transport(monkeypatch)
    feed["digital_input_bits"] = 1
    target = home_targets(pose, pose, [.1] * 6)[1]
    assert transport.move(target, stop_on_suction=True)
    transport.clients["Stop"].call_async.assert_called_once()
    transport.clients["MovLIO"].call_async.assert_not_called()
    assert not transport.moving


def test_di1_during_descent_waits_for_stop_and_fresh_stationary_feedback(monkeypatch):
    transport, feed, pose, clock = synthetic_transport(monkeypatch)
    original = transport.clients["MovLIO"].call_async.return_value

    def dispatch(_):
        feed["digital_input_bits"] = 1
        return original
    transport.clients["MovLIO"].call_async.side_effect = dispatch
    assert transport.move(home_targets(pose, pose, [.1] * 6)[1], stop_on_suction=True)
    assert clock[0] >= .3 and not transport.moving
    transport.clients["Stop"].call_async.assert_called_once()


def test_stop_rejection_and_lost_suction_block_retract(monkeypatch):
    transport, feed, pose, _ = synthetic_transport(monkeypatch)
    target = home_targets(pose, pose, [.1] * 6)[1]
    with pytest.raises(ValueError, match="Suction lost"):
        transport.move(target, require_suction=True)
    feed["digital_input_bits"] = 1
    future = Future()
    future.set_result(NS(res=1))
    transport.clients["Stop"].call_async.return_value = future
    with pytest.raises(ValueError, match="Stop rejected"):
        transport.move(target, stop_on_suction=True)
    assert transport.moving


@pytest.mark.parametrize("raw", ["", "0,{1,2},GetPose();", "0,{nan,2,3,4,5,6},GetPose();",
                                 "1,{1,2,3,4,5,6},GetPose();"])
def test_malformed_getpose_is_not_accepted(raw):
    with pytest.raises(ValueError):
        hardware.robot_values(raw, "GetPose")


def test_ui_prefill_is_strict_unapplied_and_atomic(tmp_path):
    path = tmp_path / "last_session.json"
    assert ui_state.load_state(path) is None
    ui_state.save_state(path, "item.yaml", None)
    assert ui_state.load_state(path) == {"schema_version": 1, "item": "item.yaml", "bin": None}
    ui_state.save_state(path, "new.yaml", "bin.yaml")
    assert ui_state.load_state(path)["item"] == "new.yaml"
    path.write_text('{"schema_version":0}')
    with pytest.raises(ValueError):
        ui_state.save_state(path, "item.yaml", "bin.yaml")
    assert path.read_text() == '{"schema_version":0}'


def test_runtime_catalog_uses_strict_copied_pairs_and_no_implicit_choice(pair, monkeypatch):
    root, path, _ = pair
    # Share existing synthetic Bin Teach fixture builders, not operator station artifacts.
    helpers = Path(__file__).resolve().parents[2] / "item_perception_yolo/test"
    monkeypatch.syspath_prepend(str(helpers))
    from test_bin_teach import _applied, _capture
    from item_perception_yolo import bin_teach_core as bin_core
    applied = _applied(root)
    capture = _capture(applied)
    destination = bin_core.bin_output_path("192.168.20.204", root=root,
                                           created_at=datetime.fromisoformat(
                                               capture.captured_at_utc.replace("Z", "+00:00")))
    bin_core.write_bin_teach(
        destination, applied, bin_core.BinArucoSettings("DICT_5X5_50", 60.), capture, root=root)
    monkeypatch.setattr(profiles, "latest_station_calibration", lambda **_: applied)
    runtime = root / "runtime_teach"
    runtime.mkdir()
    for source in (path, path.with_suffix(".pt"), destination):
        shutil.copy2(source, runtime / source.name)
    selected = profiles.runtime_selection(root)
    assert selected.deployment and selected.item_path.parent == runtime
    assert selected.bin.points == bin_core.load_bin_teach(destination, root=root).points
    with pytest.raises(ValueError, match="offline_teach"):
        controller.load_item_profile(selected.item_path, root=root)
    with pytest.raises(ValueError, match="runtime_teach"):
        controller.load_item_profile(path, root=root, deployment=True)
    extra = runtime / "second.yaml"
    shutil.copy2(path, extra)
    with pytest.raises(ValueError, match="exactly one"):
        profiles.runtime_selection(root)
    extra.unlink()
    (runtime / "unknown.pt").write_bytes(b"opaque")
    with pytest.raises(ValueError, match="paired .pt"):
        profiles.runtime_selection(root)


def test_supervisor_uses_signatures_not_weight_hashing_and_clears_changed_preview(tmp_path):
    path = tmp_path / "profile.yaml"
    path.write_text("first")
    node = NS(state_lock=threading.RLock(), cancel=threading.Event(), preview_targets=plan(),
              preview_sources=((path, controller.source_signature(path)),), hardware=None,
              tf_broadcaster=MagicMock(), events=MagicMock(),
              get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: __import__(
                  "builtin_interfaces.msg", fromlist=["Time"]).Time())))
    node.clear_preview = MethodType(controller.RobotController.clear_preview, node)
    controller.RobotController._supervise(node)
    assert len(node.tf_broadcaster.sendTransform.call_args.args[0]) == 6
    path.write_text("changed")
    controller.RobotController._supervise(node)
    assert not node.preview_targets


def test_debug_gui_is_unapplied_and_has_no_hardware_transport(pair, monkeypatch):
    import rclpy
    from PyQt5 import QtWidgets
    from robot_controller.gui import ControllerWindow
    root, path, _ = pair
    monkeypatch.setattr(controller, "workspace_root", lambda: root)
    monkeypatch.setattr(controller, "load_robot_lan1_ip", lambda _: "192.168.20.204")
    monkeypatch.setattr(controller, "_parse_env_file", lambda _: {
        "DOBOT_ROBOT_NODE_NAME": "dobot_bringup_ros2"})
    ui_state.save_state(root / "logs/robot_controller/last_session.json", path, None)
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    rclpy.init(args=[])
    node = controller.RobotController()
    window = None
    try:
        assert node.debug and node.hardware is None and len(list(node.clients)) == 1
        window = ControllerWindow(node)
        assert node.profile_path is None and not window.home.isEnabled()
        assert window.item_path.text() == str(path)
        window.load_selected()
        assert window.home.isEnabled() and not window.pick.isEnabled()
        assert node.profile_path == path and node.action_thread is None
        with pytest.raises(RuntimeError, match="TF-only"):
            node.check_command_owner("EnableRobot")
        window.stop_action()
        assert node.cancel.is_set() and not node.preview_targets
        application.processEvents()
    finally:
        if window is not None:
            window.close()
        node.close_runtime()
        node.destroy_node()
        rclpy.shutdown()


def test_debug_pick_converts_full_platform_transform_and_publishes_all_candidates(pair):
    root, path, profile = pair
    home = pose_matrix([100, 200, 800, 180, 0, 0])
    platform = pose_matrix([200, 100, 50, 5, 10, 20])
    selected = NS(validate=MagicMock(), bin=NS(sha256="b"),
                  station=NS(camera=NS(sha256="c"), platform=NS(sha256="p",
                             base_from_platform=platform)))
    digest = controller.file_profile_digest(path, root)
    batch = {"evidence": {"camera_sha256": "c", "platform_sha256": "p",
                          "bin_sha256": "b", "model_sha256": profile["model"]["sha256"]},
             "observation_stamp_ns": 100_000_000_000, "depth_stamp_ns": 100_000_000_000,
             "targets": [{"position_m": [.1, .2, .05]}, {"position_m": [.2, .1, .05]}]}
    node = NS(profile_path=path, root=root, headless=False, debug=True, hardware=None,
              summary={"profile_sha256": digest}, selection=selected, action_lock=threading.Lock(),
              pose_client=MagicMock(), check_detector_owner=MagicMock(),
              kinematics=NS(forward=lambda _: home), current_joints=lambda: [.1]*6,
              check_cancelled=MagicMock(), clear_preview=MagicMock(), install_preview=MagicMock(),
              set_execution_state=MagicMock(), events=MagicMock(),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_100_000_000)),
              _request_poses=lambda *_: NS(success=True, message=json.dumps(batch)))
    node._home = MethodType(controller.RobotController._home, node)
    node.action_lock.acquire()
    controller.RobotController._run_action(node, "pick")
    assert not node.action_lock.locked()
    targets = node.install_preview.call_args.args[0]
    assert len(targets) == 14
    expected = platform @ [.1, .2, .05, 1.]
    np.testing.assert_allclose(targets[5].matrix[:3, 3],
                               [*expected[:2], expected[2] + .01])
    assert node.set_execution_state.call_args.args[0] == "DEBUG"
    # Hash disagreement must clear rather than install executable/preview targets.
    node.install_preview.reset_mock()
    batch["evidence"]["bin_sha256"] = "wrong"
    node.action_lock.acquire()
    controller.RobotController._run_action(node, "pick")
    node.install_preview.assert_not_called()
    assert "mismatch" in node.set_execution_state.call_args.args[1]
    node._home = MagicMock()
    node.pose_client.service_is_ready.return_value = False
    node.action_lock.acquire()
    controller.RobotController._run_action(node, "pick")
    node._home.assert_not_called()
    assert "independently armed" in node.set_execution_state.call_args.args[1]


def test_feedback_staleness_includes_controller_timer_not_only_republished_feed():
    node = NS(state_lock=threading.RLock(), _sole_publisher=MagicMock(), current_joints=MagicMock(),
              robot_feedback=(True, True, hardware.time.monotonic()), feed_sequence=4,
              controller_progress_at=hardware.time.monotonic()-2,
              feed_feedback=({}, hardware.time.monotonic()))
    with pytest.raises(ValueError, match="stale"):
        controller.RobotController.feedback_snapshot(node, enabled=True)


def test_feedback_fault_and_nonzero_tool_block_execution():
    now = hardware.time.monotonic()
    feed = {"robot_mode": 5, "ErrorStatus": 0, "CollisionStates": 0, "isPauseCmdFlag": 0,
            "userCoordinate": 0, "toolCoordinate": 1}
    node = NS(state_lock=threading.RLock(), _sole_publisher=MagicMock(), current_joints=MagicMock(),
              robot_feedback=(True, True, now), controller_progress_at=now,
              feed_feedback=(feed, now), feed_sequence=3)
    with pytest.raises(ValueError, match="nonzero user/tool"):
        controller.RobotController.feedback_snapshot(node, enabled=True)


def test_service_timeout_late_motion_ack_receives_safety_stop(monkeypatch):
    transport, _, _, _ = synthetic_transport(monkeypatch)
    pending = Future()
    transport.clients["MovLIO"].call_async.return_value = pending
    with pytest.raises(ValueError, match="timeout"):
        transport.call("MovLIO")
    transport.node.cancel.set()
    pending.set_result(NS(res=0))
    transport.clients["Stop"].call_async.assert_called_once()


def test_suction_monitor_interrupts_before_slow_motion_ack(monkeypatch):
    transport, feed, _, _ = synthetic_transport(monkeypatch)
    pending = Future()
    transport.clients["MovLIO"].call_async.return_value = pending
    feed["digital_input_bits"] = 1
    original_snapshot = transport.node.feedback_snapshot

    def snapshot(**kw):
        if transport.clients["Stop"].call_async.call_count and not pending.done():
            pending.set_result(NS(res=0))
        return original_snapshot(**kw)
    transport.node.feedback_snapshot = snapshot
    assert transport.call("MovLIO", monitor_suction=True).res == 0
    assert transport.suction_interrupted
    transport.clients["Stop"].call_async.assert_called_once()


def test_real_supervisor_fault_requests_stop_and_never_claims_completion():
    node = NS(state_lock=threading.RLock(), preview_targets=(), preview_sources=(),
              hardware=NS(moving=True), execution_state="BUSY", cancel=threading.Event(),
              feedback_snapshot=MagicMock(side_effect=ValueError("stale")), holding_item=False,
              clear_preview=MagicMock(), request_stop=MagicMock(), set_execution_state=MagicMock())
    controller.RobotController._supervise(node)
    assert node.cancel.is_set()
    node.request_stop.assert_called_once()
    node.set_execution_state.assert_called_once_with("FAILED", "stale")


def test_headless_debug_node_loads_runtime_profile_without_commands(pair, monkeypatch):
    import rclpy
    root, path, _ = pair
    directory = root / "runtime_teach"
    directory.mkdir()
    for source in (path, path.with_suffix(".pt")):
        shutil.copy2(source, directory / source.name)
    selected = NS(item_path=directory / path.name)
    monkeypatch.setattr(controller, "workspace_root", lambda: root)
    monkeypatch.setattr(controller, "load_robot_lan1_ip", lambda _: "192.168.20.204")
    monkeypatch.setattr(controller, "_parse_env_file", lambda _: {
        "DOBOT_ROBOT_NODE_NAME": "dobot_bringup_ros2"})
    monkeypatch.setattr(controller, "runtime_selection", lambda _: selected)
    rclpy.init(args=["--ros-args", "-p", "headless:=true"])
    node = controller.RobotController()
    try:
        assert node.headless and node.debug and node.hardware is None
        assert node.profile_path.parent == directory and node.action_thread is None
        from rclpy.parameter import Parameter
        result = node.set_parameters([Parameter("item_teach_file", value=str(path))])[0]
        assert not result.successful
    finally:
        node.close_runtime()
        node.destroy_node()
        rclpy.shutdown()


def test_real_launch_constructs_transport_and_initializes_but_never_picks(pair, monkeypatch):
    import rclpy
    root, _, _ = pair
    initialized = threading.Event()
    constructed = []

    class FakeStartup:
        moving = False

        def __init__(self, node):
            self.node = node
            constructed.append(self)

        def initialize(self):
            self.node.set_execution_state("READY", "Synthetic initialization complete")
            initialized.set()

    monkeypatch.setattr(hardware, "DobotHardware", FakeStartup)
    monkeypatch.setattr(controller, "workspace_root", lambda: root)
    monkeypatch.setattr(controller, "load_robot_lan1_ip", lambda _: "192.168.20.204")
    monkeypatch.setattr(controller, "_parse_env_file", lambda _: {
        "DOBOT_ROBOT_NODE_NAME": "dobot_bringup_ros2"})
    rclpy.init(args=["--ros-args", "-p", "debug:=false"])
    node = controller.RobotController()
    try:
        assert initialized.wait(1) and not node.debug and isinstance(node.hardware, FakeStartup)
        assert node.execution_state == "READY" and node.profile_path is None
        assert not node.preview_targets and not node.holding_item
    finally:
        node.close_runtime()
        node.destroy_node()
        rclpy.shutdown()
    state = root / "logs/robot_controller/last_session.json"
    state.write_text('{"schema_version":0}')
    constructed.clear()
    rclpy.init(args=["--ros-args", "-p", "debug:=false"])
    try:
        with pytest.raises(ValueError, match="exact schema 1"):
            controller.RobotController()
        assert not constructed
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize("kind", ["partition", "unknown", "malformed", "symlink"])
def test_runtime_catalog_rejects_bad_entries_without_fallback(tmp_path, kind):
    directory = tmp_path / "runtime_teach"
    directory.mkdir()
    if kind == "partition":
        (directory / "items").mkdir()
    elif kind == "unknown":
        (directory / "unknown.yaml").write_text("artifact_type: tray_teach\n")
    elif kind == "malformed":
        (directory / "bad.yaml").write_text("[not yaml")
    else:
        outside = tmp_path / "outside.yaml"
        outside.write_text("artifact_type: item_teach\n")
        (directory / "item.yaml").symlink_to(outside)
    with pytest.raises(ValueError):
        profiles.runtime_selection(tmp_path)


def test_unknown_duplicate_command_provider_and_legacy_clients_are_blocked():
    nodes = [("robot_controller", "/"), ("dobot_bringup_ros2", "/")]
    services = {("dobot_bringup_ros2", "/"): [("/dobot_bringup_ros2/srv/MovLIO", [])]}
    node = NS(debug=False, publisher_node="/dobot_bringup_ros2",
              get_name=lambda: "robot_controller", get_namespace=lambda: "/",
              get_node_names_and_namespaces=lambda: nodes,
              get_service_names_and_types_by_node=lambda name, ns: services.get((name, ns), []))
    controller.RobotController.check_command_owner(node, "MovLIO")
    nodes.append(("unknown", "/"))
    services[("unknown", "/")] = services[("dobot_bringup_ros2", "/")]
    with pytest.raises(ValueError, match="Duplicate command-service"):
        controller.RobotController.check_command_owner(node, "MovLIO")
    nodes[:] = nodes[:2] + [("motion_debug_gui", "/")]
    with pytest.raises(ValueError, match="Competing legacy"):
        controller.RobotController.check_command_owner(node, "MovLIO")


def test_acknowledgement_with_nonempty_queue_is_not_motion_completion(monkeypatch):
    transport, feed, pose, _ = synthetic_transport(monkeypatch)
    feed["isRunQueuedCmd"] = 1
    target = home_targets(pose, pose, [.1]*6)[1]
    with pytest.raises(ValueError, match="completion timeout"):
        transport.move(target)
    assert transport.moving  # Caller must contain the ambiguous in-progress command.


def test_bad_ik_blocks_motion_before_command_dispatch(monkeypatch):
    transport, _, pose, _ = synthetic_transport(monkeypatch)
    future = Future()
    future.set_result(NS(res=0, robot_return="0,{0,0,0,0,0,0},InverseKin();"))
    transport.clients["InverseKin"].call_async.return_value = future
    with pytest.raises(ValueError, match="InverseKin does not match"):
        transport.move(home_targets(pose, pose, [.1]*6)[0])
    transport.clients["RelMovLUser"].call_async.assert_not_called()
    assert not transport.moving


def test_settle_window_and_do_feedback_confirmation(monkeypatch):
    transport, feed, _, clock = synthetic_transport(monkeypatch)
    assert not transport.sensor(True, .2, settling_sec=0)
    assert clock[0] >= .2
    future = transport.clients["DO"].call_async.return_value

    def output(request):
        feed["digital_outputs"] |= 1 << (request.index-1)
        return future

    transport.clients["DO"].call_async.side_effect = output
    transport.output(13, True)
    assert feed["digital_outputs"] & (1 << 12)


def test_expired_during_transit_blocks_suction_and_final_descent():
    fake = FakeHardware()
    calls = [0]

    def check(_):
        calls[0] += 1
        if calls[0] == 2:
            raise ValueError("Expired during transit")

    with pytest.raises(ValueError, match="Expired during transit"):
        execute(fake, check=check)
    assert ("DO", 13, True) not in fake.trace
    assert not any(v[0:2] == ("move", "p1_pick") for v in fake.trace)


def test_stop_feedback_drift_blocks_retract_even_with_idle_flags(monkeypatch):
    transport, feed, pose, _ = synthetic_transport(monkeypatch)
    feed["digital_input_bits"] = 1
    original = transport.node.feedback_snapshot

    def snapshot(**kw):
        if transport.clients["Stop"].call_async.call_count:
            feed["tool_vector_actual"][2] += .8  # <1 mm per tick, but not stationary overall.
        return original(**kw)

    transport.node.feedback_snapshot = snapshot
    with pytest.raises(ValueError, match="did not confirm stationary"):
        transport.move(home_targets(pose, pose, [.1]*6)[1], stop_on_suction=True)
    assert transport.moving


def test_pose_service_must_have_one_canonical_provider():
    providers = [("item_teach", "/")]
    node = NS(get_node_names_and_namespaces=lambda: providers,
              get_service_names_and_types_by_node=lambda *_: [("/item_detect/get_item_poses", [])])
    controller.RobotController.check_detector_owner(node)
    providers.append(("item_detect", "/"))
    with pytest.raises(ValueError, match="exactly one canonical"):
        controller.RobotController.check_detector_owner(node)


def test_signal_shutdown_keeps_ros_alive_until_stop_and_restores_handlers(monkeypatch):
    handlers = {}
    previous = {controller.signal.SIGINT: "previous-int",
                controller.signal.SIGTERM: "previous-term"}
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setattr(controller.signal, "getsignal", lambda number: previous[number])
    monkeypatch.setattr(controller.signal, "signal", lambda number, callback: handlers.__setitem__(
        number, callback))
    alive, order = [False], []

    def initialize(**kwargs):
        assert kwargs["signal_handler_options"] == controller.SignalHandlerOptions.NO
        alive[0] = True

    def close():
        assert alive[0]
        order.append("stop-before-context-shutdown")

    node = NS(headless=True, fatal_error=None, debug=False, close_runtime=close,
              events=MagicMock(), destroy_node=MagicMock())
    executor = MagicMock()
    executor.spin_once.side_effect = lambda **_: handlers[controller.signal.SIGINT](2, None)
    monkeypatch.setattr(controller.rclpy, "init", initialize)
    monkeypatch.setattr(controller.rclpy, "ok", lambda: alive[0])
    monkeypatch.setattr(controller.rclpy, "shutdown", lambda: alive.__setitem__(0, False))
    monkeypatch.setattr(controller, "RobotController", lambda: node)
    monkeypatch.setattr(controller, "MultiThreadedExecutor", lambda **_: executor)
    controller.main()
    assert order == ["stop-before-context-shutdown"] and not alive[0]
    assert handlers == previous
