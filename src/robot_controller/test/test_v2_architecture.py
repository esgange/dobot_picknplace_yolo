from pathlib import Path
import json

import pytest

from robot_controller.state_machine import ControllerStateMachine, STATES, legal_targets
from robot_controller.controller import PackageEventLogger
from robot_controller_interfaces.action import GoHome, PickItem
from robot_controller_interfaces.srv import Preview


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "robot_controller"


def test_every_declared_legal_transition_and_every_other_transition():
    for source in STATES:
        for target in STATES:
            machine = ControllerStateMachine(initial=source)
            if target == source or target in legal_targets(source):
                assert machine.transition(target, "test").state == target
            else:
                with pytest.raises(ValueError, match="Illegal controller transition"):
                    machine.transition(target, "test")


def test_typed_action_result_codes_are_in_result_not_goal():
    assert GoHome.Result.SUCCESS == 0
    assert GoHome.Result.CANCELED == 2
    assert PickItem.Result.NO_PICK == 1
    assert PickItem.Result.STOP_UNCONFIRMED == 5
    assert not hasattr(GoHome.Goal, "SUCCESS")


def test_preview_constants_and_three_process_launch():
    assert (Preview.Request.HOME, Preview.Request.PICK, Preview.Request.CLEAR) == (1, 2, 3)
    launch = (PACKAGE / "launch/robot_controller.launch.py").read_text()
    assert 'executable="robot_controller"' in launch
    assert 'executable="robot_controller_preview"' in launch
    assert 'executable="robot_controller_gui"' in launch
    assert "UnlessCondition(headless)" in launch
    assert "item_teach_file" not in launch
    assert "bin_teach_file" not in launch
    response = Preview.Response()
    response.tf_frames = ["preview_home"]
    assert response.tf_frames == ["preview_home"]
    preview = (PACKAGE / "python/robot_controller/preview.py").read_text()
    assert "tf_child_frames" not in preview


def test_legacy_public_commands_are_absent_and_preview_has_no_dobot_transport():
    controller = (PACKAGE / "python/robot_controller/controller.py").read_text()
    preview = (PACKAGE / "python/robot_controller/preview.py").read_text()
    gui = (PACKAGE / "python/robot_controller/gui.py").read_text()
    combined = controller + preview + gui
    for legacy in ("validate_profile", "set_live", "enable_robot",
                   "set_debug_images", "request_item_poses", "std_srvs"):
        assert legacy not in combined
    assert "DobotTransport" not in preview
    assert "dobot_bringup_ros2/srv" not in preview
    assert "dobot_msgs_v4.srv" not in preview
    assert "DobotTransport" not in gui
    assert "dobot_bringup_ros2/srv" not in gui


def test_runtime_continue_replans_without_vendor_queue_resume_or_empty_movlio():
    runtime = "\n".join(
        path.read_text() for path in (PACKAGE / "python/robot_controller").glob("*.py"))
    assert "InverseKin" not in runtime
    assert "_call_queue_control" not in runtime
    assert "self.managed.continue_operation()" in runtime
    assert 'service = "MovLIO" if events else "MovL"' in runtime
    assert 'fields["mdis"] = events' in runtime


def test_motion_origin_uses_feedinfo_without_getpose_or_second_pose_subscription():
    hardware = (PACKAGE / "python/robot_controller/hardware.py").read_text()
    controller = (PACKAGE / "python/robot_controller/controller.py").read_text()
    constructor = hardware.split("def __init__(self, node, monitor):", 1)[1].split(
        "def close(self):", 1)[0]
    assert 'self.call("GetPose"' not in hardware
    assert "GetPose" not in constructor
    assert 'snapshot.feed["tool_vector_actual"]' in hardware
    assert 'timer != previous_timer' in hardware
    assert '"/dobot_bringup_ros2/msg/FeedInfo"' in controller
    assert '"/dobot_msgs_v4/msg/ToolVectorActual"' not in controller


def test_pause_continue_are_controller_services_and_stop_remains_direct():
    controller = (PACKAGE / "python/robot_controller/controller.py").read_text()
    gui = (PACKAGE / "python/robot_controller/gui.py").read_text()
    assert '"/robot_controller/pause"' in controller
    assert '"/robot_controller/continue"' in controller
    assert '"/robot_controller/stop"' in controller
    assert 'self._command("stop")' in gui
    assert 'self._command("return_item")' in gui


def test_idle_pause_latch_is_not_a_startup_or_ready_gate():
    hardware = (PACKAGE / "python/robot_controller/hardware.py").read_text()
    feedback = (PACKAGE / "python/robot_controller/feedback.py").read_text()
    assert "_correct_persistent_pause_once" not in hardware
    idle = hardware.split("def _idle", 1)[1].split("def _wait_enabled", 1)[0]
    assert "isPauseCmdFlag" not in idle
    blockers = feedback.split("def enabled_blockers", 1)[1]
    assert 'blockers.append(f"isPauseCmdFlag=' not in blockers


def test_headless_configuration_does_not_call_startup():
    controller = (PACKAGE / "python/robot_controller/controller.py").read_text()
    headless_block = controller.split("if self.headless:", 1)[1].split(
        "self.create_timer", 1)[0]
    assert "load_runtime_configuration" in headless_block
    assert "hardware.startup" not in headless_block
    assert '"INACTIVE"' in headless_block


def test_headless_controller_uses_shared_prefix_runtime_catalog():
    profiles = (PACKAGE / "python/robot_controller/profiles.py").read_text()
    assert "from item_perception_yolo.runtime_teach import runtime_teach_catalog" in profiles
    runtime = profiles.split("def runtime_selection", 1)[1]
    assert "runtime_teach_catalog(root)" in runtime
    assert "artifact_type" not in runtime


def test_gui_configuration_can_reload_only_from_unheld_idle_ready():
    controller = (PACKAGE / "python/robot_controller/controller.py").read_text()
    gui = (PACKAGE / "python/robot_controller/gui.py").read_text()
    assert "INACTIVE" in legal_targets("READY")
    assert '("UNCONFIGURED", "INACTIVE", "READY")' in controller
    assert '"Reload Teach Configuration" if configured' in gui
    assert 'current in ("UNCONFIGURED", "INACTIVE", "READY")' in gui


def test_hardware_and_preview_share_candidate_orientation_planner():
    controller = (PACKAGE / "python/robot_controller/controller.py").read_text()
    preview = (PACKAGE / "python/robot_controller/preview.py").read_text()
    motion = (PACKAGE / "python/robot_controller/motion.py").read_text()
    shared = (ROOT / "item_perception_yolo/python/item_perception_yolo/pick_planning.py"
              ).read_text()
    assert "candidate_pose_in_base(" in controller
    assert "candidate_pose_in_base(" in preview
    assert "plan = pick_targets(" in controller
    assert "plan = pick_targets(" in preview
    assert "item[:3, 1]" in shared
    assert "select_pick_attitude(" in controller and "select_pick_attitude(" in preview
    assert "reference_rotation" not in controller + preview + motion + shared
    assert "selected_offset_direction" in controller
    assert "rotation_from_home_deg" in controller


def test_pick_uses_named_forward_and_return_queue_batches():
    motion = (PACKAGE / "python/robot_controller/motion.py").read_text()
    hardware = (PACKAGE / "python/robot_controller/hardware.py").read_text()
    assert '"candidate_1_home_to_pick"' in motion
    assert 'batch_name=f"candidate_{index}_pick_to_home"' in motion
    assert 'batch_name=f"candidate_{index}_pick_to_retry_{next_index}_pick"' in motion
    assert "admit_each_reply_in_order_then_verify_terminal_feedback" in hardware
    assert "calls, progress=progress, outputs_by_call=outputs_by_call" in hardware
    assert 'settings["timing"]["pick_settling"]' in motion
    assert '"motion_batch_queued"' in hardware
    assert '"cp=0"' not in hardware
    assert '"cp="' not in hardware
    assert '"r="' not in hardware


def test_idle_supervision_preempts_unexpected_motion():
    controller = (PACKAGE / "python/robot_controller/controller.py").read_text()
    block = controller.split("def _stop_unexpected_idle_motion", 1)[1].split(
        "def _supervise", 1)[0]
    assert "_request_stop" in block
    assert "_confirm_shared_stop" in block
    assert "_finish_stop_state" in block


def test_package_event_log_is_bounded(tmp_path):
    logger = PackageEventLogger(tmp_path, "fixture")
    for index in range(1001):
        logger.record("INFO", "test", "event", index=index)
    values = [json.loads(line) for line in logger.path.read_text().splitlines()]
    assert len(values) == 1
    assert values[0]["index"] == 1000
    assert values[0]["node"] == "fixture"
