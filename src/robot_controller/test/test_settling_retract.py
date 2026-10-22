"""Independent settling height, additional transport lift and continuous DI1 gate."""

import numpy as np
import pytest

from robot_controller.motion import PickExecutor, pick_targets, gripper_close_events
from robot_controller.pick_session import return_targets
from test_motion_v2 import FakeHardware, item_pose, matrix, settings, tray_target
from test_deferred_drop import RetractRig


@pytest.mark.parametrize("prepick, settling, extra", [
    (40., 20., 30.), (10., 80., 20.), (100., 10., 20.), (40., 0., 30.), (40., 20., 0.),
])
def test_approach_is_independent_and_second_lift_adds_only_to_settling(prepick, settling, extra):
    taught = settings(close_on_pick=False)
    taught["motion"].update(prepick_height=prepick, settling_retract=settling,
                            pick_retract=extra, standoff_height=10.)
    plan = pick_targets(matrix(1.), item_pose(z=.3), taught, 1)
    assert [plan[i].matrix[2, 3] for i in (2, 3, 4, 5)] == pytest.approx([
        .31 + prepick / 1000, .31, .31 + settling / 1000,
        .31 + (settling + extra) / 1000])
    assert plan[1].matrix[2, 3] >= plan[2].matrix[2, 3]
    release, _ = return_targets(plan)
    assert np.array_equal(release.matrix, plan[2].matrix)
    with pytest.raises(ValueError, match="Home Z"):
        pick_targets(matrix(.31 + max(prepick, settling + extra) / 1000 - .001),
                     item_pose(z=.3), taught, 1)


@pytest.mark.parametrize("stopped_z", [.3, .325, .36])
def test_delayed_close_is_on_second_lift_and_saved_release_is_independent(stopped_z):
    taught = settings(close_on_pick=False)
    taught["motion"].update(prepick_height=10., settling_retract=20., pick_retract=30.)
    plan = pick_targets(matrix(1.), item_pose(), taught, 1)
    hardware = FakeHardware([True])
    hardware.pose[2, 3] = stopped_z
    saved = []
    PickExecutor(hardware, finish_home=True).run(
        [plan], taught, tray_target=tray_target(), check=lambda _i: None,
        return_home=lambda **_k: pytest.fail("Unexpected Home"),
        remember_prepick=lambda target, _grip: saved.append(target))
    first, second, exit_target = hardware.targets[-1][:3]
    assert [t.matrix[2, 3] for t in (first, second, exit_target)] == pytest.approx([
        max(stopped_z, .32), max(stopped_z, .35), 1.])
    assert not first.motion_io
    assert second.motion_io == gripper_close_events(50)
    assert saved[-1].matrix[2, 3] == pytest.approx(max(.31, stopped_z))
    assert len(hardware.targets) == 2  # No additional arrival/DI wait between lifts.
    request = next(kwargs for kind, _target, kwargs in hardware.log if kind == "move")
    assert request["pickup_retract_pose"][2, 3] == pytest.approx(.32)


def test_actual_settling_height_arms_loss_before_second_lift_finishes():
    taught = settings()
    taught["motion"].update(prepick_height=80., settling_retract=20., pick_retract=30.)
    plan = pick_targets(matrix(.5), item_pose(z=.1), taught, 1)
    rig = RetractRig()
    sample = rig.emit(0., .1, True)
    rig.transport.defer_pickup_drop(plan[4].matrix, sample)
    assert rig.emit(.1, .119, False).suction_present
    assert rig.emit(.8, .119, False).suction_present
    assert not rig.stops  # Low before settling height does not consume the debounce.
    assert rig.emit(.9, .121, False).suction_present
    assert rig.monitor._pickup_retract is None
    assert rig.emit(1.399, .14, False).suction_present
    assert not rig.emit(1.401, .149, False).suction_present
    assert rig.stops  # Monitor is active before pick_retract (.15), and pre-pick (.18).
