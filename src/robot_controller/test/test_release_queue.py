"""Real transport verifies uninterrupted release queues without commanding hardware."""

import pytest

from robot_controller.errors import OperationCanceled
from test_placement_queue import QueueRig, HELD, RELEASE, OPEN
from test_queued_return import return_rig


def operation_rig(returning):
    if returning:
        rig, _source = return_rig()
        rig.steps = iter([dict(outputs=0, inputs=0, home=True)])

        def run():
            rig.node.managed._put_back(dropped=False)
    else:
        rig = QueueRig()
        rig.steps = iter([
            dict(outputs=0, inputs=0), dict(outputs=0, inputs=0, retract=True)])

        def run():
            rig.node.placement.run(rig.node)
            rig.node.placement.finish_pending(rig.node)
    rig.order.clear()
    return rig, run


@pytest.mark.parametrize("returning", [False, True])
@pytest.mark.parametrize("blocker", [
    {"tool_vector_actual": [0.] * 6},
    {"RunningStatus": 1, "isRunQueuedCmd": 1},
])
def test_whole_route_is_queued_but_completion_still_requires_final_pose_and_idle(
        returning, blocker):
    rig, run = operation_rig(returning)
    endpoint = {"home": True} if returning else {"retract": True}
    rig.steps = iter([
        dict(outputs=HELD, inputs=1),
        dict(outputs=RELEASE, inputs=OPEN, drop=True),
        dict(outputs=0, inputs=0, **endpoint, **blocker),
        dict(outputs=0, inputs=0, **endpoint),
    ])
    received = []
    advance = rig.monitor.wait_next

    def next_sample(*args, **kwargs):
        assert rig.node.managed.session.attempts[0].state == "HELD"
        received.append(len(rig.requests))
        return advance(*args, **kwargs)
    rig.monitor.wait_next = next_sample
    run()
    commands = ["MovL", "MovLIO", "MovLIO"] + (["MovJ"] if returning else [])
    assert received == [len(commands)] * 4
    assert rig.order[:len(commands)] == commands
    completed = [call for call in rig.node.events.record.call_args_list
                 if call.args[1] == "motion_batch_completed"]
    assert len(completed) == 1
    assert all(call.kwargs["terminal_stable_sec"] == 0. for call in completed)
    assert not any(name in ("DO", "CP", "SpeedFactor") for name, _ in rig.requests)
    assert rig.node.managed.session.held_index is None


@pytest.mark.parametrize("returning", [False, True])
def test_stop_after_drop_admission_prevents_retract_and_home_admission(returning):
    rig, run = operation_rig(returning)
    canceled = [False]
    rig.node.cancel_requested = lambda: canceled[0]

    def check_cancel():
        if canceled[0]:
            raise OperationCanceled("Stop after drop admission")
    rig.node.raise_if_cancelled = check_cancel

    def stop(index):
        if index == 2:
            canceled[0] = True
    rig.on_request = stop
    with pytest.raises(OperationCanceled):
        run()
    assert [name for name, _ in rig.requests if name != "Stop"] == ["MovL", "MovLIO"]
    assert rig.requests[-1][0] == "Stop"
    assert rig.node.managed.session.held_index == 1
    assert rig.node.managed.session.attempts[0].state == "HELD"
