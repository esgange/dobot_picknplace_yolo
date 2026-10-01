"""Real transport verifies the drop/return boundary without commanding hardware."""

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
def test_drop_requires_target_and_idle_then_returns_on_first_valid_sample(returning, blocker):
    rig, run = operation_rig(returning)
    rig.drop_steps = iter([
        dict(outputs=HELD, inputs=1),
        dict(outputs=RELEASE, inputs=OPEN, drop=True, **blocker),
        dict(outputs=RELEASE, inputs=OPEN, drop=True),
    ])
    received = []
    advance = rig.monitor.wait_next

    def next_sample(*args, **kwargs):
        received.append(len(rig.requests))
        return advance(*args, **kwargs)
    rig.monitor.wait_next = next_sample
    run()
    assert received[:3] == [2, 2, 2]  # Neither retract nor Home admitted early.
    assert rig.order[:6] == ["MovL", "MovLIO", "feedback", "feedback", "feedback", "MovLIO"]
    completed = [call for call in rig.node.events.record.call_args_list
                 if call.args[1] == "motion_batch_completed"]
    assert len(completed) == 2
    assert all(call.kwargs["terminal_stable_sec"] == 0. for call in completed)
    assert not any(name in ("DO", "CP", "SpeedFactor") for name, _ in rig.requests)
    assert rig.node.managed.session.held_index is None


@pytest.mark.parametrize("returning", [False, True])
def test_stop_at_confirmed_drop_prevents_the_entire_return_queue(returning):
    rig, run = operation_rig(returning)
    canceled = [False]
    rig.node.cancel_requested = lambda: canceled[0]

    def check_cancel():
        if canceled[0]:
            raise OperationCanceled("Stop at drop boundary")
    rig.node.raise_if_cancelled = check_cancel

    def stop(event_level, event, *_args, **_kwargs):
        if event == "release_drop_arrived":
            canceled[0] = True
    rig.node.events.record.side_effect = stop
    with pytest.raises(OperationCanceled):
        run()
    assert [name for name, _ in rig.requests if name != "Stop"] == ["MovL", "MovLIO"]
    assert rig.node.managed.session.held_index == 1
    assert rig.node.managed.session.attempts[0].state == "HELD"
