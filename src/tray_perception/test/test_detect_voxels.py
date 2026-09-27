import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from tf2_ros import TransformException

from tray_perception import detect, rviz
from test_rviz import sample


@pytest.fixture
def loop(monkeypatch):
    now = [10.]
    monkeypatch.setattr(detect.time, "monotonic", lambda: now[0])
    publisher_node = SimpleNamespace(
        create_publisher=MagicMock(side_effect=lambda *_: MagicMock()))
    preview = rviz.TrayRvizPreview(publisher_node, topic_prefix="/tray_detect")
    view = {"connection": 1, "rgb": {"stamp_ns": 100}, "depth": {"stamp_ns": 100}}
    node = SimpleNamespace(
        rviz=preview, requests=SimpleNamespace(busy=False), lock=threading.RLock(),
        work_lock=threading.Lock(), snapshot=MagicMock(return_value=view),
        visuals=MagicMock(return_value={"cloud": sample()}), _check_snapshot=MagicMock())
    return detect.TrayVoxelLoop(node), now, view


def test_headless_cloud_uses_only_depth_visuals_at_one_hz_and_skips_duplicates(loop):
    refresh, now, view = loop
    node = refresh.node
    refresh.tick()
    node.snapshot.assert_called_once_with(depth_required=True)
    node.visuals.assert_called_once_with(view, [], cloud=True)
    node._check_snapshot.assert_called_once_with(view)
    assert node.rviz.publisher.publish.call_count == 1
    now[0] += .99
    refresh.tick()
    assert node.snapshot.call_count == 1
    now[0] += .01
    refresh.tick()
    assert node.snapshot.call_count == 2 and node.visuals.call_count == 1
    assert node.rviz.refreshed_at == 10.
    now[0] += 1
    view["rgb"]["stamp_ns"] = view["depth"]["stamp_ns"] = 101
    node.visuals.return_value = {"cloud": sample(101)}
    refresh.tick()
    assert node.visuals.call_count == 2 and node.rviz.publisher.publish.call_count == 2


@pytest.mark.parametrize("reason", [ValueError("Depth missing"),
                                    TransformException("RGB-time TF unavailable")])
def test_missing_depth_or_tf_retains_then_greys_and_recovers(loop, reason):
    refresh, now, view = loop
    node = refresh.node
    refresh.tick()
    now[0] += 5.01
    node.snapshot.side_effect = reason
    refresh.tick()
    node.rviz.tick()
    assert node.rviz.grey and node.rviz.status()["reason"] == str(reason)
    assert node.rviz.publisher.publish.call_count == 2
    assert node.work_lock.acquire(blocking=False)
    node.work_lock.release()
    node.snapshot.side_effect = None
    view["rgb"]["stamp_ns"] = view["depth"]["stamp_ns"] = 101
    node.visuals.return_value = {"cloud": sample(101)}
    now[0] += 1
    refresh.tick()
    assert not node.rviz.grey and node.rviz.publisher.publish.call_count == 3


def test_pose_requests_take_priority_without_queuing_cloud_work(loop):
    refresh, _, _ = loop
    node = refresh.node
    node.requests.busy = True
    refresh.tick()
    node.snapshot.assert_not_called()
    node.requests.busy = False
    node.work_lock.acquire()
    refresh.tick()
    node.snapshot.assert_not_called()
    node.work_lock.release()
    refresh.tick()
    assert node.visuals.call_count == 1


def test_request_arriving_during_tf_wait_skips_native_cloud_work(loop):
    refresh, _, view = loop
    node = refresh.node

    def snapshot(**_):
        node.requests.busy = True
        return view
    node.snapshot.side_effect = snapshot
    refresh.tick()
    node.visuals.assert_not_called()
    assert node.work_lock.acquire(blocking=False)
    node.work_lock.release()


def test_obsolete_cloud_is_not_published_and_native_failure_is_terminal(loop):
    refresh, now, _ = loop
    node = refresh.node
    node._check_snapshot.side_effect = ValueError("CameraInfo changed")
    refresh.tick()
    node.rviz.publisher.publish.assert_not_called()
    now[0] += 1
    node.visuals.side_effect = RuntimeError("Invalid native tray visualization reply")
    with pytest.raises(RuntimeError, match="Invalid native"):
        refresh.tick()
    assert node.work_lock.acquire(blocking=False)
    node.work_lock.release()
