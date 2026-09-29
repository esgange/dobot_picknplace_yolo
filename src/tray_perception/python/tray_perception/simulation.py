"""Historical, teaching-only visualization of the exact simulated response."""

import copy
import math

from geometry_msgs.msg import TransformStamped


FRAME = "tray_teach_simulated_tray"


def response_transform(response, view, now_ns):
    """Reject mismatched replies instead of displaying a nearby live detection."""
    stamp = response.header.stamp.sec * 1_000_000_000 + response.header.stamp.nanosec
    result = view["result"]
    selected = result["selected"]
    if (not response.success or response.header.frame_id != "base_link"
            or not 0 < stamp <= now_ns or stamp != view["rgb"]["stamp_ns"]
            or not response.batch_id or response.found != (selected is not None)
            or response.detected_count != result["count"]
            or response.valid_count != sum(d["valid"] for d in result["detections"])
            or response.status != ("OK" if response.found else "NO_VALID_TRAY")):
        raise ValueError("Simulated tray response does not match its observation")
    if selected is None:
        if response.tray.id:
            raise ValueError("Empty simulated response contains an old tray pose")
        return None
    pose = response.tray
    p, q = pose.pose.position, pose.pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    expected = selected["position"] + selected["quaternion"]
    if (not selected["valid"] or pose.id != f"{response.batch_id}:{selected['source_index']}"
            or not all(math.isfinite(v) for v in values)
            or not all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(values, expected))
            or abs(sum(v*v for v in values[3:]) - 1) > 1e-6):
        raise ValueError("Simulated tray pose differs from the returned selection")
    transform = TransformStamped()
    transform.header = copy.deepcopy(response.header)
    transform.child_frame_id = FRAME
    transform.transform.translation.x, transform.transform.translation.y, \
        transform.transform.translation.z = p.x, p.y, p.z
    transform.transform.rotation = copy.deepcopy(q)
    return transform


class TraySimulationPreview:
    def __init__(self, node):
        self.node = node
        self.binding = self.transform = None

    def clear(self):
        with self.node.lock:
            self.binding = self.transform = None
            self.node.selected = None
            self.node._published_key = None
            self.node.rviz.clear_pose()

    def install(self, response, view):
        with self.node.lock:
            self.clear()
            self.node.requests.validate_view(view)
            transform = response_transform(response, view, self.node.get_clock().now().nanoseconds)
            # Keep identity evidence only; the GUI owns the one frozen image pair.
            self.binding = {key: view[key] for key in (
                "generation", "trigger_binding", "trigger_epoch", "camera_context")}
            self.transform = transform
            if view["cloud"] is not None:
                self.node.rviz.accept(view["cloud"])
            self.node.events.record(
                "INFO", "tray_simulated_pose", "Frozen returned tray; teaching only",
                batch_id=response.batch_id, source_stamp_ns=view["rgb"]["stamp_ns"],
                frame=FRAME if transform is not None else "", found=response.found,
                position=None if transform is None else view["result"]["selected"]["position"],
                quaternion=None if transform is None else view["result"]["selected"]["quaternion"])

    def tick(self):
        with self.node.lock:
            if self.binding is None:
                return False
            try:
                self.node.requests.validate_view(self.binding)
            except (ValueError, OSError, RuntimeError) as exc:
                self.clear()
                self.node.events.record("WARNING", "tray_simulated_pose_cleared", str(exc))
                return False
            if self.transform is not None:
                # Refresh display lifetime only. Geometry stays at its captured base pose.
                self.transform.header.stamp = self.node.get_clock().now().to_msg()
                self.node.broadcaster.sendTransform(self.transform)
                self.node.rviz.show_pose(self.transform)
            return True  # An empty batch also suppresses any earlier live pose.
