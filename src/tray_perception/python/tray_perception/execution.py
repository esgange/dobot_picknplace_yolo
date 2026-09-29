"""Report ROS executor failures to the tray UI/headless loop and package log."""

import traceback


def spin_checked(node, executor):
    error = None
    trace = ""
    try:
        executor.spin()
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        trace = traceback.format_exc()
    if node.shutdown_requested.is_set() or not node.context.ok():
        return
    reason = "Tray ROS reception stopped: " + (error or "executor exited unexpectedly")
    node.fatal_error = reason + ". Restart Tray Teach/Detect after rebuilding its clients."
    # Disarm logically without trying to destroy ROS handles in a failed executor.
    node.requests.disarm(node.fatal_error)
    node.events.record("FATAL", "tray_executor_failed", node.fatal_error, traceback=trace)
