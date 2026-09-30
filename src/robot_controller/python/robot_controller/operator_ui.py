"""Operator labels and button policy, independent of Qt and hardware commands."""

from .errors import EMERGENCY_STOP_MESSAGE


BUSY_ACTIVITIES = {
    "STARTING": "Preparing robot…", "HOMING": "Moving Home…",
    "PICKING": "Picking item…", "TRAY_POSITIONING": "Moving to Tray Detect…",
    "PLACING": "Placing item…", "RETURNING_ITEM": "Returning item…",
    "RECOVERING": "Recovering to Home…",
    "PAUSING": "Stopping and parking — waiting for confirmation…",
    "STOPPING": "Stopping — waiting for confirmation…",
}
ATTENTION = ("FAULT", "RECOVERY_REQUIRED", "HELD_UNKNOWN")


def acquisition_paused(state):
    return bool(state is not None and state.state == "PAUSED" and state.operation == "place"
                and state.phase == "TRAY_ACQUISITION_PAUSED")


def presentation(state, preview=False):
    if state is None:
        return "OFFLINE", "Controller status is missing or older than 1 second."
    if state.state == "FAULT" and EMERGENCY_STOP_MESSAGE in state.message:
        return "EMERGENCY STOP PRESSED", state.message
    if preview:
        return "PREVIEW", "No robot motion. Home, Pick Item and Place Item show planned TFs."
    if not state.feedback_fresh:
        return "OFFLINE", "Robot feedback unavailable. " + state.motion_block_reason
    if state.state in BUSY_ACTIVITIES:
        return "BUSY", BUSY_ACTIVITIES[state.state] + "\n" + state.message
    if state.state in ATTENTION:
        return "ATTENTION REQUIRED", state.message
    if state.state in ("UNCONFIGURED", "INACTIVE"):
        return "NOT READY", (
            "Reload teach configuration to prepare the robot." if state.configured else
            "Select teach files and Load Teach Configuration.")
    if state.state == "PAUSED":
        return "PAUSED", state.message
    if state.state == "HOLDING":
        return "HOLDING ITEM", "Item held. Place it, or Pause to return it to the bin."
    if state.state == "READY":
        return "READY", "Choose an available action."
    return "ATTENTION REQUIRED", "Unknown controller state: " + state.state


def button_policy(state, *, preview=False, pending=False, action_pending=False,
                  managed_pending=False, target_error="", item_selected=False,
                  bin_selected=False, tray_selected=False):
    """Empty reason means enabled; local service availability is applied by the GUI."""
    names = ("configure", "recover", "pause", "return_item", "continue",
             "home", "pick", "place", "preview_toggle", "speed")
    reasons = dict.fromkeys(names, "Unavailable in the current state")
    if state is None:
        reasons = dict.fromkeys(names, "Controller status unavailable")
        if preview:
            reasons["preview_toggle"] = ""
        return reasons
    current = state.state
    idle = not (state.operation_active or action_pending or pending or managed_pending)
    if (current in ("UNCONFIGURED", "INACTIVE", "READY") and idle
            and not state.holding_item and state.configuration_editable):
        reasons["configure"] = (
            "Select an Item Teach file" if not item_selected else
            "Connect robot feedback before loading and preparing"
            if not state.feedback_fresh else "")
    reasons["preview_toggle"] = (
        "" if preview else "Load teach configuration first" if not state.configured else
        "" if idle and state.preview_ready
        and current in ("INACTIVE", "READY", "HOLDING", *ATTENTION)
        else "Finish the operation and wait for fresh stationary robot feedback")
    if preview:
        reasons["configure"] = "Turn Preview OFF before loading and preparing the robot"
        base = ("Wait for the pending request" if not idle else
                "Fresh stationary robot feedback required" if not state.preview_ready else
                "Select an Item Teach file" if not item_selected else "")
        reasons["home"] = base
        reasons["pick"] = (base or ("Select Bin and Tray Teach files"
                                    if not bin_selected or not tray_selected else "")
                           or ("Arm Item Teach or start Item Detect with exactly one provider"
                               if not state.item_detector_ready else ""))
        reasons["place"] = (base or ("Select a Tray Teach file" if not tray_selected else "")
                            or target_error
                            or ("Arm Tray Teach or start Tray Detect with exactly one provider"
                                if not state.tray_detector_ready else ""))
        # Preview validates the selected file's joints, which may differ from the
        # controller's loaded configuration. Never use its at_tray_detect flag here.
        return reasons
    if pending or managed_pending:
        return {key: (value if key == "preview_toggle" else "Wait for the pending request")
                for key, value in reasons.items()}
    if not state.feedback_fresh:
        for name in names:
            if name not in ("configure", "preview_toggle"):
                reasons[name] = "Robot feedback unavailable"
        return reasons
    if current in ATTENTION and idle:
        reasons["recover"] = "" if state.configured else "Load teach configuration first"
    if (state.startup_complete and current in
            ("HOLDING", "HOMING", "PICKING", "TRAY_POSITIONING", "PLACING")
            and (idle or state.operation_active and state.operation in
                 ("home", "pick", "tray_position", "place"))):
        reasons["pause"] = ""
    if current == "PAUSED":
        reasons["continue"] = (state.continue_block_reason or "Continue is unavailable"
                               if not state.can_continue else "")
        if state.can_return_item and state.motion_ready:
            reasons["return_item"] = ""
        if acquisition_paused(state):
            reasons["continue"] = "Use Place Item (Retry) or Return Item"
            reasons["place"] = (state.continue_block_reason or "Retry is unavailable"
                                if not state.can_continue else
                                "Arm Tray Teach or start Tray Detect with exactly one provider"
                                if not state.tray_detector_ready else
                                "" if state.at_tray_detect else
                                "Robot is not at Tray Detect position")
        return reasons
    if current not in ("READY", "HOLDING") or not idle:
        return reasons
    base = ("Load teach configuration to prepare the robot" if not state.startup_complete else
            "Load teach configuration first" if not state.configured else
            state.motion_block_reason or "Robot motion is unavailable"
            if not state.motion_ready else "")
    reasons["home"] = reasons["speed"] = base
    reasons["pick"] = (base or ("Return or place the held item first"
                                if state.holding_item or current == "HOLDING" else "")
                       or ("Load Item and Bin Teach configuration"
                           if not state.pick_configured else "")
                       or ("Load a Tray Teach file with a recorded Tray Detect Pose"
                           if not state.tray_position_recorded else "")
                       or ("Arm Item Teach or start Item Detect with exactly one provider"
                           if not state.item_detector_ready else ""))
    reasons["place"] = (base or ("Load a Tray Teach file with a recorded Tray Detect Pose"
                                 if not state.tray_position_recorded else "")
                        or ("Pick an item successfully first"
                            if not state.manual_placement_enabled
                            and (not state.holding_item or current != "HOLDING") else "")
                        or ("Robot is not at Tray Detect position"
                            if not state.at_tray_detect else "")
                        or target_error
                        or ("Arm Tray Teach or start Tray Detect with exactly one provider"
                            if not state.tray_detector_ready else ""))
    return reasons
