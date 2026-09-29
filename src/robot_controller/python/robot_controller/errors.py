"""Typed controller failures used to produce deterministic action results."""

import json


EMERGENCY_STOP_MESSAGE = "Emergency stop pressed — cannot start or recover."
EMERGENCY_STOP_GUIDANCE = (
    EMERGENCY_STOP_MESSAGE + " Release the emergency-stop button, then click Recover / Clear Error.")


def command_failure_message(name, result):
    code = None if result is None else result.res
    if code == -3:
        return f"{EMERGENCY_STOP_GUIDANCE} ({name} returned -3.)"
    return f"{name} failed: {code}"


def alarm_ids(robot_return):
    """Parse the V4.6.5 GetErrorID payload: {[controller/alarm IDs]}."""
    try:
        if (not isinstance(robot_return, str) or not robot_return.startswith("{")
                or not robot_return.endswith("}")):
            raise ValueError("missing payload braces")
        ids = json.loads(robot_return[1:-1])
        if type(ids) is not list or any(type(code) is not int or code < 0 for code in ids):
            raise ValueError("expected a flat list of nonnegative integer alarm IDs")
        return tuple(ids)
    except (TypeError, ValueError) as exc:
        raise FeedbackFailure(f"Cannot read robot alarms: malformed GetErrorID payload: {exc}") from exc


UNKNOWN_ITEM_GUIDANCE = (
    "DI1 suction is HIGH without trusted controller-owned pickup context; outputs preserved. "
    "Keep the robot stopped. Safely secure and clear any item from the gripper, "
    "or check the suction sensor for an obstruction. Once DI1 shows LOW, "
    "close Gripper Diagnostics and click Recover again.")


class ControllerError(RuntimeError):
    """Base class for an operation that cannot complete safely."""


class OperationCanceled(ControllerError):
    """The active operation was explicitly cancelled and Stop was requested."""


class CommandRejected(ControllerError):
    """A command was rejected before completion could be established."""


class EmergencyStopPressed(CommandRejected):
    """Confirmed Dobot emergency-stop response or alarm, never a guessed fault."""


def command_rejection(name, result):
    error = EmergencyStopPressed if result is not None and result.res == -3 else CommandRejected
    return error(command_failure_message(name, result))


class CommandResponseTimeout(ControllerError):
    """A command acknowledgement is ambiguous because no reply arrived."""


class FeedbackFailure(ControllerError):
    """Required robot feedback was missing, stale, malformed, or unsafe."""


class HeldSuctionLost(FeedbackFailure):
    """Confirmed held-item DI1 loss; an active Pick may put back its saved item."""


class StopUnconfirmed(ControllerError):
    """The independent Stop path did not prove a stationary empty queue."""


class HeldUnknown(ControllerError):
    """DI1 is active without trusted controller-owned pickup context."""


class NoPick(ControllerError):
    """All valid candidates were attempted without confirmed suction."""


class ManagedInterruption(ControllerError):
    """Transfer the operation executor to the requested Pause/return routine."""


class PausedItemDropped(ControllerError):
    """Fresh DI1 lost during managed parking; retain the item's return context."""


class ReturnedToHome(ControllerError):
    """An explicit controlled return finished and ended the original action."""
