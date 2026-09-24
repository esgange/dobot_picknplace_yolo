"""Typed controller failures used to produce deterministic action results."""


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
