"""Per-run provider selection shared by the controller and GUI."""

from .errors import FeedbackFailure


ANY, TEACH, HEADLESS = 0, 1, 2
SERVICES = {'pick': '/item_detect/get_item_poses', 'place': '/tray_detect/get_tray_pose_v3'}


def validate_mode(mode):
    if type(mode) is not int or mode not in (ANY, TEACH, HEADLESS):
        raise ValueError('Invalid run detector mode')
    return mode


def check_mode(owners, kind, mode):
    validate_mode(mode)
    if mode == ANY:
        return  # The caller still enforces its canonical single-provider contract.
    prefix = 'item' if kind == 'pick' else 'tray'
    expected = prefix + ('_teach' if mode == TEACH else '_detect')
    if owners != [(expected, '/')]:
        raise FeedbackFailure(f'This run requires exactly one /{expected} provider; got {owners}')


def active_mode(node):
    return getattr(getattr(node, 'auto_run', None), 'detector_mode', ANY)
