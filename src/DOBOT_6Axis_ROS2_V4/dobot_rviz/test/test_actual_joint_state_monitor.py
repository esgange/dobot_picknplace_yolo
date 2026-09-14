import json
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace

from sensor_msgs.msg import JointState


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / 'dobot_rviz'
    / 'actual_joint_state_monitor.py'
)
SPEC = spec_from_file_location('actual_joint_state_monitor', SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MONITOR = module_from_spec(SPEC)
SPEC.loader.exec_module(MONITOR)


def valid_message() -> JointState:
    message = JointState()
    message.header.stamp.sec = 1
    message.name = list(MONITOR.EXPECTED_JOINT_NAMES)
    message.position = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
    return message


def publisher(node_name: str, namespace: str = '/'):
    return SimpleNamespace(node_name=node_name, node_namespace=namespace)


def test_accepts_exact_bringup_joint_state_contract():
    assert MONITOR.validate_joint_state_message(valid_message()) == ''
    assert MONITOR.validate_publishers(
        [publisher('dobot_bringup_ros2')],
        'dobot_bringup_ros2',
    ) == ''


def test_rejects_wrong_joint_order_and_non_finite_positions():
    message = valid_message()
    message.name[0], message.name[1] = message.name[1], message.name[0]
    assert 'joint names must be exactly' in MONITOR.validate_joint_state_message(message)

    message = valid_message()
    message.position[3] = float('nan')
    assert MONITOR.validate_joint_state_message(message) == 'position contains a non-finite value'


def test_rejects_zero_timestamp():
    message = valid_message()
    message.header.stamp.sec = 0
    assert MONITOR.validate_joint_state_message(message) == 'header timestamp is zero'


def test_rejects_missing_wrong_or_multiple_publishers():
    assert MONITOR.validate_publishers([], 'dobot_bringup_ros2') == 'no publisher exists'
    assert 'publisher must be exactly' in MONITOR.validate_publishers(
        [publisher('untrusted_joint_source')],
        'dobot_bringup_ros2',
    )
    assert 'exactly one publisher is required' in MONITOR.validate_publishers(
        [publisher('dobot_bringup_ros2'), publisher('second_source')],
        'dobot_bringup_ros2',
    )


def test_package_event_logger_overwrites_before_event_1001(tmp_path):
    tmp_path.joinpath('.env.example').write_text('', encoding='utf-8')
    tmp_path.joinpath('src').mkdir()
    event_logger = MONITOR.PackageEventLogger(tmp_path)

    for event_number in range(1001):
        event_logger.record('INFO', 'test_event', f'event={event_number}')

    records = event_logger.path.read_text(encoding='utf-8').splitlines()
    assert len(records) == 1
    assert json.loads(records[0])['message'] == 'event=1000'
