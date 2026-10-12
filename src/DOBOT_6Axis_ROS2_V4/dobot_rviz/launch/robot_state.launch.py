"""Bringup-owned robot TF and joint monitoring, independent of any viewer."""

import os
import re
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch_ros.actions import Node


_ROS_NODE_NAME = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')


def _project_root() -> Path:
    for candidate in Path(__file__).resolve().parents:
        if (candidate / '.env.example').is_file() and (candidate / 'src').is_dir():
            return candidate
    raise RuntimeError(
        '[DOBOT TF] Cannot locate the repository root containing .env.example and src/'
    )


def _read_required_project_values(project_root: Path) -> dict[str, str]:
    env_path = project_root / '.env'
    if not env_path.is_file():
        raise RuntimeError(f'[DOBOT TF] Required project configuration is missing: {env_path}')

    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(env_path.read_text(encoding='utf-8').splitlines(), 1):
        if not raw_line or raw_line.startswith('#'):
            continue
        if raw_line.startswith('export ') or '=' not in raw_line:
            raise RuntimeError(f'[DOBOT TF] Invalid .env syntax at {env_path}:{line_number}')
        key, value = raw_line.split('=', 1)
        if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', key):
            raise RuntimeError(f'[DOBOT TF] Invalid .env key at {env_path}:{line_number}')
        if value != value.strip():
            raise RuntimeError(
                f'[DOBOT TF] .env values cannot have surrounding whitespace at '
                f'{env_path}:{line_number}'
            )
        if key in values:
            raise RuntimeError(
                f'[DOBOT TF] Duplicate .env key {key} at {env_path}:{line_number}'
            )
        values[key] = value

    required_keys = ('ROS_LOCALHOST_ONLY', 'DOBOT_ROBOT_NODE_NAME')
    missing = [key for key in required_keys if key not in values]
    if missing:
        raise RuntimeError('[DOBOT TF] Root .env is missing: ' + ', '.join(missing))
    if values['ROS_LOCALHOST_ONLY'] != '1' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError(
            '[DOBOT TF] ROS_LOCALHOST_ONLY must be exactly 1 in root .env and the '
            'inherited shell environment'
        )
    if not _ROS_NODE_NAME.fullmatch(values['DOBOT_ROBOT_NODE_NAME']):
        raise RuntimeError('[DOBOT TF] DOBOT_ROBOT_NODE_NAME must be a valid ROS node name')
    return values


def _shutdown_when_process_exits(process_name: str):
    return [EmitEvent(event=Shutdown(reason=f'{process_name} exited'))]


def generate_launch_description():
    project_root = _project_root()
    project_values = _read_required_project_values(project_root)
    package_share = get_package_share_path('dobot_rviz')
    model_path = package_share / 'urdf' / 'cr10_robot.urdf'

    if not model_path.is_file():
        raise RuntimeError(f'[DOBOT TF] CR10 URDF is missing: {model_path}')

    robot_description = model_path.read_text(encoding='utf-8')

    joint_state_monitor = Node(
        package='dobot_rviz',
        executable='actual_joint_state_monitor.py',
        name='dobot_actual_joint_state_monitor',
        output='screen',
        parameters=[{
            'expected_publisher_node': project_values['DOBOT_ROBOT_NODE_NAME'],
            'project_root': str(project_root),
        }],
        on_exit=_shutdown_when_process_exits('actual joint-state monitor'),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='dobot_robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        on_exit=_shutdown_when_process_exits('robot_state_publisher'),
    )

    return LaunchDescription([
        joint_state_monitor,
        robot_state_publisher,
    ])
