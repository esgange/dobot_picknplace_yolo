import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node

from orbbec_camera_launcher.event_log import PackageEventLogger
from orbbec_camera_launcher.project_config import (
    FIXED_CAMERA_COUNT,
    ORBBEC_LAUNCH_FILE,
    camera_specs,
    load_project_config,
    orbbec_launch_arguments,
)


def generate_launch_description():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError(
            '[ORBBEC CAMERA] Source scripts/source_ros_workspace.bash first; '
            'ROS_LOCALHOST_ONLY must be exactly 1'
        )

    project_root, values = load_project_config(require_camera_ready=True)
    cameras = camera_specs(values)
    if len(cameras) != FIXED_CAMERA_COUNT:
        raise RuntimeError(
            f'Headless launch requires exactly {FIXED_CAMERA_COUNT} cameras'
        )
    vendor_launch = (
        Path(get_package_share_directory('orbbec_camera')) / 'launch' / ORBBEC_LAUNCH_FILE
    )
    if not vendor_launch.is_file():
        raise RuntimeError(f'Required vendored Orbbec launch file is missing: {vendor_launch}')

    event_logger = PackageEventLogger(project_root)
    event_logger.record(
        'INFO',
        'headless_launch_requested',
        'strict root .env accepted; starting mandatory camera supervisor',
        camera_count=len(cameras),
        camera_names=[camera.name for camera in cameras],
        serial_numbers=[camera.serial_number for camera in cameras],
        local_only=True,
    )

    supervisor = Node(
        package='orbbec_camera_launcher',
        executable='camera_watchdog',
        namespace='camera_watchdog',
        name='supervisor',
        output='screen',
        parameters=[{
            'camera_names': [camera.name for camera in cameras],
            'serial_numbers': [camera.serial_number for camera in cameras],
            'orbbec_launch_file': ORBBEC_LAUNCH_FILE,
            'device_num': FIXED_CAMERA_COUNT,
            'launch_args_json': json.dumps(
                orbbec_launch_arguments(values),
                separators=(',', ':'),
                sort_keys=True,
            ),
            'workspace_root': str(project_root),
            'scan_timeout_sec': float(values['ORBBEC_SCAN_TIMEOUT_SEC']),
            'startup_timeout_sec': float(values['ORBBEC_STARTUP_TIMEOUT_SEC']),
            'health_timeout_sec': float(values['ORBBEC_HEALTH_TIMEOUT_SEC']),
            'check_period_sec': float(values['ORBBEC_CHECK_PERIOD_SEC']),
            'max_attempts': int(values['ORBBEC_MAX_ATTEMPTS']),
            'retry_delay_sec': float(values['ORBBEC_RETRY_DELAY_SEC']),
            'shutdown_timeout_sec': float(values['ORBBEC_SHUTDOWN_TIMEOUT_SEC']),
        }],
        on_exit=Shutdown(reason='Orbbec camera supervisor stopped'),
    )
    return LaunchDescription([supervisor])
