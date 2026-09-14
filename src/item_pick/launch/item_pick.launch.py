import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_TEACH_FILES_ROOT = Path.home() / 'CATARM' / 'apps' / 'edge-station-node' / 'teach_files'


def _item_pick_pythonpath() -> str:
    paths = []
    for parent in Path(__file__).resolve().parents:
        build_path = parent / 'build' / 'item_pick'
        source_path = parent / 'src' / 'item_pick'
        if build_path.exists() or source_path.exists():
            if build_path.exists():
                paths.append(str(build_path))
            if source_path.exists():
                paths.append(str(source_path))
            break

    current_pythonpath = os.environ.get('PYTHONPATH', '')
    if current_pythonpath:
        paths.append(current_pythonpath)
    return os.pathsep.join(paths)


def _env_bool_launch_default(name: str, default: bool = False) -> str:
    raw = os.environ.get(name, '1' if default else '0')
    enabled = str(raw or '').strip().lower() in {'1', 'true', 'yes', 'on'}
    return 'true' if enabled else 'false'


def generate_launch_description():
    profiles_dir = LaunchConfiguration('profiles_dir')
    bin_teach_dir = LaunchConfiguration('bin_teach_dir')
    platform_calibration_file = LaunchConfiguration('platform_calibration_file')
    fixed_camera_calibration_file = LaunchConfiguration('fixed_camera_calibration_file')
    camera_bin_valid_pose_attempts = LaunchConfiguration('camera_bin_valid_pose_attempts')
    item_pose_array_topic = LaunchConfiguration('item_pose_array_topic')
    robot_joint_topic = LaunchConfiguration('robot_joint_topic')
    pick_two_stage_descent_enabled = LaunchConfiguration(
        'pick_two_stage_descent_enabled'
    )
    pick_fast_descent_switch_z_up_mm = LaunchConfiguration(
        'pick_fast_descent_switch_z_up_mm'
    )
    pick_fast_descent_speed_percent = LaunchConfiguration(
        'pick_fast_descent_speed_percent'
    )
    return LaunchDescription([
        # Single-file teach layout: profiles_dir holds exactly one
        # items/<id>.yaml at runtime; bin_teach_dir holds exactly one
        # bins/<id>.yaml. The legacy item_pick_runtime_settings.json /
        # *_tool.yaml sidecar / bin_pick_tool_offset_profiles.json
        # launch args are intentionally absent because pick: state now
        # lives inside the items yaml.
        DeclareLaunchArgument(
            'profiles_dir',
            default_value=str(_TEACH_FILES_ROOT / 'items'),
        ),
        DeclareLaunchArgument(
            'bin_teach_dir',
            default_value=str(_TEACH_FILES_ROOT / 'bins'),
        ),
        DeclareLaunchArgument(
            'platform_calibration_file',
            default_value=str(_TEACH_FILES_ROOT / 'platform' / 'platform_calibration_robot_platform_1.yaml'),
        ),
        DeclareLaunchArgument(
            'fixed_camera_calibration_file',
            default_value=str(_TEACH_FILES_ROOT / 'calibration' / 'cr10_orbbec335.yaml'),
        ),
        DeclareLaunchArgument(
            'camera_bin_valid_pose_attempts',
            default_value='15',
        ),
        DeclareLaunchArgument(
            'item_pose_array_topic',
            default_value='bin_item_poses',
        ),
        DeclareLaunchArgument(
            'robot_joint_topic',
            default_value='joint_states_robot',
        ),
        DeclareLaunchArgument(
            'pick_two_stage_descent_enabled',
            default_value=_env_bool_launch_default(
                'EDGE_PICK_TWO_STAGE_DESCENT_ENABLED',
                False,
            ),
        ),
        DeclareLaunchArgument(
            'pick_fast_descent_switch_z_up_mm',
            default_value=os.environ.get(
                'EDGE_PICK_FAST_DESCENT_SWITCH_Z_UP_MM',
                '30',
            ),
        ),
        DeclareLaunchArgument(
            'pick_fast_descent_speed_percent',
            default_value=os.environ.get(
                'EDGE_PICK_FAST_DESCENT_SPEED_PERCENT',
                '50',
            ),
        ),
        Node(
            package='item_pick',
            executable='item_pick',
            name='item_pick',
            output='screen',
            parameters=[{
                'profiles_dir': profiles_dir,
                'bin_teach_dir': bin_teach_dir,
                'platform_calibration_file': platform_calibration_file,
                'fixed_camera_calibration_file': fixed_camera_calibration_file,
                'camera_bin_valid_pose_attempts': camera_bin_valid_pose_attempts,
                'item_pose_array_topic': item_pose_array_topic,
                'robot_joint_topic': robot_joint_topic,
                'pick_two_stage_descent_enabled': ParameterValue(
                    pick_two_stage_descent_enabled,
                    value_type=bool,
                ),
                'pick_fast_descent_switch_z_up_mm': ParameterValue(
                    pick_fast_descent_switch_z_up_mm,
                    value_type=float,
                ),
                'pick_fast_descent_speed_percent': ParameterValue(
                    pick_fast_descent_speed_percent,
                    value_type=float,
                ),
            }],
            additional_env={
                'PYTHONPATH': _item_pick_pythonpath(),
            },
        ),
    ])
