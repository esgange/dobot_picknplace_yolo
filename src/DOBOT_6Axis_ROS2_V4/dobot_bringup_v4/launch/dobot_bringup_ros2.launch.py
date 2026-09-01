from launch import LaunchDescription
import launch_ros.actions
import json
import ipaddress
import os
from pathlib import Path
import re


_ENV_KEY = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_SUPPORTED_ENV_KEYS = {"DOBOT_ROBOT_IP"}


def _load_project_env():
    """Load and validate the required project .env.

    The repository root is identified by its tracked .env.example marker. This
    works both from a source checkout and an installed colcon workspace. No
    third-party dotenv package is required offline.
    """
    launch_dir = Path(__file__).resolve().parent
    project_root = None
    for parent in (launch_dir, *launch_dir.parents):
        if (parent / ".env.example").is_file():
            project_root = parent
            break

    if project_root is None:
        raise RuntimeError(
            "[DOBOT BRINGUP] Repository root marker .env.example was not found "
            "above the launch file"
        )

    env_path = project_root / ".env"
    if not env_path.is_file():
        raise RuntimeError(
            f"[DOBOT BRINGUP] Required project configuration is missing: {env_path}"
        )

    values = {}
    with env_path.open("r", encoding="utf-8") as env_file:
        for line_number, line in enumerate(env_file, start=1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("export "):
                raise RuntimeError(
                    f"[DOBOT BRINGUP] .env must use KEY=value syntax only: "
                    f"{env_path}:{line_number}"
                )
            if "=" not in line:
                raise RuntimeError(
                    f"[DOBOT BRINGUP] Malformed .env line {env_path}:{line_number}"
                )
            key, value = line.split("=", 1)
            if key != key.strip() or value != value.strip():
                raise RuntimeError(
                    f"[DOBOT BRINGUP] .env must not contain spaces around '=': "
                    f"{env_path}:{line_number}"
                )
            if not _ENV_KEY.fullmatch(key):
                raise RuntimeError(
                    f"[DOBOT BRINGUP] Invalid .env key {env_path}:{line_number}"
                )
            if key not in _SUPPORTED_ENV_KEYS:
                raise RuntimeError(
                    f"[DOBOT BRINGUP] Unsupported .env key '{key}' "
                    f"{env_path}:{line_number}"
                )
            if key in values:
                raise RuntimeError(
                    f"[DOBOT BRINGUP] Duplicate .env key '{key}' "
                    f"{env_path}:{line_number}"
                )
            values[key] = value

    ip_address = values.get("DOBOT_ROBOT_IP", "").strip()
    if not ip_address:
        raise RuntimeError(
            f"[DOBOT BRINGUP] .env must define a non-empty DOBOT_ROBOT_IP: {env_path}"
        )
    try:
        ipaddress.ip_address(ip_address)
    except ValueError as exc:
        raise RuntimeError(
            f"[DOBOT BRINGUP] DOBOT_ROBOT_IP is not a valid IP address: {ip_address}"
        ) from exc

    # The project configuration is authoritative. This intentionally replaces
    # inherited values so a shell variable cannot silently select another robot.
    os.environ.update(values)
    return env_path, values


env_path, project_config = _load_project_env()
cur_config_path = Path(__file__).resolve().parent.parent / "config"
cur_json_path = cur_config_path / "param.json"

with cur_json_path.open("r", encoding="utf-8") as file:
    json_data = json.load(file)

robot_number = json_data["robot_number"]
current_robot = json_data["current_robot"]
node_info = json_data["node_info"]
current_robot_info = node_info[current_robot - 1]

trajectory_duration = current_robot_info["trajectory_duration"]
robot_node_name = current_robot_info["robot_node_name"]

ip_address = project_config["DOBOT_ROBOT_IP"].strip()
# This vendored profile contains only the Dobot CR10.
robot_type = "cr10"

print(f"[DOBOT BRINGUP] Using IP: {ip_address}")
print(f"[DOBOT BRINGUP] Using Robot Type: {robot_type}")
print(f"[DOBOT BRINGUP] Loaded project config: {env_path}")


dobot_ros2_params = [
    {"robot_ip_address": ip_address},
    {"robot_type": robot_type},
    {"trajectory_duration": trajectory_duration},
    {"robot_node_name": robot_node_name},
    {"robot_number": robot_number},
]


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package="dobot_bringup_v4",
            executable="dobot_bringup_v4_node",
            name=robot_node_name,
            output="screen",
            parameters=dobot_ros2_params
            # respawn=True
        ),
    ])
