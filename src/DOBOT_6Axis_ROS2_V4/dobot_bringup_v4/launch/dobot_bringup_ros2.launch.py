from launch import LaunchDescription
import launch_ros.actions
from datetime import datetime, timezone
import json
import ipaddress
import math
import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET


_ENV_KEY = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_ROS_NODE_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_PACKAGE_NAME = re.compile(r"^[A-Za-z0-9_.-]+$")
_SUPPORTED_ENV_KEYS = {
    "DOBOT_ROBOT_LAN1_IP",
    "DOBOT_ROBOT_LAN2_IP",
    "DOBOT_ROBOT_TYPE",
    "DOBOT_ROBOT_NUMBER",
    "DOBOT_TRAJECTORY_DURATION",
    "DOBOT_ROBOT_NODE_NAME",
}
_REQUIRED_ENV_KEYS = frozenset(_SUPPORTED_ENV_KEYS)
_MAX_LOG_EVENTS = 1000


def _load_project_env():
    """
    Load and validate the required project .env.

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

    missing = sorted(_REQUIRED_ENV_KEYS - values.keys())
    if missing:
        raise RuntimeError(
            f"[DOBOT BRINGUP] .env is missing required key(s): {', '.join(missing)}"
        )

    for key in ("DOBOT_ROBOT_LAN1_IP", "DOBOT_ROBOT_LAN2_IP"):
        try:
            ipaddress.IPv4Address(values[key])
        except ipaddress.AddressValueError as exc:
            raise RuntimeError(
                f"[DOBOT BRINGUP] {key} must be a valid IPv4 address: {values[key]}"
            ) from exc

    if values["DOBOT_ROBOT_LAN1_IP"] == values["DOBOT_ROBOT_LAN2_IP"]:
        raise RuntimeError(
            "[DOBOT BRINGUP] DOBOT_ROBOT_LAN1_IP and DOBOT_ROBOT_LAN2_IP must be different"
        )

    if values["DOBOT_ROBOT_TYPE"] != "cr10":
        raise RuntimeError(
            "[DOBOT BRINGUP] DOBOT_ROBOT_TYPE must be exactly 'cr10' for this CR10-only workspace"
        )

    if values["DOBOT_ROBOT_NUMBER"] != "1":
        raise RuntimeError(
            "[DOBOT BRINGUP] DOBOT_ROBOT_NUMBER must be exactly '1'; multi-robot configuration is not supported"
        )

    try:
        trajectory_duration = float(values["DOBOT_TRAJECTORY_DURATION"])
    except ValueError as exc:
        raise RuntimeError(
            "[DOBOT BRINGUP] DOBOT_TRAJECTORY_DURATION must be a finite number greater than zero"
        ) from exc
    if not math.isfinite(trajectory_duration) or trajectory_duration <= 0.0:
        raise RuntimeError(
            "[DOBOT BRINGUP] DOBOT_TRAJECTORY_DURATION must be a finite number greater than zero"
        )

    if not _ROS_NODE_NAME.fullmatch(values["DOBOT_ROBOT_NODE_NAME"]):
        raise RuntimeError(
            "[DOBOT BRINGUP] DOBOT_ROBOT_NODE_NAME must contain only letters, digits, and underscores and must not start with a digit"
        )

    # The project configuration is authoritative. This intentionally replaces
    # inherited values so a shell variable cannot silently select another robot.
    os.environ.update(values)
    return env_path.parent, env_path, values


project_root, env_path, project_config = _load_project_env()


def _package_manifests(root):
    source_root = root / "src"
    if not source_root.is_dir():
        raise RuntimeError(f"[DOBOT BRINGUP] Workspace source directory is missing: {source_root}")

    manifests = []
    package_names = set()
    for manifest in sorted(source_root.rglob("package.xml")):
        try:
            package_name = ET.parse(manifest).findtext("name")
        except ET.ParseError as exc:
            raise RuntimeError(f"[DOBOT BRINGUP] Invalid package manifest: {manifest}") from exc
        package_name = (package_name or "").strip()
        if not package_name or not _PACKAGE_NAME.fullmatch(package_name):
            raise RuntimeError(f"[DOBOT BRINGUP] Invalid package name in manifest: {manifest}")
        if package_name in package_names:
            raise RuntimeError(f"[DOBOT BRINGUP] Duplicate package name: {package_name}")
        package_names.add(package_name)
        manifests.append((package_name, manifest))

    if not manifests:
        raise RuntimeError(f"[DOBOT BRINGUP] No package manifests found below: {source_root}")
    return manifests


def _record_package_event(log_root, package_name, level, event, message):
    """Append one bounded JSONL event to a package-owned log file."""
    if not _PACKAGE_NAME.fullmatch(package_name):
        raise RuntimeError(f"[DOBOT BRINGUP] Invalid datalog package name: {package_name}")
    path = log_root / package_name / "events.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        if path.exists():
            with path.open("r", encoding="utf-8") as existing:
                event_count = sum(1 for _ in existing)
        else:
            event_count = 0
        mode = "w" if event_count >= _MAX_LOG_EVENTS else "a"
        record = {
            "timestamp": datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace(
                "+00:00", "Z"
            ),
            "package": package_name,
            "level": level,
            "event": event,
            "message": message,
        }
        with path.open(mode, encoding="utf-8") as stream:
            stream.write(json.dumps(record, separators=(",", ":")) + "\n")
    except OSError as exc:
        raise RuntimeError(f"[DOBOT BRINGUP] Cannot write package datalog: {path}") from exc


def _initialize_package_logs(root):
    log_root = root / "logs"
    packages = []
    for package_name, manifest in _package_manifests(root):
        # Parse each manifest here so a package with malformed XML fails before
        # the driver node is started.
        if ET.parse(manifest).findtext("name") != package_name:
            raise RuntimeError(f"[DOBOT BRINGUP] Package manifest name mismatch: {manifest}")
        _record_package_event(log_root, package_name, "INFO", "package_log_initialized", f"manifest={manifest}")
        packages.append(package_name)
    return log_root, packages


log_root, package_log_names = _initialize_package_logs(project_root)
lan1_ip = project_config["DOBOT_ROBOT_LAN1_IP"].strip()
lan2_ip = project_config["DOBOT_ROBOT_LAN2_IP"].strip()
robot_type = project_config["DOBOT_ROBOT_TYPE"]
robot_number = int(project_config["DOBOT_ROBOT_NUMBER"])
trajectory_duration = float(project_config["DOBOT_TRAJECTORY_DURATION"])
robot_node_name = project_config["DOBOT_ROBOT_NODE_NAME"]

print(f"[DOBOT BRINGUP] LAN1 (primary): {lan1_ip}")
print(f"[DOBOT BRINGUP] LAN2 (diagnostic failover): {lan2_ip}")
print(f"[DOBOT BRINGUP] Using Robot Type: {robot_type}")
print(f"[DOBOT BRINGUP] Loaded project config: {env_path}")
print(f"[DOBOT BRINGUP] Package datalogs: {log_root} ({len(package_log_names)} packages)")


dobot_ros2_params = [
    {"robot_lan1_ip": lan1_ip},
    {"robot_lan2_ip": lan2_ip},
    {"robot_type": robot_type},
    {"trajectory_duration": trajectory_duration},
    {"robot_node_name": robot_node_name},
    {"robot_number": robot_number},
    {"datalog_directory": str(log_root)},
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
