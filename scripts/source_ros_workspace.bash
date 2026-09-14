#!/usr/bin/env bash

# Canonical shell environment for this ROS 2 Humble workspace. Source this
# file; executing it cannot update the caller's environment.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "ERROR: source scripts/source_ros_workspace.bash from the workspace root" >&2
  exit 2
fi

_picknplace_source_ros_environment() {
  local script_dir project_root env_path ros_setup workspace_setup
  local line key value required_key ip_value octet
  local -A values=()
  local -a octets=()
  local -a required_keys=(
    ROS_LOCALHOST_ONLY
    DOBOT_ROBOT_LAN1_IP
    DOBOT_ROBOT_LAN2_IP
    DOBOT_CONNECTION_TIMEOUT_MS
    DOBOT_ROBOT_TYPE
    DOBOT_ROBOT_NUMBER
    DOBOT_TRAJECTORY_DURATION
    DOBOT_ROBOT_NODE_NAME
    ORBBEC_CAMERA_1_NAME
    ORBBEC_CAMERA_1_SERIAL
    ORBBEC_CAMERA_2_NAME
    ORBBEC_CAMERA_2_SERIAL
    ORBBEC_DEVICE_PRESET
    ORBBEC_ENABLE_COLOR
    ORBBEC_ENABLE_DEPTH
    ORBBEC_DEPTH_REGISTRATION
    ORBBEC_ALIGN_TARGET_STREAM
    ORBBEC_ALIGN_MODE
    ORBBEC_ENABLE_FRAME_SYNC
    ORBBEC_ENABLE_TEMPORAL_FILTER
    ORBBEC_COLOR_WIDTH
    ORBBEC_COLOR_HEIGHT
    ORBBEC_COLOR_FPS
    ORBBEC_DEPTH_WIDTH
    ORBBEC_DEPTH_HEIGHT
    ORBBEC_DEPTH_FPS
    ORBBEC_ENABLE_POINT_CLOUD
    ORBBEC_ENUMERATE_NET_DEVICE
    ORBBEC_SCAN_TIMEOUT_SEC
    ORBBEC_STARTUP_TIMEOUT_SEC
    ORBBEC_HEALTH_TIMEOUT_SEC
    ORBBEC_CHECK_PERIOD_SEC
    ORBBEC_MAX_ATTEMPTS
    ORBBEC_RETRY_DELAY_SEC
    ORBBEC_SHUTDOWN_TIMEOUT_SEC
  )

  script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)" || return 1
  project_root="$(cd -- "${script_dir}/.." && pwd -P)" || return 1
  env_path="${project_root}/.env"
  ros_setup="/opt/ros/humble/setup.bash"
  workspace_setup="${project_root}/install/setup.bash"

  if [[ ! -f "${env_path}" ]]; then
    echo "ERROR: required project configuration is missing: ${env_path}" >&2
    return 1
  fi
  if [[ ! -f "${ros_setup}" ]]; then
    echo "ERROR: required ROS 2 Humble setup is missing: ${ros_setup}" >&2
    return 1
  fi
  if [[ ! -f "${workspace_setup}" ]]; then
    echo "ERROR: workspace is not built: ${workspace_setup}" >&2
    return 1
  fi

  while IFS= read -r line || [[ -n "${line}" ]]; do
    if [[ -z "${line}" || "${line}" == \#* ]]; then
      continue
    fi
    if [[ "${line}" == export\ * || "${line}" != *=* ]]; then
      echo "ERROR: invalid project .env syntax: ${env_path}" >&2
      return 1
    fi

    key="${line%%=*}"
    value="${line#*=}"
    if [[ ! "${key}" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
      echo "ERROR: invalid project .env key: ${key}" >&2
      return 1
    fi
    if [[ "${value}" == [[:space:]]* || "${value}" == *[[:space:]] ]]; then
      echo "ERROR: project .env values must not have surrounding whitespace: ${key}" >&2
      return 1
    fi
    case "${key}" in
      ROS_LOCALHOST_ONLY|DOBOT_ROBOT_LAN1_IP|DOBOT_ROBOT_LAN2_IP|DOBOT_CONNECTION_TIMEOUT_MS|DOBOT_ROBOT_TYPE|DOBOT_ROBOT_NUMBER|DOBOT_TRAJECTORY_DURATION|DOBOT_ROBOT_NODE_NAME|ORBBEC_CAMERA_1_NAME|ORBBEC_CAMERA_1_SERIAL|ORBBEC_CAMERA_2_NAME|ORBBEC_CAMERA_2_SERIAL|ORBBEC_DEVICE_PRESET|ORBBEC_ENABLE_COLOR|ORBBEC_ENABLE_DEPTH|ORBBEC_DEPTH_REGISTRATION|ORBBEC_ALIGN_TARGET_STREAM|ORBBEC_ALIGN_MODE|ORBBEC_ENABLE_FRAME_SYNC|ORBBEC_ENABLE_TEMPORAL_FILTER|ORBBEC_COLOR_WIDTH|ORBBEC_COLOR_HEIGHT|ORBBEC_COLOR_FPS|ORBBEC_DEPTH_WIDTH|ORBBEC_DEPTH_HEIGHT|ORBBEC_DEPTH_FPS|ORBBEC_ENABLE_POINT_CLOUD|ORBBEC_ENUMERATE_NET_DEVICE|ORBBEC_SCAN_TIMEOUT_SEC|ORBBEC_STARTUP_TIMEOUT_SEC|ORBBEC_HEALTH_TIMEOUT_SEC|ORBBEC_CHECK_PERIOD_SEC|ORBBEC_MAX_ATTEMPTS|ORBBEC_RETRY_DELAY_SEC|ORBBEC_SHUTDOWN_TIMEOUT_SEC)
        ;;
      *)
        echo "ERROR: unsupported project .env key: ${key}" >&2
        return 1
        ;;
    esac
    if [[ -n "${values[${key}]+present}" ]]; then
      echo "ERROR: duplicate project .env key: ${key}" >&2
      return 1
    fi
    values["${key}"]="${value}"
  done < "${env_path}"

  for required_key in "${required_keys[@]}"; do
    if [[ -z "${values[${required_key}]+present}" ]]; then
      echo "ERROR: project .env is missing required key: ${required_key}" >&2
      return 1
    fi
  done
  if [[ "${values[ROS_LOCALHOST_ONLY]}" != "1" ]]; then
    echo "ERROR: ROS_LOCALHOST_ONLY must be exactly 1" >&2
    return 1
  fi
  for required_key in DOBOT_ROBOT_LAN1_IP DOBOT_ROBOT_LAN2_IP; do
    ip_value="${values[${required_key}]}"
    if [[ ! "${ip_value}" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
      echo "ERROR: ${required_key} must be a valid IPv4 address" >&2
      return 1
    fi
    IFS='.' read -r -a octets <<< "${ip_value}"
    for octet in "${octets[@]}"; do
      if [[ ! "${octet}" =~ ^(0|[1-9][0-9]{0,2})$ ]] || ((10#${octet} > 255)); then
        echo "ERROR: ${required_key} must be a valid IPv4 address" >&2
        return 1
      fi
    done
  done
  if [[ "${values[DOBOT_ROBOT_LAN1_IP]}" == "${values[DOBOT_ROBOT_LAN2_IP]}" ]]; then
    echo "ERROR: DOBOT_ROBOT_LAN1_IP and DOBOT_ROBOT_LAN2_IP must be different" >&2
    return 1
  fi
  if [[ ! "${values[DOBOT_CONNECTION_TIMEOUT_MS]}" =~ ^[1-9][0-9]*$ ]] ||
     ((10#${values[DOBOT_CONNECTION_TIMEOUT_MS]} < 100 || 10#${values[DOBOT_CONNECTION_TIMEOUT_MS]} > 60000)); then
    echo "ERROR: DOBOT_CONNECTION_TIMEOUT_MS must be an integer from 100 through 60000" >&2
    return 1
  fi
  if [[ "${values[DOBOT_ROBOT_TYPE]}" != "cr10" ]]; then
    echo "ERROR: DOBOT_ROBOT_TYPE must be exactly cr10" >&2
    return 1
  fi
  if [[ "${values[DOBOT_ROBOT_NUMBER]}" != "1" ]]; then
    echo "ERROR: DOBOT_ROBOT_NUMBER must be exactly 1" >&2
    return 1
  fi
  if ! /usr/bin/python3 -c \
    'import math, sys; value = float(sys.argv[1]); sys.exit(0 if math.isfinite(value) and value > 0.0 else 1)' \
    "${values[DOBOT_TRAJECTORY_DURATION]}" 2>/dev/null; then
    echo "ERROR: DOBOT_TRAJECTORY_DURATION must be a finite number greater than zero" >&2
    return 1
  fi
  if [[ ! "${values[DOBOT_ROBOT_NODE_NAME]}" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
    echo "ERROR: DOBOT_ROBOT_NODE_NAME must be a valid ROS node name" >&2
    return 1
  fi
  for required_key in ORBBEC_CAMERA_1_NAME ORBBEC_CAMERA_2_NAME; do
    if [[ -n "${values[${required_key}]}" && ! "${values[${required_key}]}" =~ ^[A-Za-z][A-Za-z0-9_]*$ ]]; then
      echo "ERROR: ${required_key} must be empty or a valid ROS name" >&2
      return 1
    fi
  done
  for required_key in ORBBEC_CAMERA_1_SERIAL ORBBEC_CAMERA_2_SERIAL; do
    if [[ -n "${values[${required_key}]}" && ! "${values[${required_key}]}" =~ ^[A-Za-z0-9_.:-]+$ ]]; then
      echo "ERROR: ${required_key} contains invalid characters" >&2
      return 1
    fi
  done
  if [[ -n "${values[ORBBEC_CAMERA_1_NAME]}" &&
        "${values[ORBBEC_CAMERA_1_NAME]}" == "${values[ORBBEC_CAMERA_2_NAME]}" ]]; then
    echo "ERROR: configured Orbbec camera names must be unique" >&2
    return 1
  fi
  if [[ -n "${values[ORBBEC_CAMERA_1_SERIAL]}" &&
        "${values[ORBBEC_CAMERA_1_SERIAL]}" == "${values[ORBBEC_CAMERA_2_SERIAL]}" ]]; then
    echo "ERROR: configured Orbbec serial numbers must be unique" >&2
    return 1
  fi
  if [[ "${values[ORBBEC_DEVICE_PRESET]}" != "High Accuracy" ]]; then
    echo "ERROR: ORBBEC_DEVICE_PRESET must be exactly High Accuracy" >&2
    return 1
  fi
  for required_key in ORBBEC_ENABLE_COLOR ORBBEC_ENABLE_DEPTH ORBBEC_DEPTH_REGISTRATION ORBBEC_ENABLE_FRAME_SYNC ORBBEC_ENABLE_TEMPORAL_FILTER ORBBEC_ENABLE_POINT_CLOUD ORBBEC_ENUMERATE_NET_DEVICE; do
    if [[ "${values[${required_key}]}" != "true" && "${values[${required_key}]}" != "false" ]]; then
      echo "ERROR: ${required_key} must be exactly true or false" >&2
      return 1
    fi
  done
  if [[ "${values[ORBBEC_ENABLE_COLOR]}" != "true" || "${values[ORBBEC_ENABLE_DEPTH]}" != "true" ]]; then
    echo "ERROR: ORBBEC_ENABLE_COLOR and ORBBEC_ENABLE_DEPTH must be exactly true" >&2
    return 1
  fi
  if [[ "${values[ORBBEC_ENUMERATE_NET_DEVICE]}" != "false" ]]; then
    echo "ERROR: ORBBEC_ENUMERATE_NET_DEVICE must be exactly false" >&2
    return 1
  fi
  if [[ "${values[ORBBEC_ALIGN_TARGET_STREAM]}" != "COLOR" || "${values[ORBBEC_ALIGN_MODE]}" != "SW" ]]; then
    echo "ERROR: Orbbec alignment must be exactly target COLOR and mode SW" >&2
    return 1
  fi
  for required_key in ORBBEC_COLOR_WIDTH ORBBEC_COLOR_HEIGHT ORBBEC_COLOR_FPS ORBBEC_DEPTH_WIDTH ORBBEC_DEPTH_HEIGHT ORBBEC_DEPTH_FPS; do
    if [[ ! "${values[${required_key}]}" =~ ^[1-9][0-9]*$ ]]; then
      echo "ERROR: ${required_key} must be a canonical positive integer" >&2
      return 1
    fi
  done
  for required_key in ORBBEC_SCAN_TIMEOUT_SEC ORBBEC_STARTUP_TIMEOUT_SEC ORBBEC_HEALTH_TIMEOUT_SEC ORBBEC_CHECK_PERIOD_SEC ORBBEC_SHUTDOWN_TIMEOUT_SEC; do
    if ! /usr/bin/python3 -c \
      'import math, sys; value = float(sys.argv[1]); sys.exit(0 if math.isfinite(value) and value > 0.0 else 1)' \
      "${values[${required_key}]}" 2>/dev/null; then
      echo "ERROR: ${required_key} must be a finite number greater than zero" >&2
      return 1
    fi
  done
  if [[ "${values[ORBBEC_MAX_ATTEMPTS]}" != "3" ]]; then
    echo "ERROR: ORBBEC_MAX_ATTEMPTS must be exactly 3" >&2
    return 1
  fi
  if [[ "${values[ORBBEC_RETRY_DELAY_SEC]}" != "3" ]]; then
    echo "ERROR: ORBBEC_RETRY_DELAY_SEC must be exactly 3" >&2
    return 1
  fi

  # Remove inherited ROS overlays so no package, Python module, or library can
  # be selected from the previous Dobot workspace.
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
  unset PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH

  export ROS_LOCALHOST_ONLY="${values[ROS_LOCALHOST_ONLY]}"
  source "${ros_setup}" || return 1
  source "${workspace_setup}" || return 1

  if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
    echo "ERROR: expected ROS_DISTRO=humble after sourcing ${ros_setup}" >&2
    return 1
  fi
}

if ! _picknplace_source_ros_environment; then
  unset -f _picknplace_source_ros_environment
  return 1
fi
unset -f _picknplace_source_ros_environment
