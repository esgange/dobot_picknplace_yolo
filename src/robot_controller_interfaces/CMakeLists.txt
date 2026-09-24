cmake_minimum_required(VERSION 3.8)
project(robot_controller_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoHome.action"
  "action/PickItem.action"
  "action/ReplayCalibration.action"
  "msg/CalibrationPosition.msg"
  "msg/ControllerStatus.msg"
  "srv/Command.srv"
  "srv/Configure.srv"
  "srv/Preview.srv"
  "srv/SetGlobalSpeed.srv"
  "srv/CaptureCalibration.srv"
  DEPENDENCIES action_msgs std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
