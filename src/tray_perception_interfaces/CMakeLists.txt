cmake_minimum_required(VERSION 3.8)
project(tray_perception_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrayPose.msg" "msg/PlacementDepthRequest.msg" "msg/PlacementDepthResult.msg"
  "srv/GetTrayPose.srv"
  DEPENDENCIES geometry_msgs std_msgs)
ament_export_dependencies(rosidl_default_runtime)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
