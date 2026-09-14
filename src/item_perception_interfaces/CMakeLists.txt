cmake_minimum_required(VERSION 3.8)
project(item_perception_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ItemCandidate.msg" "srv/GetItemPoses.srv"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
