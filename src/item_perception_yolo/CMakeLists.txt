cmake_minimum_required(VERSION 3.8)
project(item_perception_yolo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(YOLO_RUNTIME_DIR "${CMAKE_CURRENT_BINARY_DIR}/yolo_runtime")
file(REMOVE_RECURSE "${YOLO_RUNTIME_DIR}")
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/extract_yolo_runtime.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/wheels" "${YOLO_RUNTIME_DIR}"
  RESULT_VARIABLE YOLO_EXTRACT_RESULT
  ERROR_VARIABLE YOLO_EXTRACT_ERROR
)
if(NOT YOLO_EXTRACT_RESULT EQUAL 0)
  message(FATAL_ERROR "Required offline YOLO runtime extraction failed: ${YOLO_EXTRACT_ERROR}")
endif()
install(DIRECTORY "${YOLO_RUNTIME_DIR}/" DESTINATION lib/${PROJECT_NAME}/yolo_runtime
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)

ament_python_install_package(
  item_perception_yolo
  PACKAGE_DIR python/item_perception_yolo
)

install(PROGRAMS
  scripts/bin_teach
  scripts/platform_teach
  scripts/item_teach
  scripts/item_detect
  scripts/item_preview_worker
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  launch/bin_teach.launch.py
  launch/platform_teach.launch.py
  launch/item_teach.launch.py
  launch/item_detect.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)

install(FILES README.md QUICKSTART.md LICENSE NOTICE.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_teach_workflows
    test
    TIMEOUT 120
    ENV
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:${CMAKE_CURRENT_SOURCE_DIR}/../camera_calibration/python:$ENV{PYTHONPATH}"
  )
endif()

ament_package()
