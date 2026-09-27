cmake_minimum_required(VERSION 3.8)
project(tray_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})
install(PROGRAMS scripts/tray_teach scripts/tray_worker DESTINATION lib/${PROJECT_NAME})
install(FILES launch/tray_teach.launch.py DESTINATION share/${PROJECT_NAME}/launch)
install(FILES README.md LICENSE NOTICE.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_tray_teach test TIMEOUT 120
    ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:$ENV{PYTHONPATH}"
        "QT_QPA_PLATFORM=offscreen")
endif()
ament_package()
