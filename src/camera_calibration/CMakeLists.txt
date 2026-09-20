cmake_minimum_required(VERSION 3.8)
project(camera_calibration)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(OPENCV_WHEEL_NAME
  "opencv_python-4.10.0.84-cp37-abi3-manylinux_2_17_x86_64.manylinux2014_x86_64.whl"
)
set(OPENCV_WHEEL_SHA256
  "9ace140fc6d647fbe1c692bcb2abce768973491222c067c131d80957c595b71f"
)
get_filename_component(WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(OPENCV_WHEEL "${WORKSPACE_ROOT}/third_party/wheels/${OPENCV_WHEEL_NAME}")
if(NOT EXISTS "${OPENCV_WHEEL}")
  message(FATAL_ERROR "Required offline OpenCV wheel is missing: ${OPENCV_WHEEL}")
endif()
file(SHA256 "${OPENCV_WHEEL}" OPENCV_WHEEL_ACTUAL_SHA256)
if(NOT OPENCV_WHEEL_ACTUAL_SHA256 STREQUAL OPENCV_WHEEL_SHA256)
  message(FATAL_ERROR
    "OpenCV wheel checksum mismatch: expected=${OPENCV_WHEEL_SHA256} "
    "actual=${OPENCV_WHEEL_ACTUAL_SHA256} file=${OPENCV_WHEEL}"
  )
endif()

set(OPENCV_RUNTIME_DIR "${CMAKE_CURRENT_BINARY_DIR}/opencv_runtime")
file(REMOVE_RECURSE "${OPENCV_RUNTIME_DIR}")
file(MAKE_DIRECTORY "${OPENCV_RUNTIME_DIR}")
execute_process(
  COMMAND "${Python3_EXECUTABLE}" -m zipfile -e
    "${OPENCV_WHEEL}" "${OPENCV_RUNTIME_DIR}"
  RESULT_VARIABLE OPENCV_EXTRACT_RESULT
  ERROR_VARIABLE OPENCV_EXTRACT_ERROR
)
if(NOT OPENCV_EXTRACT_RESULT EQUAL 0)
  message(FATAL_ERROR
    "Failed to extract the required offline OpenCV wheel: ${OPENCV_EXTRACT_ERROR}"
  )
endif()
foreach(REQUIRED_RUNTIME_FILE
    "cv2/__init__.py"
    "cv2/cv2.abi3.so"
    "opencv_python-4.10.0.84.dist-info/LICENSE.txt"
    "opencv_python-4.10.0.84.dist-info/LICENSE-3RD-PARTY.txt")
  if(NOT EXISTS "${OPENCV_RUNTIME_DIR}/${REQUIRED_RUNTIME_FILE}")
    message(FATAL_ERROR
      "Extracted OpenCV runtime is incomplete: ${REQUIRED_RUNTIME_FILE}"
    )
  endif()
endforeach()

ament_python_install_package(
  camera_calibration_gui
  PACKAGE_DIR python/camera_calibration_gui
)

install(PROGRAMS
  scripts/camera_calibration_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# OpenCV's binary loader recursively imports its Python package when ament
# symlink-install puts the package in install/ but resolves its files to build/.
# Keep this native runtime self-contained as real installed files in every mode.
install(CODE
  "execute_process(
    COMMAND \"${Python3_EXECUTABLE}\"
      \"${CMAKE_CURRENT_SOURCE_DIR}/scripts/install_opencv_runtime.py\"
      \"${OPENCV_RUNTIME_DIR}\"
      \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/opencv_runtime\"
    RESULT_VARIABLE OPENCV_INSTALL_RESULT
    ERROR_VARIABLE OPENCV_INSTALL_ERROR
  )
  if(NOT OPENCV_INSTALL_RESULT EQUAL 0)
    message(FATAL_ERROR
      \"Required private OpenCV runtime copy failed: \${OPENCV_INSTALL_ERROR}\")
  endif()")

install(FILES README.md LICENSE NOTICE.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_calibration_core
    test/test_calibration_core.py
    TIMEOUT 180
    ENV
      "PYTHONPATH=${OPENCV_RUNTIME_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/python:$ENV{PYTHONPATH}"
      "CAMERA_CALIBRATION_TEST_OPENCV_RUNTIME=${OPENCV_RUNTIME_DIR}"
  )
  ament_add_pytest_test(
    test_runtime_install
    test/test_runtime_install.py
    TIMEOUT 30
  )
endif()

ament_package()
