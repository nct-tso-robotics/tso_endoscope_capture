cmake_minimum_required(VERSION 3.16)
project(endoscope_capture LANGUAGES CXX)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

set(CAPTURE_ROS_VERSION "$ENV{ROS_VERSION}" CACHE STRING "ROS major version (1 or 2)")
if(NOT CAPTURE_ROS_VERSION)
  set(CAPTURE_ROS_VERSION "1" CACHE STRING "ROS major version (1 or 2)" FORCE)
endif()
if(NOT CAPTURE_ROS_VERSION MATCHES "^[12]$")
  message(FATAL_ERROR "CAPTURE_ROS_VERSION must be 1 or 2")
endif()
set(ENV{ROS_VERSION} "${CAPTURE_ROS_VERSION}")

# ROS resource lookup recursively searches a source package and prunes only
# hidden directories. A visible install tree inside it duplicates launch files
# and executable names even when the build tree contains CATKIN_IGNORE.
file(RELATIVE_PATH capture_install_relative "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_INSTALL_PREFIX}")
if(NOT capture_install_relative MATCHES "^\\.\\./" AND
   NOT IS_ABSOLUTE "${capture_install_relative}" AND
   NOT capture_install_relative MATCHES "(^|/)\\.[^./][^/]*(/|$)")
  message(FATAL_ERROR "Install inside a hidden directory or outside the package source to keep ROS resource lookup unambiguous")
endif()

if(CAPTURE_ROS_VERSION STREQUAL "2")
  enable_language(C)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rcl_interfaces REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(camera_info_manager REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/CaptureStatus.msg"
    DEPENDENCIES std_msgs)
  rosidl_get_typesupport_target(capture_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)
  set(capture_share_destination "share/${PROJECT_NAME}")
else()
  find_package(catkin REQUIRED COMPONENTS
    roscpp image_transport sensor_msgs camera_info_manager std_msgs
    message_generation)
  add_message_files(FILES CaptureStatus.msg)
  generate_messages(DEPENDENCIES std_msgs)
  catkin_package(CATKIN_DEPENDS
    roscpp image_transport sensor_msgs camera_info_manager std_msgs message_runtime)
  set(capture_ros_includes ${catkin_INCLUDE_DIRS})
  set(capture_ros_libraries ${catkin_LIBRARIES})
  set(capture_share_destination "${CATKIN_PACKAGE_SHARE_DESTINATION}")
endif()
set(CAPTURE_BUILD_ROS ON CACHE BOOL "Build the integrated ROS capture node" FORCE)
include(CTest)
add_subdirectory(capture_driver)
if(BUILD_TESTING AND CAPTURE_ROS_VERSION STREQUAL "1")
  add_test(NAME capture_launch_discovery
    COMMAND bash
      "${CMAKE_CURRENT_SOURCE_DIR}/capture_driver/tests/test_launch_discovery.sh"
      "${CMAKE_CURRENT_SOURCE_DIR}" "${CATKIN_DEVEL_PREFIX}")
  set_tests_properties(capture_launch_discovery PROPERTIES TIMEOUT 20)
endif()
install(DIRECTORY config DESTINATION ${capture_share_destination})
if(CAPTURE_ROS_VERSION STREQUAL "2")
  install(FILES launch/endoscope.launch.xml DESTINATION ${capture_share_destination}/launch)
else()
  install(FILES launch/endoscope.launch DESTINATION ${capture_share_destination}/launch)
endif()
install(FILES LICENSE NOTICE.md capture_driver/LICENSE.upstream
  DESTINATION ${capture_share_destination})
if(CAPTURE_ROS_VERSION STREQUAL "2")
  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
endif()
