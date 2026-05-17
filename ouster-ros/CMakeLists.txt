cmake_minimum_required(VERSION 3.10.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/ouster-sdk/cmake)
include(DefaultBuildType)

# ==== Project Name ====
project(ouster_ros)

# ==== Requirements ====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ouster_sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

# Modern replacement for the deprecated/removed `ament_target_dependencies()`.
# Resolves each ament package against, in order of preference:
#   1. `${pkg}_TARGETS` (set by rosidl for msg/srv pkgs and by recent
#      ament_cmake versions for library pkgs — e.g. tf2 on Lyrical).
#   2. The `pkg::pkg` imported target when only that is available.
#   3. Legacy ament `${pkg}_INCLUDE_DIRS` / `${pkg}_LIBRARIES` variables
#      (still the only thing exposed by header-only packages like
#      `pcl_conversions` on Humble/Jazzy/Kilted).
#
# `visibility` is one of PUBLIC / PRIVATE / INTERFACE, or the special value
# PLAIN. PLAIN omits the keyword on `target_link_libraries` calls — required
# when the target was first linked with the plain signature (notably
# `ament_add_gtest` on Humble/Jazzy/Kilted, where the generated executable
# is wired with `target_link_libraries(<exe> <libs>)` and CMake then
# rejects any keyword call against the same target).
function(_ouster_link_ament_deps target visibility)
  if(visibility STREQUAL "PLAIN")
    set(_link_vis "")
    # target_include_directories has no plain signature. PRIVATE is correct
    # for the only target type we use PLAIN with today (test executables).
    set(_include_vis PRIVATE)
  else()
    set(_link_vis ${visibility})
    set(_include_vis ${visibility})
  endif()
  foreach(_pkg IN LISTS ARGN)
    if(DEFINED ${_pkg}_TARGETS)
      target_link_libraries(${target} ${_link_vis} ${${_pkg}_TARGETS})
    elseif(TARGET ${_pkg}::${_pkg})
      target_link_libraries(${target} ${_link_vis} ${_pkg}::${_pkg})
    elseif(DEFINED ${_pkg}_INCLUDE_DIRS OR DEFINED ${_pkg}_LIBRARIES)
      if(${_pkg}_INCLUDE_DIRS)
        target_include_directories(${target} SYSTEM ${_include_vis}
          ${${_pkg}_INCLUDE_DIRS})
      endif()
      if(${_pkg}_LIBRARIES)
        target_link_libraries(${target} ${_link_vis} ${${_pkg}_LIBRARIES})
      endif()
    else()
      message(FATAL_ERROR
        "_ouster_link_ament_deps: cannot resolve dependency '${_pkg}' "
        "(none of ${_pkg}_TARGETS, target ${_pkg}::${_pkg}, or legacy "
        "${_pkg}_INCLUDE_DIRS/${_pkg}_LIBRARIES are defined)")
    endif()
  endforeach()
endfunction()

# `ament_add_gtest_executable.cmake` ships in two flavors across ROS 2:
#   - Humble/Jazzy/Kilted: plain `target_link_libraries(<exe> <libs>)`
#   - Lyrical:             keyword `target_link_libraries(<exe> PRIVATE <libs>)`
# Mixing the two on the same target is a hard CMake error, so we detect
# which form ament uses and match it for our follow-up link calls on the
# test executable. The detection only runs when BUILD_TESTING is on.
function(_ouster_detect_gtest_link_mode out_var)
  set(${out_var} "PRIVATE" PARENT_SCOPE)  # safe default: keyword
  set(_gtest_module "${ament_cmake_gtest_DIR}/ament_add_gtest_executable.cmake")
  if(NOT EXISTS "${_gtest_module}")
    return()
  endif()
  file(READ "${_gtest_module}" _gtest_module_contents)
  # Look for target_link_libraries(...) calls without a PUBLIC/PRIVATE/
  # INTERFACE keyword in the first argument-position after the target.
  string(REGEX MATCHALL
    "target_link_libraries[ \t\n]*\\([^)]*\\)"
    _tll_calls "${_gtest_module_contents}")
  foreach(_call IN LISTS _tll_calls)
    if(NOT _call MATCHES "target_link_libraries[ \t\n]*\\([^)]*(PUBLIC|PRIVATE|INTERFACE)")
      set(${out_var} "PLAIN" PARENT_SCOPE)
      return()
    endif()
  endforeach()
endfunction()
# Use OpenCV libs variable (OpenCV::OpenCV target may not resolve on all systems)
if(NOT OpenCV_LIBS)
  set(OpenCV_LIBS ${OpenCV_LIBRARIES})
endif()

# ==== Options ====
add_compile_options(-Wall -Wextra)
if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
option(CMAKE_POSITION_INDEPENDENT_CODE "Build position independent code." ON)

set(_ouster_ros_INCLUDE_DIRS
  include
  ouster-sdk/ouster_sensor/include
  ouster-sdk/ouster_client/include
  ouster-sdk/ouster_pcap/include
  ouster-sdk/ouster_client/include/optional-lite
)

# ==== Libraries ====
set(_SAVE_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)

option(BUILD_VIZ "Turn off building VIZ" OFF)
option(BUILD_PCAP "Turn off building PCAP" OFF)
option(BUILD_OSF "Turn off building OSF" OFF)
option(BUILD_MAPPING "Turn off building MAPPING" OFF)
find_package(OusterSDK REQUIRED)

set(BUILD_SHARED_LIBS ${_SAVE_BUILD_SHARED_LIBS})

# catkin adds all include dirs to a single variable, don't try to use targets
include_directories(
  ${_ouster_ros_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

# use only MPL-licensed parts of eigen
add_compile_definitions(EIGEN_MPL2_ONLY)

set(OUSTER_TARGET_LINKS ouster_client)
if (BUILD_PCAP)
  list(APPEND OUSTER_TARGET_LINKS ouster_pcap)
endif()

add_library(ouster_ros_library SHARED
  src/impl/file_util.cpp
  src/os_ros.cpp
)

set(ouster_ros_library_deps
  rclcpp
  sensor_msgs
  geometry_msgs
  ouster_sensor_msgs
  pcl_conversions
  tf2
  tf2_eigen
)

_ouster_link_ament_deps(ouster_ros_library PUBLIC
  ${ouster_ros_library_deps}
)

target_link_libraries(ouster_ros_library
  PUBLIC
    ouster_build
    OusterSDK::ouster_sensor
    pcl_common
  PRIVATE
    ${OUSTER_TARGET_LINKS}
    ${OpenCV_LIBS}
)

# helper method to construct ouster-ros components
function(create_ros2_component
  component_lib_name
  src_list
  additional_dependencies
)

  add_library(${component_lib_name} SHARED
    ${src_list})
  target_compile_definitions(${component_lib_name}
    PRIVATE
      "OUSTER_ROS_BUILDING_DLL"
  )

  _ouster_link_ament_deps(${component_lib_name} PUBLIC
    rclcpp
    class_loader
    rclcpp_components
    rclcpp_lifecycle
    std_msgs
    sensor_msgs
    geometry_msgs
    ${additional_dependencies}
  )

  target_link_libraries(${component_lib_name}
    PUBLIC
      ouster_ros_library
  )

endfunction()


# ==== os_sensor_component ====
create_ros2_component(os_sensor_component
  "src/os_sensor_node_base.cpp;src/os_sensor_node.cpp"
  "std_srvs"
)
rclcpp_components_register_node(os_sensor_component
  PLUGIN "ouster_ros::OusterSensor"
  EXECUTABLE os_sensor
)

# ==== os_replay_component ====
create_ros2_component(os_replay_component
  "src/os_sensor_node_base.cpp;src/os_replay_node.cpp"
  ""
)
rclcpp_components_register_node(os_replay_component
  PLUGIN "ouster_ros::OusterReplay"
  EXECUTABLE os_replay
)

# ==== os_cloud_component ====
create_ros2_component(os_cloud_component
  "src/os_processing_node_base.cpp;src/os_cloud_node.cpp"
  "tf2_ros"
)
rclcpp_components_register_node(os_cloud_component
  PLUGIN "ouster_ros::OusterCloud"
  EXECUTABLE os_cloud
)

# ==== os_image_component ====
create_ros2_component(os_image_component
  "src/os_processing_node_base.cpp;src/os_image_node.cpp"
  ""
)
rclcpp_components_register_node(os_image_component
  PLUGIN "ouster_ros::OusterImage"
  EXECUTABLE os_image
)

# ==== os_driver_component ====
create_ros2_component(os_driver_component
  "src/os_sensor_node_base.cpp;src/os_sensor_node.cpp;src/os_driver_node.cpp"
  "std_srvs"
)
rclcpp_components_register_node(os_driver_component
  PLUGIN "ouster_ros::OusterDriver"
  EXECUTABLE os_driver
)


if (BUILD_PCAP)
# ==== os_pcap_component ====
create_ros2_component(os_pcap_component
  "src/os_sensor_node_base.cpp;src/os_pcap_node.cpp"
  ""
)
rclcpp_components_register_node(os_pcap_component
  PLUGIN "ouster_ros::OusterPcap"
  EXECUTABLE os_pcap
)
endif()

# ==== Test ====
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    src/os_ros.cpp
    test/test_main.cpp
    test/lock_free_ring_buffer_test.cpp
    test/point_accessor_test.cpp
    test/point_transform_test.cpp
    test/point_cloud_compose_test.cpp
  )

  if(TARGET ${PROJECT_NAME}_test)
    # Match the keyword/plain mode that ament_add_gtest used for this
    # target — see _ouster_detect_gtest_link_mode above.
    _ouster_detect_gtest_link_mode(_test_link_mode)
    _ouster_link_ament_deps(${PROJECT_NAME}_test ${_test_link_mode}
      rclcpp
      ouster_sensor_msgs
    )
    target_include_directories(${PROJECT_NAME}_test PRIVATE
      ${_ouster_ros_INCLUDE_DIRS}
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    if(_test_link_mode STREQUAL "PLAIN")
      target_link_libraries(${PROJECT_NAME}_test ouster_ros_library)
    else()
      target_link_libraries(${PROJECT_NAME}_test ${_test_link_mode} ouster_ros_library)
    endif()
  endif()
endif()


# ==== Install ====
set(OUSTER_INSTALL_TARGETS
  ouster_ros_library
  os_sensor_component
  os_replay_component
  os_cloud_component
  os_image_component
  os_driver_component
)
if (BUILD_PCAP)
  list(APPEND OUSTER_INSTALL_TARGETS os_pcap_component)
endif()

install(
  TARGETS
    ${OUSTER_INSTALL_TARGETS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    include/${PROJECT_NAME}/
)

install(
  DIRECTORY
    ouster-sdk/ouster_client/include/ouster
    ouster-sdk/ouster_client/include/optional-lite/nonstd
  DESTINATION include
)

install(
  FILES
    ../LICENSE
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(ouster_ros_library)
ament_export_dependencies(${ouster_ros_library_deps})
ament_package()
