cmake_minimum_required(VERSION 3.14)

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
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

# Modern replacement for the deprecated/removed `ament_target_dependencies()`.
# Prefers the package's `${pkg_TARGETS}` list (set by rosidl for msg/srv pkgs
# and by recent ament_cmake versions for library pkgs); falls back to the
# `pkg::pkg` imported target when only that is available.
function(_ouster_link_ament_deps target visibility)
  foreach(_pkg IN LISTS ARGN)
    if(DEFINED ${_pkg}_TARGETS)
      target_link_libraries(${target} ${visibility} ${${_pkg}_TARGETS})
    elseif(TARGET ${_pkg}::${_pkg})
      target_link_libraries(${target} ${visibility} ${_pkg}::${_pkg})
    else()
      message(FATAL_ERROR
        "_ouster_link_ament_deps: cannot resolve dependency '${_pkg}' "
        "(neither ${_pkg}_TARGETS nor target ${_pkg}::${_pkg} is defined)")
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

# The Ouster SDK is vendored via the ouster-sdk submodule. Silence
# -Wpedantic / -Wdeprecated-declarations on its own compilation units so the
# workspace's pedantic compile flags don't fail the build on third-party code
# we don't own. Our wrapper (ouster_ros_library, os_*_node, etc.) is still
# compiled with the full diagnostic set.
foreach(_sdk_target ouster_client ouster_sensor ouster_pcap ouster_osf ouster_viz)
  if(TARGET ${_sdk_target})
    target_compile_options(${_sdk_target} PRIVATE
      -Wno-pedantic
      -Wno-deprecated-declarations
    )
  endif()
endforeach()

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
  # PUBLIC (unsupported)
    ouster_build
    OusterSDK::ouster_sensor
    pcl_common
  # PRIVATE (unsupported)
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

# ==== os_pinhole_component ====
create_ros2_component(os_pinhole_component
  "src/os_processing_node_base.cpp;src/os_pinhole_node.cpp"
  "tf2;tf2_geometry_msgs;tf2_ros"
)
rclcpp_components_register_node(os_pinhole_component
  PLUGIN "ouster_ros::OusterPinhole"
  EXECUTABLE os_pinhole
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
  _ouster_link_ament_deps(${PROJECT_NAME}_test PUBLIC
    rclcpp
    ouster_sensor_msgs
  )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    ${_ouster_ros_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_test ouster_ros_library)
endif()


# ==== Install ====
set(OUSTER_INSTALL_TARGETS
  ouster_ros_library
  os_sensor_component
  os_replay_component
  os_cloud_component
  os_image_component
  os_pinhole_component
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
ament_export_dependencies(rosidl_default_runtime)
ament_export_libraries(ouster_ros_library)
ament_export_dependencies(${ouster_ros_library_deps})
ament_package()
