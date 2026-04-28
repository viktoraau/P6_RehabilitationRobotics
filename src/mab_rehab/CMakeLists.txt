cmake_minimum_required(VERSION 3.16)
project(mab_rehab)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

option(
  MAB_REHAB_STRICT_DEPS
  "Fail configure if ros2_control dependencies are missing"
  OFF
)

find_package(hardware_interface QUIET)
find_package(controller_manager_msgs QUIET)
find_package(pluginlib QUIET)
find_package(rclcpp QUIET)
find_package(rclcpp_lifecycle QUIET)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RunCalibration.srv"
  "srv/SetTorqueBandwidth.srv"
  "srv/RunDriveTests.srv"
)

set(MAB_REHAB_PLUGIN_BUILDABLE ON)
if(
  NOT hardware_interface_FOUND OR
  NOT pluginlib_FOUND OR
  NOT rclcpp_FOUND OR
  NOT rclcpp_lifecycle_FOUND OR
  NOT controller_manager_msgs_FOUND)
  set(MAB_REHAB_PLUGIN_BUILDABLE OFF)
endif()

if(NOT MAB_REHAB_PLUGIN_BUILDABLE)
  message(WARNING
    "mab_rehab: ros2_control build dependencies are missing. "
    "Installing configuration/URDF only.")
  if(MAB_REHAB_STRICT_DEPS)
    message(FATAL_ERROR "Strict dependency mode is enabled and ros2_control deps were not found.")
  endif()
endif()

# Reuse the CANdle-SDK that already lives under candle_ros2/ in this workspace.
set(CANDLESDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../candle_ros2/CANdle-SDK")
if(MAB_REHAB_PLUGIN_BUILDABLE AND NOT EXISTS "${CANDLESDK_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "Expected CANdle-SDK at ${CANDLESDK_DIR}")
endif()

if(MAB_REHAB_PLUGIN_BUILDABLE)
  # Only add the SDK subdirectory if no other package in this workspace already did.
  if(NOT TARGET candle)
    set(CANDLESDK_BUILD_CANDLETOOL OFF CACHE BOOL "Disable candletool build" FORCE)
    set(CANDLESDK_BUILD_EXAMPLES OFF CACHE BOOL "Disable SDK examples build" FORCE)
    set(CANDLESDK_BUILD_PYTHON OFF CACHE BOOL "Disable SDK python build" FORCE)
    set(MAKE_TESTS OFF CACHE BOOL "Disable CANdle-SDK tests" FORCE)
    add_subdirectory("${CANDLESDK_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/candlesdk" EXCLUDE_FROM_ALL)
  endif()

  add_library(mab_rehab_hardware SHARED
    src/mab_system.cpp
    src/config/hardware_config.cpp
    src/config/joint_config.cpp
    src/transport/candle_transport.cpp
    src/transport/md_drive.cpp
    src/power/pds_manager.cpp
    src/rt/maintenance_gate.cpp
    src/rt/health_monitor.cpp
    src/rt/mode_switch_worker.cpp
    src/maintenance/controller_switcher.cpp
    src/maintenance/maintenance_services.cpp
    src/maintenance/md_config_uploader.cpp
  )
  target_include_directories(mab_rehab_hardware PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_include_directories(mab_rehab_hardware PRIVATE
    ${CANDLESDK_DIR}/candletool/include
    ${CANDLESDK_DIR}/candletool/3rd_party/mINI/src
    ${CANDLESDK_DIR}/candletool/3rd_party/CLI11/include
  )
  target_compile_definitions(mab_rehab_hardware PRIVATE
    DEFAULT_CANDLETOOL_CONFIG_DIR="/etc/candletool/config/motors/"
  )
  ament_target_dependencies(mab_rehab_hardware
    controller_manager_msgs
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(mab_rehab_hardware
    candle
    ${cpp_typesupport_target}
  )

  pluginlib_export_plugin_description_file(hardware_interface mab_rehab.xml)

  install(
    TARGETS mab_rehab_hardware
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch urdf docs DESTINATION share/${PROJECT_NAME})
install(FILES mab_rehab.xml DESTINATION share/${PROJECT_NAME})

if(MAB_REHAB_PLUGIN_BUILDABLE)
  ament_export_dependencies(
    controller_manager_msgs
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
