cmake_minimum_required(VERSION 3.8)
project(IK)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(orientation_ik_3r_node
  src/orientation_ik_3r_node.cpp
)

target_include_directories(orientation_ik_3r_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(orientation_ik_3r_node
  rclcpp
  tf2_msgs
  geometry_msgs
  trajectory_msgs
)

target_compile_features(orientation_ik_3r_node PUBLIC cxx_std_17)

install(TARGETS
  orientation_ik_3r_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()