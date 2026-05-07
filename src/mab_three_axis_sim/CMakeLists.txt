cmake_minimum_required(VERSION 3.16)
project(mab_three_axis_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  docs
  launch
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
