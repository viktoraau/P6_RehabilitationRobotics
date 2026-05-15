cmake_minimum_required(VERSION 3.8)
project(wrist_games_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartCalibration.srv"
  "srv/StartGame.srv"
  "srv/StartAirplaneGame.srv"
  "srv/SetStiffness.srv"
  "srv/SetStiffnessScale.srv"
  "srv/SetDampingGain.srv"
  "srv/SetInertia.srv"
  "srv/SetAdmittanceEnabled.srv"
  "srv/SetReferenceJoint.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
