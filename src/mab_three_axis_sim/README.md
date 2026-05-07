# mab_three_axis_sim

Simulation-only ROS 2 package for the current 3-DoF rehabilitation robot model.
It launches RViz, `robot_state_publisher`, and a fake `ros2_control` stack based on
`mock_components/GenericSystem`, so we can test standard position, velocity, and
trajectory commands before moving to the real hardware.

The robot geometry and meshes now come from `complete_system_urdf_description`
(the CAD-exported description package in `Steve_robot`), while this package owns
the fake-controller bringup, RViz config, and command-controller setup.

## What it provides

- CAD-based robot description from `complete_system_urdf_description`
- Standardized movable joint names for the control stack:
  - `joint_1`
  - `joint_2`
  - `joint_3`
- Fake `ros2_control` interfaces for:
  - `position`
  - `velocity`
- Controllers:
  - `joint_state_broadcaster`
  - `joint_trajectory_controller`
  - `joint_group_position_controller`
  - `joint_group_velocity_controller`
- RViz config with `base_link` as the fixed frame

## Build

Run from the repository root so both `Steve_robot` and `ros2_ws/src` are in the
same colcon workspace:

```bash
cd ~/P6_RehabilitationRobotics
source /opt/ros/jazzy/setup.bash
colcon build --packages-select complete_system_urdf_description mab_three_axis_sim mab_orientation_ik computed_torque_controller
source install/setup.bash
```

## Launch

Default bringup:

```bash
ros2 launch mab_three_axis_sim mab_three_axis_sim.launch.py
```

Useful launch overrides:

```bash
ros2 launch mab_three_axis_sim mab_three_axis_sim.launch.py active_controller:=position
ros2 launch mab_three_axis_sim mab_three_axis_sim.launch.py active_controller:=velocity
ros2 launch mab_three_axis_sim mab_three_axis_sim.launch.py active_controller:=none
ros2 launch mab_three_axis_sim mab_three_axis_sim.launch.py use_rviz:=false
ros2 launch mab_three_axis_sim mab_three_axis_sim.launch.py enable_orientation_ik:=false
```

The supported values for `active_controller` are:

- `trajectory`
- `position`
- `velocity`
- `none`

## Inspect the running controllers

```bash
ros2 control list_controllers -c /controller_manager
ros2 topic echo /joint_states
```

## Controller switching

Only one command controller should be active at a time.

Switch to trajectory control:

```bash
ros2 control switch_controllers \
  --deactivate joint_group_position_controller joint_group_velocity_controller \
  --activate joint_trajectory_controller \
  --best-effort
```

Switch to position control:

```bash
ros2 control switch_controllers \
  --deactivate joint_trajectory_controller joint_group_velocity_controller \
  --activate joint_group_position_controller \
  --best-effort
```

Switch to velocity control:

```bash
ros2 control switch_controllers \
  --deactivate joint_trajectory_controller joint_group_position_controller \
  --activate joint_group_velocity_controller \
  --best-effort
```

## Command examples

Trajectory command:

```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [\"joint_1\", \"joint_2\", \"joint_3\"], points: [{positions: [0.40, 0.20, -0.15], time_from_start: {sec: 2, nanosec: 0}}]}"
```

Position command:

```bash
ros2 topic pub --once /joint_group_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.20, -0.10, 0.30]}"
```

Velocity command:

```bash
ros2 topic pub --once /joint_group_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.15, 0.00, -0.10]}"
```

Orientation pose command through IK:

```bash
ros2 topic pub --once /orientation_ik_bridge/pose \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.1736482, z: 0.0, w: 0.9848078}}}"
```

Orientation twist command through IK:

```bash
ros2 topic pub --once /orientation_ik_bridge/twist \
  geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.15, z: 0.0}}}"
```

Orientation trajectory command through IK:

```bash
ros2 topic pub --once /orientation_ik_bridge/trajectory \
  trajectory_msgs/msg/MultiDOFJointTrajectory \
  "{header: {frame_id: 'base_link'}, joint_names: ['tool0'], points: [{transforms: [{translation: {x: 0.0, y: 0.0, z: 0.0}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}], time_from_start: {sec: 1}}, {transforms: [{translation: {x: 0.0, y: 0.0, z: 0.0}, rotation: {x: 0.0, y: 0.2588190, z: 0.0, w: 0.9659258}}], time_from_start: {sec: 3}}]}"
```

## Notes

- This is RViz plus fake controller simulation only. It is not a physics simulator.
- The active robot description now comes from the CAD-exported
  `complete_system_urdf_description` package.
- The old placeholder Xacro is kept in this package only as a reference while the
  real model is being integrated.
- `computed_torque_controller` is left unchanged. This package only guarantees
  compatible joint names and `/joint_states` output so it can be integrated later.

## Updating it later

When the CAD files, real measurements, or final URDF become available, follow:

- `docs/ADAPTING_TO_REAL_URDF_AND_CAD.md`

That guide explains which files to update, what measurements are needed, how to
keep controller compatibility, and what to re-test after replacing the
placeholder model.
