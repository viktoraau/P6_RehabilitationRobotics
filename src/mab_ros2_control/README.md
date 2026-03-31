# MAB ROS 2 Control - Controller Switching and Interface Command Syntax

This README is a quick reference for the ROS 2 controllers in `mab_ros2_control`.

## Controllers in this package

Configured in `config/mab_three_axis.controllers.yaml`:

- `joint_state_broadcaster`
- `joint_trajectory_controller`
- `joint_group_position_controller`
- `joint_group_velocity_controller`

Joint names:

- `joint_1`
- `joint_2`
- `joint_3`

## 1) Start the bringup

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mab_ros2_control mab_three_axis_bringup.launch.py
```

Default launch behavior in this repo:

- `joint_state_broadcaster` starts active
- `joint_group_velocity_controller` starts active
- `joint_trajectory_controller` starts inactive
- `joint_group_position_controller` starts inactive
- Select a different startup controller with `active_controller:=velocity|position|trajectory|effort|none`
- High-rate bench tests can use `fast_mode:=true controller_update_rate_hz:=200`

High-rate bench example for the single connected `MD941`:

```bash
ros2 launch mab_ros2_control mab_three_axis_bringup.launch.py \
  data_rate:=5M md_id_1:=0 md_id_2:=0 md_id_3:=941 \
  active_controller:=none fast_mode:=true controller_update_rate_hz:=200
```

## 2) Inspect controller + interface state

```bash
ros2 control list_controllers 
ros2 control list_hardware_interfaces -
```

## 3) Generic controller switch syntax

Only one command controller should be active at a time.

```bash
ros2 control switch_controllers \
  --deactivate <controller_1> <controller_2> ... \
  --activate <controller_to_activate> \
  --best-effort
```

## 4) Switch to each command mode

### Switch to position mode

```bash
ros2 control switch_controllers \
  --deactivate joint_trajectory_controller joint_group_velocity_controller \
  --activate joint_group_position_controller \
  --best-effort
```

### Switch to velocity mode

```bash
ros2 control switch_controllers --deactivate joint_trajectory_controller   joint_group_position_controller --activate joint_group_velocity_controller --best-effort
```


## 5) Command each interface

### Position interface (`joint_group_position_controller`)

Topic:

- `/joint_group_position_controller/commands`

Message type:

- `std_msgs/msg/Float64MultiArray`

Syntax:

```bash
ros2 topic pub --once /joint_group_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, -2.10]}"
```

### Velocity interface (`joint_group_velocity_controller`)

Topic:

- `/joint_group_velocity_controller/commands`

Message type:

- `std_msgs/msg/Float64MultiArray`

Syntax:

```bash
ros2 topic pub --once /joint_group_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.20, 0.00, 0.00]}"
```

### Trajectory interface (`joint_trajectory_controller`)

Topic:

- `/joint_trajectory_controller/joint_trajectory`

Message type:

- `trajectory_msgs/msg/JointTrajectory`

Syntax:

```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [\"joint_1\", \"joint_2\", \"joint_3\"], points: [{positions: [0.20, 0.10, -0.10], time_from_start: {sec: 2, nanosec: 0}}]}"
```

## 6) Quick helper script (already in repo)

You can use:

```bash
source mab_ros2_control/scripts/maintenance_fast_test.sh
env_setup
controller_status

# Switch controller mode
shift_controller pos
shift_controller vel
shift_controller traj

# Send commands
send_cmd pos  0.10 0.00 -0.10
send_cmd vel  0.20 0.00  0.00
send_cmd traj 0.20 0.10 -0.10
```

## 7) Useful checks when debugging

```bash
# Confirm command topics exist
ros2 topic list | grep joint_group_.*controller
ros2 topic list | grep joint_trajectory_controller

# Watch joint states
ros2 topic echo /joint_states
```

If a command does not move the robot, check that the matching command controller is `active` in:

```bash
ros2 control list_controllers -c /controller_manager
```

## 8) Realtime setup on the Pi

Use the helper script:

```bash
mab_ros2_control/scripts/setup_realtime_pi.sh status
sudo mab_ros2_control/scripts/setup_realtime_pi.sh apply
```

More detail is in `docs/realtime_pi.md`.
