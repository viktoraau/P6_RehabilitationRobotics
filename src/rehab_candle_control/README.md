# rehab_candle_control

Custom ROS 2 bringup for MAB CANdle + MD/PDS with keyboard speed control.

This package is kept for direct keyboard/demo workflows.
For the main `ros2_control` integration (hardware profiles, controller manager, PID controllers, and calibration helper nodes), use:

- `mab_ros2_control/README.md`
- `mab_ros2_control/docs/OPERATIONS_AND_MODIFICATION_GUIDE.md`

## What it does

1. Optionally starts the `candle_ros2` container (MD + PDS nodes).
2. Adds PDS device with ID 100.
3. Enables power stage module at socket 2.
4. Adds MD actuators with IDs 37, 939, 941.
5. Sets MD mode to VELOCITY_PID and enables all drives.
6. Publishes md/motion_command with target velocities controlled from keyboard.
7. Publishes telemetry topics for plotting/monitoring:
   - `rehab/telemetry/pds`: bus voltage, power stage output voltage, power stage load current
   - `rehab/telemetry/joint_<id>`: position, speed, torque, and commanded speed

## Build

Run from your ROS 2 workspace root:

```bash
colcon build --packages-select candle_ros2 rehab_candle_control
source install/setup.bash
```

## Launch

```bash
ros2 launch rehab_candle_control rehab_bringup.launch.py
```

## Keyboard buttons

- 1 / q: increase / decrease speed for MD 37 (`joint_1`, `MP-45-36`, 38:1 gearbox)
- 2 / w: increase / decrease speed for MD 939 (`joint_2`, `MP-40-10`, 10:1 gearbox)
- 3 / e: increase / decrease speed for MD 941 (`joint_3`, `MP-40-10`, 10:1 gearbox)
- Space: set all speeds to zero
- x: disable all MDs and quit

## Telemetry topics

- `md/joint_states` (from candle_ros2): position, speed, torque per motor
- `pds/id_100/control_module` (from candle_ros2): bus voltage
- `pds/id_100/power_stage_2` (from candle_ros2): output voltage, load current
- `rehab/telemetry/pds` (from this package): `[bus_voltage_v, output_voltage_v, load_current_a]`
- `rehab/telemetry/joint_<id>` (from this package): `[position, speed, torque, target_speed]`

## Optional launch overrides

```bash
ros2 launch rehab_candle_control rehab_bringup.launch.py \
  start_candle_backend:=false \
  pds_id:=100 power_stage_socket:=2 md_ids:='[37, 939, 941]' speed_step:=0.5 max_speed:=8.0
```
