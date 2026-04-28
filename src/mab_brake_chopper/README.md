# mab_brake_chopper

Small ROS 2 package for switching a brake-resistor chopper on when the DC bus
voltage drops below a trigger threshold.

## What it does

- Reads bus voltage from existing ROS 2 telemetry.
- Turns `GPIO 17` on when the voltage drops below the configured trigger.
- Turns the GPIO back off once the voltage rises above the trigger.
- Forces the GPIO off if voltage telemetry stops arriving.

## Supported voltage sources

- `dynamic_joint_state` from `mab_ros2_control` via `/dynamic_joint_states`
  using `mab_power_stage/bus_voltage`
- `control_module` from `candle_ros2` via `pds/id_<id>/control_module`

Default mode is `dynamic_joint_state`, so the brake chopper follows the faster
`mab_ros2_control` telemetry path by default. You can switch to `auto` or
`control_module` if needed.

## Launch

```bash
ros2 launch mab_brake_chopper brake_chopper.launch.py \
  trigger_voltage_v:=54.0
```

Dry-run without touching GPIO:

```bash
ros2 launch mab_brake_chopper brake_chopper.launch.py \
  gpio_backend:=mock
```

## Main parameters

- `trigger_voltage_v`: enable threshold in volts
- `gpio_pin`: BCM line offset, default `17`
- `gpio_backend`: `linux_char` or `mock`
- `gpio_chip_label`: default `pinctrl-rp1` for the Pi 5 style GPIO chip
- `telemetry_timeout_sec`: force output low when telemetry becomes stale
- `voltage_source`: `auto`, `dynamic_joint_state`, or `control_module`

## Notes

- The Linux GPIO character device needs permission to access the matching
  `/dev/gpiochip*` node.
- Bus voltage from the current MAB stack is reported in millivolts, so the node
  scales incoming values by `0.001` before applying the threshold.
