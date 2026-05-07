# ft300_ros2 — Robotiq FT300 Force/Torque Sensor Driver

ROS 2 driver for the **Robotiq FT300** 6-axis force/torque sensor connected via USB serial (`/dev/ttyUSB0`).

---

## Nodes

### `ft300_wrench_node` — Main Sensor Node

Reads raw force/torque data from the sensor, applies filtering, and publishes as `WrenchStamped`.

```bash
ros2 run ft300_ros2 ft300_wrench_node --ros-args -p port:=/dev/ttyUSB0
```

**Published topic:** `/wrench_input` (`geometry_msgs/WrenchStamped`)

**Filtering pipeline:**
1. Median window filter (default window = 7 samples)
2. Exponential moving average low-pass (separate α for force and torque)
3. Force and torque deadband (values below threshold are zeroed)
4. Optional bias subtraction from calibration file

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Serial port |
| `baudrate` | `19200` | Serial baud rate |
| `topic_name` | `/wrench_input` | Output topic |
| `frame_id` | `tool0` | TF frame of the sensor |
| `median_window` | `7` | Samples for median filter |
| `alpha_force` | `0.3` | Low-pass smoothing for force (0=heavy, 1=none) |
| `alpha_torque` | `0.3` | Low-pass smoothing for torque |
| `force_deadband` | `0.1` | Force deadband [N] |
| `torque_deadband` | `0.003` | Torque deadband [N·m] |
| `calibration_file` | `""` | Path to bias calibration JSON (optional) |

Provides a `/zero_wrench` service (`std_srvs/Trigger`) to zero the current reading as the bias offset.

---

### `ft300_calibration_node` — Static Calibration

Records steady-state readings at known poses. Run this with the robot stationary to build a bias calibration file.

```bash
ros2 run ft300_ros2 ft300_calibration_node
```

---

### `ft300_trajectory_calibration_node` — In-Motion Calibration

Records wrench data as the robot executes a trajectory. Used to identify dynamic sensor offsets.

```bash
ros2 run ft300_ros2 ft300_trajectory_calibration_node
```

---

## Notes

- The FT300 communicates at 19200 baud by default
- If `permission denied` on `/dev/ttyUSB0`, add your user to the `dialout` group:
  ```bash
  sudo usermod -aG dialout $USER
  ```
