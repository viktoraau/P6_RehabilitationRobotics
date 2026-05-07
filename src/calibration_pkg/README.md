# calibration_pkg — Gear Ratio and Motor Calibration

Utilities for calibrating the gear ratios and motor parameters of the 3-joint rehabilitation robot.

---

## Why Calibration Is Needed

Position is measured on the motor shaft. The gear ratio converts motor rotations to joint angle.
If the gear ratio stored on the drive is wrong, commanded angles will be off.
This package drives each joint to its mechanical end stops to measure the actual gear ratio from real hardware travel.

---

## Scripts

### `gear_calibration` — Gear Ratio from End Stops

Drives each joint to both mechanical end stops (detected by high torque + near-zero velocity = stall), measures the reported travel, then back-calculates the true gear ratio and writes it to the motor `.cfg` file.

**Physical joint ranges used as ground truth:**

| Joint | Range |
|---|---|
| `joint_1` | 130° total (±65°) |
| `joint_2` | 60° total (±30°) |
| `joint_3` | 120° total (±60°) |

**Calibration order:** `joint_3 → joint_2 → joint_1`

**Requires:**
- `joint_group_velocity_controller` to be active
- `/dynamic_joint_states` published (from `joint_state_broadcaster`)

```bash
# Make sure the velocity controller is active first
ros2 control set_controller_state joint_group_velocity_controller active

ros2 run calibration_pkg gear_calibration
```

The script will move each joint slowly, detect the end stops, and print the measured gear ratio.
Confirm before it writes to the drive configuration.

---

### `motor_cal` — Motor Parameter Calibration

Lower-level calibration of motor electrical parameters (resistance, inductance, etc.).
Run this when setting up a new drive or after motor replacement.

```bash
ros2 run calibration_pkg motor_cal
```
