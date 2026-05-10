# Admittance_controller — Rotational Admittance Controller

Converts forces and torques measured by the FT300 sensor into a desired end-effector orientation, angular velocity, and angular acceleration.
This is the "user intent" layer — it makes the robot comply with forces applied by the patient.

---

## Concept

Models the end-effector as a rotational mass-spring-damper about a reference orientation:

$$I\ddot{\theta} + D\dot{\theta} + K\theta = \tau$$

- $\theta$ — small-angle orientation deviation from reference
- $\tau$ — net torque from the wrench sensor (transformed to base frame)
- $I, D, K$ — inertia, damping, stiffness matrices (diagonal, per axis)

The desired absolute orientation is:

$$R_{des} = R_{ref} \cdot \text{Exp}(\theta)$$

---

## Nodes

### `admittance_controller_node`

- **Subscribes:** `WrenchStamped` on `/wrench_input` (from `ft300_ros2`)
- **Publishes:**
  - `/desired_orientation` — `QuaternionStamped`
  - `/desired_angular_velocity` — `Vector3Stamped`
  - `/desired_angular_acceleration` — `Vector3Stamped`
- Transforms the wrench into the base frame using TF2
- Applies a torque deadband and low-pass filter to the wrench
- Forces are converted to an equivalent torque via a cross-product lever arm: `τ_extra = gain * (r × F)`
- Clamps output to configurable max velocity/acceleration/orientation-error limits

```bash
ros2 run Admittance_controller admittance_controller_node
```

### `Command_pub`
Publishes test/manual orientation commands for debugging without the sensor.

### `traj_to_joint_state`
Converts a `JointTrajectory` message to a `JointState` for bridging/debugging.

---

## Key Parameters

| Parameter | Default | Description |
|---|---|---|
| `input_topic` | `/wrench_input` | WrenchStamped input |
| `base_frame` | `base_link` | Root TF frame |
| `inertia` | `[0.03, 0.03, 0.03]` | Virtual inertia [kg·m²] per axis |
| `damping` | `[0.08, 0.08, 0.08]` | Damping [N·m·s/rad] per axis |
| `stiffness` | `[0.25, 0.25, 0.25]` | Stiffness [N·m/rad] per axis |
| `torque_deadband_nm` | `[0.01, 0.01, 0.01]` | Ignore torques below this |
| `torque_lowpass_cutoff_hz` | `20.0` | Low-pass filter cutoff |
| `moment_arm` | `[0.0, 0.0, 0.0]` | Offset from sensor origin to grip point in sensor frame [m]; used as lever arm for `r × F` |
| `force_to_torque_gain` | `1.0` | Scalar to scale the `r × F` torque contribution; tune down toward 0 if too strong |
| `force_deadband_n` | `[0.0, 0.0, 0.0]` | Forces below this threshold (per axis, sensor frame) are zeroed before computing `r × F` |
| `max_angular_velocity` | `[1.0, 1.0, 1.0]` | Clamping [rad/s] |
| `max_orientation_error_rad` | `[0.5, 1.2, 1.0]` | Max allowed deviation from reference |
| `initialize_reference_from_tf` | `false` | Use current TF pose as reference on startup |
| `reference_tip_frame` | `tool0` | End-effector TF frame (fixed 14 cm above `RU_1`) |
