# IK — Inverse Kinematics Solver (C++)

C++ ROS 2 node that converts a desired end-effector orientation into joint trajectory commands for the 3-DOF robot arm.

---

## Node: `orientation_ik_3r_node`

Takes a desired orientation (from the admittance controller or TF) and solves the inverse kinematics using a **damped least-squares Jacobian** (Levenberg–Marquardt style).

### Input Modes

**`tf` mode** (default):
- Subscribes to `/tf` and `/tf_static`
- Computes the desired joint angles from the transform between `base_frame` and `tip_frame`

**`direct` mode:**
- Subscribes to `/desired_orientation` (`QuaternionStamped`)
- Subscribes to `/desired_angular_velocity` (`Vector3Stamped`)
- Subscribes to `/desired_angular_acceleration` (`Vector3Stamped`)
- These are the outputs of `Admittance_controller`

### Output

- Publishes `JointTrajectory` on `/joint_trajectory_controller/joint_trajectory`

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `input_mode` | `tf` | `tf` or `direct` |
| `base_frame` | `base_link` | Root frame |
| `tip_frame` | `Body2__3__1` | End-effector frame *(should be updated to `RU_1`)* |
| `publish_topic` | `/joint_trajectory_controller/joint_trajectory` | Output topic |
| `damping_lambda` | `1e-4` | Damping factor for singularity avoidance |
| `singularity_det_threshold` | `1e-6` | Det threshold to detect near-singularity |
| `max_direct_input_age_s` | `0.1` | Reject stale direct-mode inputs older than this |

---

## Build

This is a C++ (ament_cmake) package, built with:

```bash
colcon build --packages-select IK
```

Depends on: `rclcpp`, `tf2_msgs`, `geometry_msgs`, `trajectory_msgs`, `Eigen3`.

---

## Running

```bash
# Direct mode (with admittance controller)
ros2 run IK orientation_ik_3r_node --ros-args -p input_mode:=direct -p tip_frame:=RU_1

# TF mode
ros2 run IK orientation_ik_3r_node --ros-args -p input_mode:=tf -p tip_frame:=RU_1
```
