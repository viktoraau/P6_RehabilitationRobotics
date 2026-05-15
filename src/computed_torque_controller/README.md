# computed_torque_controller — Dynamics-Based Torque Controller

Subscribes to joint states and a desired trajectory, computes the required joint torques using the full robot dynamics model, and publishes effort commands.

---

## Control Law

$$\tau = M(q)\,\ddot{q}_{ref} + C(q,\dot{q})\dot{q} + G(q) + B\dot{q} + F_c \tanh(\alpha\dot{q})$$

where the reference acceleration is:

$$\ddot{q}_{ref} = \ddot{q}_{des} + K_d\dot{e} + K_p e$$

- $M$ — inertia matrix
- $C\dot{q}$ — Coriolis + centripetal bias
- $G$ — gravity torques
- $B$ — viscous friction coefficients (diagonal)
- $F_c, \alpha$ — Coulomb friction amplitude and tanh scale

---

## Two Implementations

### `controller_node` (symbolic, original)
Loads pre-computed symbolic expressions for M, C, G from `.txt` files in `dynamic_matrices/`.
Expressions are evaluated numerically at each step using `eval()` with `sin`, `cos`, `tanh`.

```bash
ros2 run computed_torque_controller controller_node
```

### `controller_node_kdl` (KDL, recommended)
Computes M, C, G online from the URDF using **PyKDL** (`python3-pykdl`).
Reads the robot URDF from `/robot_description` (published by `robot_state_publisher`).
All tunable parameters live in a YAML config file.

```bash
ros2 run computed_torque_controller controller_node_kdl \
  --ros-args --params-file install/computed_torque_controller/share/computed_torque_controller/config/controller_kdl.yaml
```

> `robot_state_publisher` must be running before this node starts.

---

## Config — `config/controller_kdl.yaml`

Edit this file before running the KDL controller:

```yaml
computed_torque_controller_kdl:
  ros__parameters:
    base_link: base_link
    tip_link:  RU_1

    joint_names: [joint_1, joint_2, joint_3]

    # Control gains — SET THESE before using
    kp: [0.0, 0.0, 0.0]   # proportional
    kd: [0.0, 0.0, 0.0]   # derivative

    # Friction model
    viscous_friction:  [0.0, 0.0, 0.0]   # B_i
    coulomb_friction:  [0.0, 0.0, 0.0]   # Fc_i
    coulomb_tanh_scale: 10.0             # α

    # Motor inertia (reflected to joint side)
    motor_inertia: [0.0, 0.0, 0.0]
    use_motor_inertia: true

    gravity: [0.0, 0.0, 9.81]
    control_rate_hz: 100.0
```

After editing, use the **installed** path (or the source path) with `--params-file`, not just the filename:

```bash
--params-file src/computed_torque_controller/config/controller_kdl.yaml
# or after build:
--params-file install/computed_torque_controller/share/computed_torque_controller/config/controller_kdl.yaml
```

---

## Topics

| Topic | Direction | Type |
|---|---|---|
| `/joint_states` | subscribe | `sensor_msgs/JointState` |
| `/joint_trajectory_controller/joint_trajectory` | subscribe | `trajectory_msgs/JointTrajectory` |
| `/joint_group_effort_controller/commands` | publish | `std_msgs/Float64MultiArray` |
| `/robot_description` | subscribe (latched) | `std_msgs/String` *(KDL node only)* |


ros2 run computed_torque_controller controller_node_kdl \
  --ros-args --params-file /home/a/wrist_games_data/pr/P6_RehabilitationRobotics/src/computed_torque_controller/config/controller_kdl.yaml \
  -p transparent_mode:=true


ros2 topic pub /bridge_desired_trajectory trajectory_msgs/msg/JointTrajectory "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  joint_names: ['joint_1', 'joint_2', 'joint_3'],
  points: [
    {
      positions: [0.0, 0.0, 0.0],
      velocities: [0.0, 0.0, 0.0],
      accelerations: [0.0, 0.0, 0.0],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}" 
