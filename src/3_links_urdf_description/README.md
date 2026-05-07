# 3_links_urdf_description — Legacy URDF / Gazebo Description

> **Superseded.** The active robot description used in bring-up is `complete_system_urdf_description` (loaded by `mab_rehab`).
> This package is kept for reference but is not used in the current system.

---

## Contents

- `urdf/` — older URDF/xacro files with link names `Body1__1__1`, `Body6__2__1`, `Body2__3__1` (legacy naming)
- `launch/display.launch.py` — view the URDF in Rviz
- `launch/gazebo.launch.py` — spawn the robot in Gazebo
- `config/ros_gz_bridge_gazebo.yaml` — ROS↔Gazebo topic bridge configuration

---

## Launch

```bash
# Visualise in Rviz
ros2 launch 3_links_urdf_description display.launch.py

# Simulate in Gazebo
ros2 launch 3_links_urdf_description gazebo.launch.py
```

---

## Note on Link Names

This description uses the old naming convention:
```
base_link → Body1__1__1 → Body6__2__1 → Body2__3__1
```

The current system uses:
```
base_link → SP_1 → FE_1 → RU_1
```

If you see errors like `Link 'Body2__3__1' has no parent joint`, a node is using the old name.
Check the `tip_link` parameter (e.g., in `IK` or `computed_torque_controller`).
