# Adapting the Simulation to the Real CAD and URDF

This guide explains how to refine or replace the current CAD-exported robot
description while keeping the RViz and fake-controller workflow working.

## Goal

Update the simulation so it still launches in RViz and still accepts:

- `/joint_trajectory_controller/joint_trajectory`
- `/joint_group_position_controller/commands`
- `/joint_group_velocity_controller/commands`

The safest path is to keep the same joint names and base frame unless there is a
strong reason to change them.

## Files to update

Main robot-description files:

- `Steve_robot/urdf/complete_system_urdf.xacro`
- `Steve_robot/urdf/complete_system_urdf.ros2control.xml`

Main simulation wrapper files:

- `ros2_ws/src/mab_three_axis_sim/config/mab_three_axis_sim.controllers.yaml`
- `ros2_ws/src/mab_three_axis_sim/launch/mab_three_axis_sim.launch.py`
- `ros2_ws/src/mab_three_axis_sim/rviz/mab_three_axis_sim.rviz`

If joint names change, also review:

- `ros2_ws/src/computed_torque_controller/computed_torque_controller/controller_node.py`
- command examples in `README.md`

## Information to collect first

Before editing the robot description, gather these values from CAD or the final robot URDF:

- Link lengths and joint-to-joint offsets
- Joint axes for all 3 joints
- Joint origins as `xyz` and `rpy`
- Joint limits:
  - lower
  - upper
  - max velocity
  - max effort
- Link masses
- Center of mass for each link
- Inertia tensor for each link
- Mesh files for visual and collision geometry
- Base frame definition and end-effector frame definition
- The mechanical zero position used on the real robot

Use meters, kilograms, radians, and SI inertia units throughout.

## Recommended update order

1. Update geometry and kinematics first.
2. Update inertial properties second.
3. Update meshes and visuals third.
4. Update controller config only if joint names or interfaces changed.
5. Re-test the sim bringup and command topics.

## Step 1: Update the robot geometry

Edit `Steve_robot/urdf/complete_system_urdf.xacro`.

Replace or update:

- The CAD-exported visual meshes and link frames
- The collision geometry
- The joint origins:
  - `joint_1`
  - `joint_2`
  - `joint_3`
- The joint axes
- The joint limit values

Current control-joint convention is:

- `joint_1`: axis `0 1 0`
- `joint_2`: axis `-1 0 0`
- `joint_3`: axis `0 0 1`

That matches the current CAD-exported model after the joint rename. If the
real robot changes again later, update the axes and origins to match the
mechanism and then adjust the DH model separately.

## Step 2: Add CAD meshes

If you receive updated STL, DAE, or OBJ files, add them under the description
package mesh folder in `Steve_robot/meshes`, then reference them with package
URIs, for example:

```xml
<mesh filename="package://complete_system_urdf_description/meshes/base_link.stl"/>
```

Use separate visual and collision meshes when possible. If the CAD meshes are
heavy, use simplified collision meshes to keep RViz and future simulation fast.

## Step 3: Update inertial properties

Still in `Steve_robot/urdf/complete_system_urdf.xacro`, replace the current inertias
with real values from CAD or identified data:

- `<mass value="..."/>`
- `<origin xyz="..."/>` inside `<inertial>`
- `<inertia ixx="..." ixy="..." ixz="..." iyy="..." iyz="..." izz="..."/>`

Do not leave rough CAD-export inertias once you move beyond simple RViz inspection.
Even fake hardware setups benefit from physically consistent inertial data for
future simulator migration.

## Step 4: Keep the ros2_control interface stable

The fake controller block is inside the same file:

- `Steve_robot/urdf/complete_system_urdf.ros2control.xml`

For the current workflow, keep:

- the hardware plugin as `mock_components/GenericSystem`
- command interfaces:
  - `position`
  - `velocity`
- state interfaces:
  - `position`
  - `velocity`

Only change these if you intentionally want a different control architecture.
For the current RViz testing workflow, they should stay as they are.

## Step 5: Only touch the controller YAML if names or interfaces change

Edit `config/mab_three_axis_sim.controllers.yaml` only if one of these changes:

- joint names
- command interfaces
- state interfaces
- controller tolerances

If you keep `joint_1`, `joint_2`, and `joint_3`, then this file will likely need
little or no change.

If joint names change, update them consistently in all three sections:

- `joint_trajectory_controller`
- `joint_group_position_controller`
- `joint_group_velocity_controller`

## Step 6: Update RViz settings if frames change

Edit `rviz/mab_three_axis_sim.rviz` only if needed.

Most likely changes:

- `Fixed Frame` if `base_link` is renamed
- default camera angle
- whether TF names are still meaningful

If `base_link` stays the same, RViz may need no update at all.

## Step 7: Update launch only if the file structure changes

Edit `launch/mab_three_axis_sim.launch.py` only if:

- the Xacro filename changes
- the controller YAML filename changes
- you split the real robot description into multiple files

If the description package structure changes later, keep the robot geometry in
the description package and keep the fake `ros2_control` block in
`complete_system_urdf.ros2control.xml`. That keeps the model and the sim wiring
separated cleanly.

## If the final URDF already exists

If the team receives a newer working URDF/Xacro from another source, do not
manually copy everything into the sim package unless necessary.

Preferred approach:

1. keep the real geometry in `complete_system_urdf.xacro`
2. keep the fake `ros2_control` block in `complete_system_urdf.ros2control.xml`
3. keep the RViz and controller bringup in `mab_three_axis_sim`

This avoids maintaining two different robot descriptions by hand.

## Joint-name compatibility checklist

If you want `computed_torque_controller` and existing command examples to keep
working without code changes, preserve:

- `joint_1`
- `joint_2`
- `joint_3`
- `/joint_states`
- `/joint_trajectory_controller/joint_trajectory`
- `/joint_group_position_controller/commands`
- `/joint_group_velocity_controller/commands`

If you rename joints, you must also update:

- the controller YAML
- command examples in `README.md`
- any downstream nodes assuming the first three joint entries map to the robot

## Validation after updating the model

After updating the model on Linux ROS 2 Jazzy, run:

```bash
cd ~/P6_RehabilitationRobotics
source /opt/ros/jazzy/setup.bash
colcon build --packages-select complete_system_urdf_description mab_three_axis_sim computed_torque_controller
source install/setup.bash
ros2 launch mab_three_axis_sim mab_three_axis_sim.launch.py
```

Then verify:

1. RViz opens and the robot is visible.
2. `ros2 topic echo /joint_states` shows the three joints.
3. A trajectory command moves the robot in RViz.
4. Position mode works after controller switching.
5. Velocity mode works after controller switching.
6. `ros2 control list_controllers -c /controller_manager` shows only one active command controller at a time.

## Common mistakes to avoid

- Mixing millimeters from CAD with meters in URDF
- Changing joint names in the URDF but not in the controller YAML
- Importing visual meshes without updating collision geometry
- Forgetting to update inertia after replacing link shapes
- Rotating meshes in the CAD export instead of expressing the frame change in URDF
- Renaming `base_link` without updating RViz fixed frame

## Minimum handoff package when the real model arrives

When you or your teammates have the real robot model ready, the ideal handoff is:

- final Xacro or URDF
- mesh files
- link mass and inertia values
- joint limits
- zero-position convention
- note about any changed frame names

With that, this simulation package can be updated cleanly in one pass.
