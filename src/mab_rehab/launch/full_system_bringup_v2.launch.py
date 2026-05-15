"""Full system bringup v2 — Part 2 (high-level stack).

This launch intentionally starts only Part 2.

Part 1 (launch separately first):
    PDS + motors + ros2_control controllers + robot_description publisher

Part 2 (this file):
    ros2 launch mab_rehab full_system_bringup_v2.launch.py

Startup order
-------------
  Phase 1 (after hardware_ready_delay_s, default 12 s):
      joint_space_admittance  — waits internally for /joint_states anyway,
                                  but the delay avoids noisy startup warnings.
      computed_torque_controller (KDL) — expects /robot_description from Part 1.
      ft300_trajectory_calibration_node
      wrist_games

  Phase 2 (always):
      robotiq_ft_sensor_hardware  — no dependency on ros2_control, starts immediately
      foxglove_bridge             — commented out by default

Notes
-----
    - This file does not launch ros2_control or robot_state_publisher.
    - Ensure Part 1 is already running before launching this file.
    - admittance_controller + IK + admittance_to_servo_bridge replaced by
        joint_space_admittance (wrench_torque_scale applied in base frame).
    - No MoveIt Servo required for the control path.
    - CTC pointed at /bridge_desired_trajectory.

Launch arguments
----------------
  Extra args added here:
    hardware_ready_delay_s  (default 12.0) — seconds to wait before starting
                                                        the high-level nodes, allowing Part 1 topics
                                                        (/joint_states, /robot_description, TF) to settle.
    use_rviz                (default false) — launch MoveIt RViz for visualisation.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Shared launch configurations ──────────────────────────────────────
    use_rviz             = LaunchConfiguration("use_rviz")
    hardware_ready_delay = LaunchConfiguration("hardware_ready_delay_s")

    # ── Phase 1: high-level nodes (delayed) ───────────────────────────────
    #    Delayed by hardware_ready_delay_s so /joint_states and the TF tree
    #    are available before these nodes start.

    # 1a. Joint-space admittance controller
    joint_space_admittance_node = Node(
        package="Admittance_controller",
        executable="joint_space_admittance",
        name="joint_space_admittance_controller",
        output="screen",
        parameters=[{
            "input_topic":              "/ft300/wrench",
            "joint_state_topic":        "/joint_states",
            "output_topic":             "/bridge_desired_trajectory",
            "base_frame":               "base_link",
            "default_wrench_frame":     "tool0",
            # Applied in base frame after TF rotation (the v1 frame-mismatch fix).
            "wrench_force_scale":       [-1.0, -1.0, 1.0],
            "wrench_torque_scale":      [-1.0, -1.0, 1.0],
            "inertia":                  [0.02, 0.02, 0.02],
            "stiffness":                [1.0, 1.0, 1.0],
            "damping_gain":             2.0,
            "torque_deadband_nm":       [0.05, 0.05, 0.05],
            "force_deadband_n":         [0.3, 0.3, 0.3],
            "force_to_torque_gain":     1.0,
            "torque_lowpass_cutoff_hz": 20.0,
            "max_joint_velocity":       [5.5, 5.5, 5.0],
            "max_joint_acceleration":   [3.5, 3.5, 3.5],
            "max_joint_error_rad":      [1.2, 1.2, 0.8],
            "control_rate_hz":          100.0,
            "wrench_timeout_s":         0.1,
            "transparent_mode":         False,
            "admittance_enabled_on_startup": False,
        }],
    )

    # 1b. Computed torque controller (KDL)
    ctc_params_file = os.path.join(
        get_package_share_directory("computed_torque_controller"),
        "config",
        "controller_kdl.yaml",
    )
    computed_torque_controller_node = Node(
        package="computed_torque_controller",
        executable="controller_node_kdl",
        output="screen",
        parameters=[
            ctc_params_file,
            {
                "desired_trajectory_topic": "/bridge_desired_trajectory",
                "transparent_mode":         False,
            },
        ],
    )

    # 1c. FT300 gravity-compensation calibration
    ft300_calibration_node = Node(
        package="ft300_ros2",
        executable="ft300_trajectory_calibration_node",
        name="ft300_trajectory_calibration_node",
        output="screen",
        parameters=[{
            "trajectory_mode":  "topic",
            "trajectory_topic": "joint_trajectory_controller/joint_trajectory",
        }],
    )

    # 1d. Wrist games
    wrist_games_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("wrist_games"),
                "launch",
                "wrist_games.launch.py",
            )
        )
    )

    delayed_high_level_nodes = TimerAction(
        period=hardware_ready_delay,
        actions=[
            joint_space_admittance_node,
            computed_torque_controller_node,
            ft300_calibration_node,
            wrist_games_launch,
        ],
    )

    # ── Phase 2: peripherals (immediate, no ros2_control dependency) ──────

    ft_sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robotiq_ft_sensor_hardware"),
                "launch",
                "ft_sensor_standalone.launch.py",
            )
        )
    )

    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            )
        )
    )

    # MoveIt RViz — visualisation only, not needed for control
    moveit_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steve_moveit_config"),
                "launch",
                "moveit_hardware.launch.py",
            )
        ),
        launch_arguments={"use_rviz": use_rviz}.items(),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        # ── Extra args ────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "hardware_ready_delay_s",
            default_value="12.0",
            description=(
                "Seconds to wait after launch before starting high-level nodes. "
                "Covers ros2_control startup + controller spawning (~8-9 s). "
                "Increase if controllers are still loading when the admittance "
                "node starts."
            ),
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch MoveIt RViz for visualisation (not required for control).",
        ),

        # ── Phase 1 (delayed) ─────────────────────────────────────────────
        delayed_high_level_nodes,

        # ── Phase 2 (immediate, no dependency) ────────────────────────────
        ft_sensor_launch,
        foxglove_bridge_launch,
        moveit_hardware_launch,
    ])
