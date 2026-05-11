"""Full system bringup launch file.

Starts:
  - robot_state_publisher  (required by CTC-KDL and admittance controller TF init)
  - admittance_controller (Admittance_controller)
  - robotiq_ft_sensor_hardware  (ft_sensor_standalone)
  - ft300_ros2 ft300_trajectory_calibration_node
  - IK orientation_ik_3r_node
  - wrist_games (wrist_games.launch.py)
  - foxglove_bridge
  - computed_torque_controller controller_node_kdl

NOTE: If mab_three_axis_bringup.launch.py is already running (which also starts
robot_state_publisher), set the launch argument 'start_rsp:=false' to skip it here.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── 0. robot_state_publisher (provides /robot_description + TF) ───────
    #       The CTC KDL node MUST receive /robot_description to build its
    #       KDL chain. The admittance controller also needs TF to initialise
    #       its reference orientation. Skip with start_rsp:=false if your
    #       hardware bringup already provides robot_state_publisher.
    start_rsp = LaunchConfiguration("start_rsp")

    urdf_share = get_package_share_directory("complete_system_urdf_description")
    xacro_file = os.path.join(urdf_share, "urdf", "complete_system_urdf.xacro")
    robot_urdf = xacro.process_file(xacro_file).toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_urdf}],
        condition=IfCondition(start_rsp),
    )



    # ── 1. Admittance controller ───────────────────────────────────────────
    admittance_controller_node = Node(
        package="Admittance_controller",
        executable="admittance_controller",
        name="admittance_controller",
        output="screen",
    )

    # ── 2. Robotiq FT sensor standalone ───────────────────────────────────
    ft_sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robotiq_ft_sensor_hardware"),
                "launch",
                "ft_sensor_standalone.launch.py",
            )
        )
    )

    # ── 3. FT300 trajectory calibration node ──────────────────────────────
    ft300_calibration_node = Node(
        package="ft300_ros2",
        executable="ft300_trajectory_calibration_node",
        name="ft300_trajectory_calibration_node",
        output="screen",
    )

    # ── 4. IK orientation node ─────────────────────────────────────────────
    orientation_ik_node = Node(
        package="IK",
        executable="orientation_ik_3r_node",
        name="orientation_ik_3r_node",
        output="screen",
    )

    # ── 5. Wrist games ────────────────────────────────────────────────────
    wrist_games_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("wrist_games"),
                "launch",
                "wrist_games.launch.py",
            )
        )
    )

    # ── 6. Foxglove bridge ────────────────────────────────────────────────
    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            )
        )
    )

    # ── 7. Computed torque controller (KDL) ───────────────────────────────
    ctc_params_file = os.path.join(
        get_package_share_directory("computed_torque_controller"),
        "config",
        "controller_kdl.yaml",
    )

    # Node name must match the YAML namespace key "computed_torque_controller_kdl:"
    # so we do NOT override it with name=. The default from super().__init__() is
    # "computed_torque_controller_kdl", which matches the YAML top-level key.
    computed_torque_controller_node = Node(
        package="computed_torque_controller",
        executable="controller_node_kdl",
        output="screen",
        parameters=[
            ctc_params_file,
            {"transparent_mode": False},
        ],
   )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_rsp",
            default_value="true",
            description=(
                "Launch robot_state_publisher here. "
                "Set false if mab_three_axis_bringup already provides it."
            ),
        ),
        robot_state_publisher_node,
        admittance_controller_node,
        ft_sensor_launch,
        ft300_calibration_node,
        orientation_ik_node,
        wrist_games_launch,
        foxglove_bridge_launch,
        computed_torque_controller_node,
    ])
