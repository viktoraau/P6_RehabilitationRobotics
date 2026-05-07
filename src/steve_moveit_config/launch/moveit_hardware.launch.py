"""
MoveIt 2 overlay for the real MAB hardware bringup.

Usage:
  1. First start the hardware bringup:
       ros2 launch mab_ros2_control mab_three_axis_bringup.launch.py \\
           active_controller:=trajectory enable_orientation_ik:=false

  2. Then start MoveIt on top:
       ros2 launch steve_moveit_config moveit_hardware.launch.py

This launch file starts ONLY move_group + rviz. It expects
robot_state_publisher, controller_manager, and controllers to already
be running from the hardware bringup.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _build_nodes(context):
    use_rviz = LaunchConfiguration("use_rviz")

    moveit_share = get_package_share_directory("steve_moveit_config")
    description_share = FindPackageShare("complete_system_urdf_description")

    xacro_file = PathJoinSubstitution(
        [description_share, "urdf", "complete_system_urdf.xacro"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_file]),
            value_type=str,
        )
    }

    srdf_path = os.path.join(
        moveit_share, "config", "complete_system_urdf.srdf"
    )
    with open(srdf_path, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    def _load_yaml(yaml_relpath):
        fpath = os.path.join(moveit_share, yaml_relpath)
        with open(fpath, "r") as f:
            return yaml.safe_load(f)

    kinematics_yaml = _load_yaml("config/kinematics.yaml")
    ompl_planning_yaml = _load_yaml("config/ompl_planning.yaml")
    joint_limits_yaml = _load_yaml("config/joint_limits.yaml")
    moveit_controllers_yaml = _load_yaml("config/moveit_controllers.yaml")

    # Planning pipeline configuration (separate dict, no ParameterValue mixing)
    planning_pipeline_params = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_planning_yaml,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            {"robot_description_planning": joint_limits_yaml},
            moveit_controllers_yaml,
            planning_pipeline_params,
            {"use_sim_time": False},
        ],
    )

    rviz_config = os.path.join(moveit_share, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
        ],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz with the MoveIt Motion-Planning plugin.",
            ),
            OpaqueFunction(function=_build_nodes),
        ]
    )
