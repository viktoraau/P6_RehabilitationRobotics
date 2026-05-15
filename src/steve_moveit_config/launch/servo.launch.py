"""
MoveIt Servo bringup for real-time Cartesian / joint jog control.

Requires the base simulation (or hardware) to already be running:
  ros2 launch steve_moveit_config moveit_sim.launch.py

Then launch this separately:
  ros2 launch steve_moveit_config servo.launch.py
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

    srdf_path = os.path.join(moveit_share, "config", "complete_system_urdf.srdf")
    with open(srdf_path, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    servo_yaml_path = os.path.join(moveit_share, "config", "servo.yaml")
    with open(servo_yaml_path, "r") as f:
        servo_params = {"moveit_servo": yaml.safe_load(f)}

    kinematics_path = os.path.join(moveit_share, "config", "kinematics.yaml")
    with open(kinematics_path, "r") as f:
        kinematics_yaml = yaml.safe_load(f)

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
        ],
        output="screen",
    )

    return [servo_node]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=_build_nodes),
        ]
    )
