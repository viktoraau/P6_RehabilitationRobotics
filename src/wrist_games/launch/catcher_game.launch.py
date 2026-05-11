"""
catcher_game.launch.py
----------------------
Stand-alone launcher for the Catcher game.

Joint used:
  index 2 – x / RUD (radial/ulnar deviation moves paddle left/right)

Usage:
  ros2 launch wrist_games catcher_game.launch.py
  ros2 launch wrist_games catcher_game.launch.py ros_topic:=/wrist/joint_states
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("ros_topic",            default_value="/joint_states"),
        DeclareLaunchArgument("joint_names",           default_value=""),
        DeclareLaunchArgument("control_joint_index",   default_value="2"),
        DeclareLaunchArgument("control_gain",          default_value="1.0"),
        DeclareLaunchArgument("start_lives",           default_value="3"),
        DeclareLaunchArgument("points_per_catch",      default_value="10"),

        Node(
            package="wrist_games",
            executable="wrist-catcher-game",
            arguments=[
                "--ros-topic",              LaunchConfiguration("ros_topic"),
                "--joint-names",            LaunchConfiguration("joint_names"),
                "--control-joint-index",    LaunchConfiguration("control_joint_index"),
                "--control-gain",           LaunchConfiguration("control_gain"),
                "--start-lives",            LaunchConfiguration("start_lives"),
                "--points-per-catch",       LaunchConfiguration("points_per_catch"),
            ],
            output="screen",
        ),
    ])
