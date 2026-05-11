"""
game_manager.launch.py
----------------------
Primary launch file for the Wrist Games package.

Starts the GameManagerNode, which exposes one Trigger service per game:
  /wrist_games/start_airplane
  /wrist_games/start_catcher
  /wrist_games/start_jedi
  /wrist_games/start_octagon
  /wrist_games/start_pendulum
  /wrist_games/start_tunnel
  /wrist_games/start_xwing
  /wrist_games/stop_game

Joint layout expected on the JointState topic (positions[0..2]):
  index 0 – turn  (PS  – pronation / supination)
  index 1 – y     (FE  – flexion / extension)
  index 2 – x     (RUD – radial / ulnar deviation)

Alternatively supply joint_names:= with comma-separated names.

Usage:
  ros2 launch wrist_games game_manager.launch.py
  ros2 launch wrist_games game_manager.launch.py ros_topic:=/wrist/joint_states
  ros2 launch wrist_games game_manager.launch.py joint_names:=ps,fe,rud
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # ── Declare configurable launch arguments ────────────────────────────
        DeclareLaunchArgument(
            "ros_topic",
            default_value="/joint_states",
            description="JointState topic name published by the wrist device.",
        ),
        DeclareLaunchArgument(
            "joint_names",
            default_value="",
            description=(
                "Comma-separated joint names to read from the topic "
                "(e.g. 'ps,fe,rud').  Empty string = use positional indices 0/1/2."
            ),
        ),
        DeclareLaunchArgument(
            "joint_yaw_index",
            default_value="0",
            description="Index of the turn/PS joint in position[] (default 0).",
        ),
        DeclareLaunchArgument(
            "joint_v_index",
            default_value="1",
            description="Index of the y/FE joint in position[] (default 1).",
        ),
        DeclareLaunchArgument(
            "joint_h_index",
            default_value="2",
            description="Index of the x/RUD joint in position[] (default 2).",
        ),
        DeclareLaunchArgument(
            "control_joint_index",
            default_value="2",
            description="Joint index for 1-D games (catcher paddle). Default 2 (x/RUD).",
        ),
        DeclareLaunchArgument(
            "control_gain",
            default_value="1.0",
            description="Gain passed to tanh normalisation for all joint values.",
        ),
        DeclareLaunchArgument(
            "start_lives",
            default_value="3",
            description="Number of lives each game session starts with.",
        ),
        DeclareLaunchArgument(
            "points_per_catch",
            default_value="10",
            description="Score points awarded per successful in-game action.",
        ),

        # ── Game Manager Node ─────────────────────────────────────────────────
        Node(
            package="wrist_games",
            executable="wrist-game-manager",
            name="wrist_game_manager",
            parameters=[{
                "ros_topic":          LaunchConfiguration("ros_topic"),
                "joint_names":        LaunchConfiguration("joint_names"),
                "joint_yaw_index":    LaunchConfiguration("joint_yaw_index"),
                "joint_v_index":      LaunchConfiguration("joint_v_index"),
                "joint_h_index":      LaunchConfiguration("joint_h_index"),
                "control_joint_index": LaunchConfiguration("control_joint_index"),
                "control_gain":       LaunchConfiguration("control_gain"),
                "start_lives":        LaunchConfiguration("start_lives"),
                "points_per_catch":   LaunchConfiguration("points_per_catch"),
            }],
            output="screen",
        ),
    ])
