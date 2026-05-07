from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── Arguments ─────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "data_dir",
            default_value="~/wrist_games_data",
            description="Directory where patient ROM JSON files are stored",
        ),
        DeclareLaunchArgument(
            "joint_topic",
            default_value="/joint_states",
            description="ROS 2 topic publishing sensor_msgs/JointState",
        ),
        DeclareLaunchArgument(
            "demo_mode",
            default_value="false",
            description="Use keyboard instead of robot (true/false)",
        ),
        DeclareLaunchArgument(
            "num_targets",
            default_value="8",
            description="Number of reach targets per game round",
        ),

        # ── Nodes ─────────────────────────────────────────────────────────────
        Node(
            package="wrist_games",
            executable="calibration_node",
            name="calibration_node",
            output="screen",
            parameters=[{
                "data_dir":    LaunchConfiguration("data_dir"),
                "joint_topic": LaunchConfiguration("joint_topic"),
                "demo_mode":   LaunchConfiguration("demo_mode"),
            }],
        ),
        Node(
            package="wrist_games",
            executable="game_node",
            name="game_node",
            output="screen",
            parameters=[{
                "data_dir":    LaunchConfiguration("data_dir"),
                "joint_topic": LaunchConfiguration("joint_topic"),
                "demo_mode":   LaunchConfiguration("demo_mode"),
                "num_targets": LaunchConfiguration("num_targets"),
            }],
        ),
        Node(
            package="wrist_games",
            executable="airplane_node",
            name="airplane_node",
            output="screen",
            parameters=[{
                "data_dir":    LaunchConfiguration("data_dir"),
                "joint_topic": LaunchConfiguration("joint_topic"),
                "demo_mode":   LaunchConfiguration("demo_mode"),
            }],
        ),
    ])


