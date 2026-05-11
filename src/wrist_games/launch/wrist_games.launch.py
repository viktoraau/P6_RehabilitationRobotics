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
        # ── Game manager arguments ─────────────────────────────────────────────
        DeclareLaunchArgument(
            "joint_names",
            default_value="",
            description="Comma-separated joint names (e.g. 'ps,fe,rud'). Empty = use indices.",
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
            description="Joint index for 1-D games (catcher). Default 2 (x/RUD).",
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
        Node(
            package="wrist_games",
            executable="wrist-game-manager",
            name="wrist_game_manager",
            output="screen",
            parameters=[{
                "ros_topic":           LaunchConfiguration("joint_topic"),
                "joint_names":         LaunchConfiguration("joint_names"),
                "joint_yaw_index":     LaunchConfiguration("joint_yaw_index"),
                "joint_v_index":       LaunchConfiguration("joint_v_index"),
                "joint_h_index":       LaunchConfiguration("joint_h_index"),
                "control_joint_index": LaunchConfiguration("control_joint_index"),
                "control_gain":        LaunchConfiguration("control_gain"),
                "start_lives":         LaunchConfiguration("start_lives"),
                "points_per_catch":    LaunchConfiguration("points_per_catch"),
            }],
        ),
    ])


