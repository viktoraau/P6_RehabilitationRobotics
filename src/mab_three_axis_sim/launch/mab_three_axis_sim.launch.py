from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _controller_spawner_args(controller_name, active_controller_name):
    arguments = [controller_name, "--controller-manager", "/controller_manager"]
    if controller_name != active_controller_name:
        arguments.append("--inactive")
    return arguments


def _build_launch_nodes(context):
    active_controller = LaunchConfiguration("active_controller").perform(context).strip().lower()
    active_controller_map = {
        "trajectory": "joint_trajectory_controller",
        "position": "joint_group_position_controller",
        "velocity": "joint_group_velocity_controller",
        "none": None,
        "joint_trajectory_controller": "joint_trajectory_controller",
        "joint_group_position_controller": "joint_group_position_controller",
        "joint_group_velocity_controller": "joint_group_velocity_controller",
    }
    active_controller_name = active_controller_map.get(active_controller)
    if active_controller_name is None and active_controller != "none":
        valid_values = ", ".join(["trajectory", "position", "velocity", "none"])
        raise RuntimeError(
            f"Invalid active_controller '{active_controller}'. Expected one of: {valid_values}"
        )

    sim_package_share = FindPackageShare("mab_three_axis_sim")
    description_package_share = FindPackageShare("complete_system_urdf_description")
    xacro_file = PathJoinSubstitution(
        [description_package_share, "urdf", "complete_system_urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [sim_package_share, "config", "mab_three_axis_sim.controllers.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [sim_package_share, "rviz", "mab_three_axis_sim.rviz"]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_file]),
            value_type=str,
        )
    }

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=_controller_spawner_args(
            "joint_trajectory_controller", active_controller_name
        ),
        output="screen",
    )

    joint_group_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=_controller_spawner_args(
            "joint_group_position_controller", active_controller_name
        ),
        output="screen",
    )

    joint_group_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=_controller_spawner_args(
            "joint_group_velocity_controller", active_controller_name
        ),
        output="screen",
    )

    orientation_ik_node = Node(
        package="mab_orientation_ik",
        executable="orientation_ik_bridge",
        name="orientation_ik_bridge",
        parameters=[
            {
                "joint_names": ["joint_1", "joint_2", "joint_3"],
                "joint_lower_limits": [-1.22173, -0.523599, -1.047198],
                "joint_upper_limits": [1.22173, 0.523599, 1.047198],
                "base_frame": "base_link",
                "tool_frame": "tool0",
            }
        ],
        condition=IfCondition(LaunchConfiguration("enable_orientation_ik")),
        output="screen",
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controllers_file],
            output="screen",
        ),
        joint_state_broadcaster,
        joint_trajectory_controller,
        joint_group_position_controller,
        joint_group_velocity_controller,

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz alongside the fake controller bringup.",
            ),
            DeclareLaunchArgument(
                "active_controller",
                default_value="trajectory",
                description="Active command controller: trajectory, position, velocity, or none.",
            ),
            DeclareLaunchArgument(
                "enable_orientation_ik",
                default_value="true",
                description="Start the orientation-only IK bridge node.",
            ),
            OpaqueFunction(function=_build_launch_nodes),
        ]
    )
