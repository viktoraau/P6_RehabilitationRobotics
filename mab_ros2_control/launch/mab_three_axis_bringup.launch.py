from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET


def _build_launch_nodes(context, controllers_file, installed_share):
    use_xacro = LaunchConfiguration("use_xacro").perform(context).lower() in (
        "true",
        "1",
        "yes",
        "on",
    )

    if use_xacro:
        robot_description = {
            "robot_description": Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    installed_share + "/urdf/mab_three_axis.urdf.xacro",
                    " ",
                    "bus:=",
                    LaunchConfiguration("bus"),
                    " ",
                    "data_rate:=",
                    LaunchConfiguration("data_rate"),
                    " ",
                    "use_pds:=",
                    LaunchConfiguration("use_pds"),
                    " ",
                    "use_regular_can_frames:=",
                    LaunchConfiguration("use_regular_can_frames"),
                    " ",
                    "pds_id:=",
                    LaunchConfiguration("pds_id"),
                    " ",
                    "power_stage_socket:=",
                    LaunchConfiguration("power_stage_socket"),
                ]
            )
        }
    else:
        urdf_path = installed_share + "/urdf/mab_three_axis.default.urdf"
        urdf_tree = ET.parse(urdf_path)
        urdf_root = urdf_tree.getroot()
        hardware = urdf_root.find("./ros2_control[@name='mab_hardware']/hardware")
        overrides = {
            "bus": LaunchConfiguration("bus").perform(context),
            "data_rate": LaunchConfiguration("data_rate").perform(context),
            "use_pds": LaunchConfiguration("use_pds").perform(context),
            "use_regular_can_frames": LaunchConfiguration("use_regular_can_frames").perform(context),
            "pds_id": LaunchConfiguration("pds_id").perform(context),
            "power_stage_socket": LaunchConfiguration("power_stage_socket").perform(context),
        }

        if hardware is not None:
            for param_name, value in overrides.items():
                param = hardware.find(f"./param[@name='{param_name}']")
                if param is not None:
                    param.text = value

        robot_description = {"robot_description": ET.tostring(urdf_root, encoding="unicode")}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_group_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_group_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    joint_group_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_group_velocity_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    return [
        robot_state_publisher,
        control_node,
        joint_state_broadcaster,
        joint_trajectory_controller,
        joint_group_position_controller,
        joint_group_velocity_controller,
    ]


def generate_launch_description():
    package_share = FindPackageShare("mab_ros2_control")
    controllers_file = PathJoinSubstitution(
        [package_share, "config", "mab_three_axis.controllers.yaml"]
    )
    installed_share = get_package_share_directory("mab_ros2_control")

    return LaunchDescription(
        [
            DeclareLaunchArgument("bus", default_value="USB"),
            DeclareLaunchArgument("data_rate", default_value="1M"),
            DeclareLaunchArgument("use_pds", default_value="true"),
            DeclareLaunchArgument("use_regular_can_frames", default_value="false"),
            DeclareLaunchArgument("pds_id", default_value="100"),
            DeclareLaunchArgument("power_stage_socket", default_value="2"),
            DeclareLaunchArgument("use_xacro", default_value="false"),
            OpaqueFunction(
                function=lambda context: _build_launch_nodes(
                    context, controllers_file, installed_share
                )
            ),
        ]
    )
