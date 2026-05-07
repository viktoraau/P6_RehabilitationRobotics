"""
MoveIt 2 + ros2_control simulation bringup for the Steve 3-axis robot.

Launches:
  - robot_state_publisher  (publishes /robot_description & TF)
  - ros2_control_node      (mock hardware)
  - joint_state_broadcaster
  - joint_trajectory_controller
  - move_group             (MoveIt planning / execution)
  - rviz2                  (with MoveIt Motion-Planning plugin)
"""

import os

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
from moveit_configs_utils import MoveItConfigsBuilder


def _build_launch_nodes(context):
    use_rviz = LaunchConfiguration("use_rviz")

    # ── Package share directories ──────────────────────────────
    moveit_config_share = get_package_share_directory("steve_moveit_config")
    sim_share = get_package_share_directory("mab_three_axis_sim")
    description_share = FindPackageShare("complete_system_urdf_description")

    # ── Robot description via xacro ────────────────────────────
    xacro_file = PathJoinSubstitution(
        [description_share, "urdf", "complete_system_urdf.xacro"]
    )
    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", xacro_file]
    )
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content, value_type=str
        )
    }

    # ── Load SRDF ──────────────────────────────────────────────
    srdf_path = os.path.join(
        moveit_config_share, "config", "complete_system_urdf.srdf"
    )
    with open(srdf_path, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # ── Load MoveIt YAML configs ───────────────────────────────
    import yaml

    def _load_yaml(pkg_share, yaml_relpath):
        fpath = os.path.join(pkg_share, yaml_relpath)
        with open(fpath, "r") as f:
            return yaml.safe_load(f)

    kinematics_yaml = _load_yaml(moveit_config_share, "config/kinematics.yaml")
    ompl_planning_yaml = _load_yaml(
        moveit_config_share, "config/ompl_planning.yaml"
    )
    joint_limits_yaml = _load_yaml(
        moveit_config_share, "config/joint_limits.yaml"
    )
    moveit_controllers_yaml = _load_yaml(
        moveit_config_share, "config/moveit_controllers.yaml"
    )

    # Planning pipeline configuration (separate dict, no ParameterValue mixing)
    planning_pipeline_params = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_planning_yaml,
    }

    # ── Controllers YAML (ros2_control) ────────────────────────
    controllers_file = PathJoinSubstitution(
        [sim_share, "config", "mab_three_axis_sim.controllers.yaml"]
    )

    # ── Nodes ──────────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

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

    # RViz with MoveIt display config
    rviz_config = os.path.join(
        moveit_config_share, "config", "moveit.rviz"
    )
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

    return [
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        move_group_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz with the MoveIt Motion-Planning plugin.",
            ),
            OpaqueFunction(function=_build_launch_nodes),
        ]
    )
