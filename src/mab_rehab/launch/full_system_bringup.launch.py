"""Full system bringup launch file.

Starts:
  - robot_state_publisher  (required by CTC-KDL and admittance controller TF init)
  - admittance_controller (Admittance_controller)
  - robotiq_ft_sensor_hardware  (ft_sensor_standalone)
  - ft300_ros2 ft300_trajectory_calibration_node
    - IK orientation_ik_3r_node (legacy path, when use_moveit:=false)
    - MoveIt + Servo + admittance bridge (when use_moveit:=true)
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
from launch.conditions import UnlessCondition
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
    use_moveit = LaunchConfiguration("use_moveit")
    use_rviz = LaunchConfiguration("use_rviz")

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
        parameters=[{
            "trajectory_mode":  "topic",
            "trajectory_topic": "joint_trajectory_controller/joint_trajectory",
        }],
    )

    # ── 4. IK orientation node ─────────────────────────────────────────────
    orientation_ik_node = Node(
        package="IK",
        executable="orientation_ik_3r_node",
        name="orientation_ik_3r_node",
        output="screen",
        condition=UnlessCondition(use_moveit),
    )

    # ── 4b. MoveIt Servo bridge path (replaces custom IK when enabled) ───
    admittance_to_servo_bridge_node = Node(
        package="Admittance_controller",
        executable="admittance_to_servo_bridge",
        name="admittance_to_servo_bridge",
        output="screen",
        parameters=[
            {
                "input_angular_velocity_topic": "/desired_angular_velocity",
                "input_orientation_topic": "/desired_orientation",
                "input_angular_acceleration_topic": "/desired_angular_acceleration",
                "joint_state_topic": "/joint_states",
                "output_mode": "joint_jog",
                "output_joint_topic": "/servo_node/delta_joint_cmds",
                "trajectory_output_topic": "/bridge_desired_trajectory",
                "command_frame": "base_link",
                "damping_lambda": 0.01,
            }
        ],
        condition=IfCondition(use_moveit),
    )

    moveit_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steve_moveit_config"),
                "launch",
                "moveit_hardware.launch.py",
            )
        ),
        launch_arguments={"use_rviz": use_rviz}.items(),
        condition=IfCondition(use_moveit),
    )

    moveit_servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steve_moveit_config"),
                "launch",
                "servo.launch.py",
            )
        ),
        condition=IfCondition(use_moveit),
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
        DeclareLaunchArgument(
            "use_moveit",
            default_value="true",
            description=(
                "Use MoveIt Servo path (admittance -> MoveIt Servo) instead of "
                "the custom IK node."
            ),
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="When use_moveit is true, also launch MoveIt RViz.",
        ),
        robot_state_publisher_node,
        admittance_controller_node,
        ft_sensor_launch,
        ft300_calibration_node,
        orientation_ik_node,
        admittance_to_servo_bridge_node,
        moveit_hardware_launch,
        moveit_servo_launch,
        wrist_games_launch,
        #foxglove_bridge_launch,
        computed_torque_controller_node,
    ])
