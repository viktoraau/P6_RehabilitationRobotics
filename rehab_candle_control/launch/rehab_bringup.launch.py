import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_candle_backend = LaunchConfiguration("start_candle_backend")
    bus = LaunchConfiguration("bus")
    data_rate = LaunchConfiguration("data_rate")
    default_qos = LaunchConfiguration("default_qos")

    pds_id = LaunchConfiguration("pds_id")
    power_stage_socket = LaunchConfiguration("power_stage_socket")
    md_ids = LaunchConfiguration("md_ids")
    speed_step = LaunchConfiguration("speed_step")
    max_speed = LaunchConfiguration("max_speed")

    candle_ros2_launch_dir = os.path.join(
        get_package_share_directory("candle_ros2"), "launch"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_candle_backend", default_value="true"),
            DeclareLaunchArgument("bus", default_value="USB"),
            DeclareLaunchArgument("data_rate", default_value="1M"),
            DeclareLaunchArgument("default_qos", default_value="Reliable"),
            DeclareLaunchArgument("pds_id", default_value="100"),
            DeclareLaunchArgument("power_stage_socket", default_value="2"),
            DeclareLaunchArgument("md_ids", default_value="[37, 939, 941]"),
            DeclareLaunchArgument("speed_step", default_value="0.5"),
            DeclareLaunchArgument("max_speed", default_value="8.0"),
            # 1. Start the candle_ros2 backend (MD + PDS nodes)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(candle_ros2_launch_dir, "both_launch.py")
                ),
                condition=IfCondition(start_candle_backend),
                launch_arguments={
                    "bus": bus,
                    "data_rate": data_rate,
                    "default_qos": default_qos,
                }.items(),
            ),
            # 2. Start the interactive speed control node.
            #    It internally waits for all services, then calls them in order:
            #      /pds/add_pds → /pds/id_<N>/enable_ps_<M> → /md/add_mds → /md/set_mode → /md/enable
            Node(
                package="rehab_candle_control",
                executable="interactive_speed_control",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "pds_id": pds_id,
                        "power_stage_socket": power_stage_socket,
                        "md_ids": md_ids,
                        "speed_step": speed_step,
                        "max_speed": max_speed,
                    }
                ],
            ),
        ]
    )
