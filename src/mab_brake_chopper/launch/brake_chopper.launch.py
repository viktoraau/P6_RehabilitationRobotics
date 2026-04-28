from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("pds_id", default_value="100"),
            DeclareLaunchArgument("voltage_source", default_value="dynamic_joint_state"),
            DeclareLaunchArgument("trigger_voltage_v", default_value="12.0"),
            DeclareLaunchArgument("telemetry_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument("voltage_scale", default_value="0.001"),
            DeclareLaunchArgument("gpio_pin", default_value="17"),
            DeclareLaunchArgument("gpio_backend", default_value="linux_char"),
            DeclareLaunchArgument("gpio_chip_label", default_value="pinctrl-rp1"),
            DeclareLaunchArgument("gpio_chip_path", default_value=""),
            DeclareLaunchArgument("dynamic_joint_states_topic", default_value="/dynamic_joint_states"),
            DeclareLaunchArgument("dynamic_joint_name", default_value="mab_power_stage"),
            DeclareLaunchArgument("dynamic_interface_name", default_value="bus_voltage"),
            DeclareLaunchArgument("control_module_topic", default_value=""),
            Node(
                package="mab_brake_chopper",
                executable="brake_chopper_node",
                name="brake_chopper",
                output="screen",
                parameters=[
                    {
                        "pds_id": LaunchConfiguration("pds_id"),
                        "voltage_source": LaunchConfiguration("voltage_source"),
                        "trigger_voltage_v": LaunchConfiguration("trigger_voltage_v"),
                        "telemetry_timeout_sec": LaunchConfiguration("telemetry_timeout_sec"),
                        "voltage_scale": LaunchConfiguration("voltage_scale"),
                        "gpio_pin": LaunchConfiguration("gpio_pin"),
                        "gpio_backend": LaunchConfiguration("gpio_backend"),
                        "gpio_chip_label": LaunchConfiguration("gpio_chip_label"),
                        "gpio_chip_path": LaunchConfiguration("gpio_chip_path"),
                        "dynamic_joint_states_topic": LaunchConfiguration(
                            "dynamic_joint_states_topic"
                        ),
                        "dynamic_joint_name": LaunchConfiguration("dynamic_joint_name"),
                        "dynamic_interface_name": LaunchConfiguration(
                            "dynamic_interface_name"
                        ),
                        "control_module_topic": LaunchConfiguration("control_module_topic"),
                    }
                ],
            ),
        ]
    )
