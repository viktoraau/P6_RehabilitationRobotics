from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port",            default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("frame_id",        default_value="ft_sensor"),
        DeclareLaunchArgument("auto_tare",       default_value="false"),

        Node(
            package="ft_six_axis",
            executable="ft_sensor_node",
            name="ft_sensor",
            output="screen",
            parameters=[{
                "port":            LaunchConfiguration("port"),
                "baud_continuous": 460800,
                "baud_modbus":     460800,
                "frame_id":        LaunchConfiguration("frame_id"),
                "force_scale":     0.001,   # raw int32 / 1000 → N  (100 N range, 3 dp)
                "torque_scale":    0.001,   # raw int32 / 1000 → Nm (10 Nm range, 3 dp)
                "modbus_slaves":   [1, 2, 3, 4, 5, 6],
                "auto_tare":       LaunchConfiguration("auto_tare"),
            }],
        ),
    ])
