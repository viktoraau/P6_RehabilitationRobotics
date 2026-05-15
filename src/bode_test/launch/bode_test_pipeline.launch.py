"""Launch the Bode test node in full-pipeline mode.

Assumes the full system is running WITHOUT the real FT sensor:
  - mab_three_axis_bringup.launch.py  (hardware + ros2_control)
  - full_system_bringup.launch.py     (RSP, CTC-KDL, IK, admittance)
    but with ft300 / robotiq sensor nodes NOT running.

The node publishes synthetic WrenchStamped on /ft300/wrench.
The admittance controller picks it up and drives the full pipeline.

To test a different joint, pass joint_index:=1 or joint_index:=2.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('bode_test')
    config = os.path.join(pkg, 'config', 'bode_test_pipeline.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('joint_index', default_value='2',
                              description='Axis to excite: 0=joint_1/FE, 1=joint_2/UD, 2=joint_3/RU'),
        DeclareLaunchArgument('amplitude', default_value='0.15',
                              description='Sine torque amplitude in Newton-metres'),
        DeclareLaunchArgument('freq_start_hz', default_value='0.25'),
        DeclareLaunchArgument('freq_end_hz', default_value='2.0'),
        DeclareLaunchArgument('n_frequencies', default_value='25'),
        DeclareLaunchArgument('output_csv', default_value='~/bode_pipeline.csv'),
        DeclareLaunchArgument('plot_output', default_value='~/bode_pipeline.png'),

        Node(
            package='bode_test',
            executable='bode_test_node',
            name='bode_test_node',
            output='screen',
            parameters=[
                config,
                {
                    'joint_index':    LaunchConfiguration('joint_index'),
                    'amplitude':      LaunchConfiguration('amplitude'),
                    'freq_start_hz':  LaunchConfiguration('freq_start_hz'),
                    'freq_end_hz':    LaunchConfiguration('freq_end_hz'),
                    'n_frequencies':  LaunchConfiguration('n_frequencies'),
                    'output_csv':     LaunchConfiguration('output_csv'),
                    'plot_output':    LaunchConfiguration('plot_output'),
                },
            ],
        ),
    ])
