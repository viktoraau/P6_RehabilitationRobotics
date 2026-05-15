"""Launch the Bode test node in CTC mode.

Assumes the full system is already running:
  - mab_three_axis_bringup.launch.py  (hardware + ros2_control)
  - full_system_bringup.launch.py     (RSP, CTC-KDL, IK, admittance)

To test a different joint, pass joint_index:=1 or joint_index:=2.
To change amplitude:  amplitude:=0.05
To change sweep range: freq_start_hz:=0.1 freq_end_hz:=8.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('bode_test')
    config = os.path.join(pkg, 'config', 'bode_test_ctc.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('joint_index', default_value='0',
                              description='Joint to excite: 0=joint_1, 1=joint_2, 2=joint_3'),
        DeclareLaunchArgument('amplitude', default_value='0.45',
                              description='Sine amplitude in radians'),
        DeclareLaunchArgument('freq_start_hz', default_value='0.5'),
        DeclareLaunchArgument('freq_end_hz', default_value='4.0'),
        DeclareLaunchArgument('n_frequencies', default_value='35'),
        DeclareLaunchArgument('output_csv', default_value='~/bode_ctc.csv'),
        DeclareLaunchArgument('plot_output', default_value='~/bode_ctc.png'),

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
