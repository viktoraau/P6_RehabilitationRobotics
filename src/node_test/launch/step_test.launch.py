"""Launch file for the step-response accuracy test node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('node_test')
    default_config = os.path.join(pkg_share, 'config', 'step_test.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to the step_test.yaml parameter file.',
    )

    output_csv_arg = DeclareLaunchArgument(
        'output_csv',
        default_value=os.path.expanduser('~/step_test_results.csv'),
        description='Path where the CSV results file is written.',
    )

    n_waypoints_arg = DeclareLaunchArgument(
        'n_waypoints',
        default_value='10',
        description='Number of random waypoints in the test sequence.',
    )

    seed_arg = DeclareLaunchArgument(
        'random_seed',
        default_value='42',
        description='RNG seed (-1 = random each run).',
    )

    node = Node(
        package='node_test',
        executable='step_test_node',
        name='step_test_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {
                'output_csv':   LaunchConfiguration('output_csv'),
                'n_waypoints':  LaunchConfiguration('n_waypoints'),
                'random_seed':  LaunchConfiguration('random_seed'),
            },
        ],
    )

    return LaunchDescription([
        config_arg,
        output_csv_arg,
        n_waypoints_arg,
        seed_arg,
        node,
    ])
