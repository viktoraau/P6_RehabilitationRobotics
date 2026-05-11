import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wrist_games'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    include_package_data=True,
    package_data={package_name: ['assets/*.wav']},
    zip_safe=True,
    maintainer='user',
    maintainer_email='burgerbdd@gmail.com',
    description='Wrist rehabilitation games driven by ROS 2 joint_states',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_node     = wrist_games.calibration_node:main',
            'game_node            = wrist_games.game_node:main',
            'airplane_node        = wrist_games.airplane_node:main',
            'wrist-game-manager   = wrist_games.game_manager:main',
            'wrist-airplane-game  = wrist_games.airplane_game:main',
            'wrist-catcher-game   = wrist_games.catcher_game:main',
            'wrist-jedi-game      = wrist_games.jedi_game:main',
            'wrist-octagon-game   = wrist_games.octagon_game:main',
            'wrist-pendulum-game  = wrist_games.pendulum_game:main',
            'wrist-tunnel-game    = wrist_games.tunnel_game:main',
            'wrist-xwing-game     = wrist_games.xwing_game:main',
        ],
    },
)
