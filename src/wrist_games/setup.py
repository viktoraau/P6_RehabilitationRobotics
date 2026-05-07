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
    zip_safe=True,
    maintainer='user',
    maintainer_email='burgerbdd@gmail.com',
    description='Wrist rehabilitation games driven by ROS 2 joint_states',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_node  = wrist_games.calibration_node:main',
            'game_node         = wrist_games.game_node:main',
            'airplane_node     = wrist_games.airplane_node:main',
        ],
    },
)
