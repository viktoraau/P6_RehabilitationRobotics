
from setuptools import setup
import os
from glob import glob

package_name = 'brake_relay_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rehabo',
    maintainer_email='cj7.viktor@gmail.com',
    description='Brake relay node for controlling relay logic',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'brake_relay_node = brake_relay_pkg.brake_relay_node:main',
        ],
    },
)