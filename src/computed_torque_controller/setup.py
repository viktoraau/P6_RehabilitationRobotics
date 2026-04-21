"""Package setup for the computed torque controller."""

from setuptools import find_packages, setup

package_name = 'computed_torque_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['dynamic_matrices/*.txt'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rehabo',
    maintainer_email='rehabo@example.com',
    description='Computed torque controller',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            (
                'controller_node = '
                'computed_torque_controller.controller_node:main'
            ),
        ],
    },
)
