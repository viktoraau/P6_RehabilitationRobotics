from setuptools import find_packages, setup

package_name = 'Admittance_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rednux',
    maintainer_email='marcusrodbro@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'admittance_controller = Admittance_controller.admittance_controller_node:main',
            'joint_space_admittance = Admittance_controller.joint_space_admittance_node:main',
            'interactive_force_tester = Admittance_controller.forces:main',
            'command_publisher = Admittance_controller.traj_to_joint_state:main',
            'admittance_to_servo_bridge = Admittance_controller.admittance_to_servo_bridge:main',
        ],
    },
)
