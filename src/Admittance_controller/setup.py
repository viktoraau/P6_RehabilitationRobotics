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
            'orientation_admittance = Admittance_controller.orientation_admittance_node:main',
            'traj_to_joint_state = Admittance_controller.traj_to_joint_state:main',
        ],
    },
)
