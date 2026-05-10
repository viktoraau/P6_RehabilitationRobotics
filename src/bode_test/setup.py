from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bode_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aladdin',
    maintainer_email='burgerbdd@gmail.com',
    description='Bode frequency-sweep test node for CTC and full admittance pipeline.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bode_test_node = bode_test.bode_test_node:main',
        ],
    },
)
