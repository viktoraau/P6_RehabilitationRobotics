from setuptools import setup
from glob import glob
import os

package_name = "ft300_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="ROS 2 driver and calibration tools for Robotiq FT300",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ft300_wrench_node = ft300_ros2.ft300_wrench_node:main",
            "ft300_calibration_node = ft300_ros2.ft300_calibration_node:main",
        ],
    },
)