from setuptools import setup
from glob import glob
import os

package_name = "ft_six_axis"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="ROS 2 driver for 6-axis FT sensor",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ft_sensor_node = ft_six_axis.ft_sensor_node:main",
        ],
    },
)
