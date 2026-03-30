from setuptools import find_packages, setup

package_name = "mab_brake_chopper"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/brake_chopper.launch.py"]),
        ("share/" + package_name, ["README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rehab",
    maintainer_email="rehab@example.com",
    description="ROS 2 brake chopper control driven by PDS bus voltage telemetry.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "brake_chopper_node = mab_brake_chopper.brake_chopper_node:main",
        ],
    },
)
