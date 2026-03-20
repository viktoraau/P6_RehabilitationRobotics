from setuptools import setup

package_name = "rehab_candle_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/rehab_bringup.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rehab",
    maintainer_email="rehab@example.com",
    description="Launch and interactive speed controller for MAB MD drives and PDS.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "interactive_speed_control = rehab_candle_control.interactive_speed_control:main",
        ],
    },
)
