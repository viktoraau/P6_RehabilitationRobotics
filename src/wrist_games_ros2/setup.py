import glob
from setuptools import setup

package_name = "wrist_games_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob.glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    include_package_data=True,
    package_data={package_name: ["assets/*.wav"]},
    zip_safe=True,
    maintainer="OpenWrist Team",
    maintainer_email="devnull@example.com",
    description="Wrist-controlled ROS2 mini-games with score and sound feedback.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wrist-game-manager  = wrist_games_ros2.game_manager:main",
            "wrist-airplane-game = wrist_games_ros2.airplane_game:main",
            "wrist-jedi-game     = wrist_games_ros2.jedi_game:main",
            "wrist-octagon-game  = wrist_games_ros2.octagon_game:main",
            "wrist-pendulum-game = wrist_games_ros2.pendulum_game:main",
            "wrist-xwing-game    = wrist_games_ros2.xwing_game:main",
            "wrist-tunnel-game   = wrist_games_ros2.tunnel_game:main",
            "wrist-catcher-game  = wrist_games_ros2.catcher_game:main",
        ],
    },
)
