import os
from glob import glob

from setuptools import find_packages, setup

package_name = "gesture_bot_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Raahim Nawaz",
    maintainer_email="raahimtnawaz@gmail.com",
    description="ROS2 wrapper nodes for the gesture_bot perception -> decision -> actuation loop.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gesture_node = gesture_bot_ros.gesture_node:main",
            "detector_node = gesture_bot_ros.detector_node:main",
            "decision_node = gesture_bot_ros.decision_node:main",
            "base_driver_node = gesture_bot_ros.base_driver_node:main",
        ],
    },
)
