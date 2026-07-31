"""Bring up the gesture_bot node graph.

    ros2 launch gesture_bot_ros gesture_bot.launch.py

    # drive a real board instead of the sim
    ros2 launch gesture_bot_ros gesture_bot.launch.py \
        actuator:=serial serial_port:=/dev/ttyACM0

    # replay a recorded session instead of opening the camera
    ros2 launch gesture_bot_ros gesture_bot.launch.py camera:=false
    ros2 bag play my_session.bag

    # open-vocabulary detection instead of YOLO's 80 fixed classes
    ros2 launch gesture_bot_ros gesture_bot.launch.py \
        locate:=true queries:="a red mug, keys, remote control"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = "gesture_bot_ros"


def generate_launch_description():
    params = os.path.join(get_package_share_directory(PKG), "config", "gesture_bot.yaml")

    args = [
        DeclareLaunchArgument(
            "gesture_bot_src",
            default_value=os.path.expanduser("~/vision_demos"),
            description="Repo root containing gesture_bot/ (the framework-free modules).",
        ),
        DeclareLaunchArgument("camera", default_value="true",
                              description="Launch v4l2_camera. false to replay a bag."),
        DeclareLaunchArgument("detector", default_value="false",
                              description="Also run YOLO object detection."),
        DeclareLaunchArgument(
            "locate", default_value="false",
            description="Run open-vocabulary detection (OWLv2) instead of YOLO. "
                        "Publishes the same Detection2DArray on the same topic, so "
                        "do NOT enable this together with detector:=true -- they "
                        "would both write /detections.",
        ),
        DeclareLaunchArgument(
            "queries", default_value="person, cup, laptop",
            description="Comma-separated open-vocabulary queries for locate:=true.",
        ),
        DeclareLaunchArgument("video_device", default_value="/dev/video0"),
        DeclareLaunchArgument("actuator", default_value="sim",
                              description="sim | hid | serial"),
        DeclareLaunchArgument("serial_port", default_value="",
                              description="e.g. /dev/ttyACM0. Empty means dry-run."),
    ]

    src = LaunchConfiguration("gesture_bot_src")
    # The nodes import the repo's modules at runtime; _core.py reads this.
    env = {"GESTURE_BOT_SRC": src}

    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        condition=IfCondition(LaunchConfiguration("camera")),
        parameters=[{"video_device": LaunchConfiguration("video_device")}],
        # Remap so every consumer sees a single flat /image_raw.
        remappings=[("/image_raw", "/image_raw")],
        output="screen",
    )

    gesture = Node(
        package=PKG, executable="gesture_node", name="gesture_node",
        parameters=[params], additional_env=env, output="screen",
    )

    detector = Node(
        package=PKG, executable="detector_node", name="detector_node",
        condition=IfCondition(LaunchConfiguration("detector")),
        parameters=[params], additional_env=env, output="screen",
    )

    # queries arrives as one comma-separated string because launch arguments are
    # strings; the node wants a list, so split it here rather than teaching the
    # node a second input format.
    locate = Node(
        package=PKG, executable="locate_node", name="locate_node",
        condition=IfCondition(LaunchConfiguration("locate")),
        parameters=[
            params,
            {"queries": ParameterValue(
                PythonExpression(
                    ["[s.strip() for s in '",
                     LaunchConfiguration("queries"),
                     "'.split(',') if s.strip()]"]
                ),
                value_type=None,
            )},
        ],
        additional_env=env, output="screen",
    )

    decision = Node(
        package=PKG, executable="decision_node", name="decision_node",
        parameters=[params], additional_env=env, output="screen",
    )

    base = Node(
        package=PKG, executable="base_driver_node", name="base_driver_node",
        parameters=[
            params,
            {
                "actuator": LaunchConfiguration("actuator"),
                "serial_port": LaunchConfiguration("serial_port"),
            },
        ],
        additional_env=env, output="screen",
    )

    return LaunchDescription([*args, camera, gesture, detector, locate, decision, base])
