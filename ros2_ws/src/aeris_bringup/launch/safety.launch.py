from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bringup_share = get_package_share_directory("aeris_bringup")
    safety_config = os.path.join(bringup_share, "config", "safety.yaml")

    return LaunchDescription(
        [
            Node(
                package="aeris_safety",
                executable="safety_node",
                name="aeris_safety",
                output="screen",
                parameters=[safety_config],
            ),
        ]
    )
