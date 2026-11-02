from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("aeris_bringup"),
        "config",
        "telemetry.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="aeris_flight_control",
                executable="telemetry_monitor",
                name="aeris_telemetry_monitor",
                output="screen",
                parameters=[config_file],
            )
        ]
    )
