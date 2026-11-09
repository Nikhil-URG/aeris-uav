from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bringup_share = get_package_share_directory("aeris_bringup")
    bridge_config = os.path.join(
        bringup_share, "config", "gz_lidar_bridge.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_lidar_bridge",
                output="screen",
                parameters=[{"config_file": bridge_config}],
            ),
            Node(
                package="aeris_navigation",
                executable="navigation_node",
                name="aeris_navigation",
                output="screen",
                parameters=[
                    {
                        "scan_topic": "/aeris/lidar/scan",
                        "output_topic": "/aeris/navigation/obstacles",
                        "forward_cone_deg": 70.0,
                        "obstacle_threshold_m": 4.0,
                        "publish_max_range_m": 10.0,
                        "sector_width_deg": 10.0,
                    }
                ],
            ),
        ]
    )
