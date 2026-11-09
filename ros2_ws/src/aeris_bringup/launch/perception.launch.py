from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bringup_share = get_package_share_directory("aeris_bringup")
    bridge_config = os.path.join(
        bringup_share, "config", "gz_camera_bridge.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_camera_bridge",
                output="screen",
                parameters=[{"config_file": bridge_config}],
            ),
            Node(
                package="aeris_perception",
                executable="perception",
                name="aeris_perception",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/aeris/camera/image_raw",
                        "camera_info_topic": "/aeris/camera/camera_info",
                        "detections_topic": "/aeris/perception/detections",
                        "class_name": "red_target",
                        "min_area_px": 500,
                        "hfov_rad": 1.74,
                    }
                ],
            ),
        ]
    )
