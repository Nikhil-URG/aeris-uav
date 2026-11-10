from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bringup_share = get_package_share_directory("aeris_bringup")
    default_grid = os.path.join(
        bringup_share, "config", "grids", "city_grid.yaml"
    )
    grid_file = LaunchConfiguration("grid_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "grid_file",
                default_value=default_grid,
                description="Absolute path to an AERIS grid YAML",
            ),
            Node(
                package="aeris_grid_gui",
                executable="grid_gui",
                name="aeris_grid_gui",
                output="screen",
                additional_env={"QT_QPA_PLATFORM": "xcb"},
                parameters=[
                    {"grid_file": ParameterValue(grid_file, value_type=str)}
                ],
            ),
        ]
    )
