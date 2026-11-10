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

    telemetry_node = Node(
        package="aeris_flight_control",
        executable="telemetry_monitor",
        name="aeris_telemetry_monitor",
        output="screen",
        parameters=[os.path.join(bringup_share, "config", "telemetry.yaml")],
    )

    offboard_node = Node(
        package="aeris_flight_control",
        executable="offboard_controller",
        name="aeris_offboard_controller",
        output="screen",
        parameters=[
            os.path.join(bringup_share, "config", "offboard_grid.yaml")
        ],
    )

    safety_node = Node(
        package="aeris_safety",
        executable="safety_node",
        name="aeris_safety",
        output="screen",
        parameters=[
            os.path.join(bringup_share, "config", "safety_grid.yaml")
        ],
    )

    grid_node = Node(
        package="aeris_grid_navigation",
        executable="grid_navigation",
        name="aeris_grid_navigation",
        output="screen",
        parameters=[
            {"grid_file": ParameterValue(grid_file, value_type=str)}
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "grid_file",
                default_value=default_grid,
                description="Absolute path to an AERIS grid YAML",
            ),
            telemetry_node,
            offboard_node,
            safety_node,
            grid_node,
        ]
    )
