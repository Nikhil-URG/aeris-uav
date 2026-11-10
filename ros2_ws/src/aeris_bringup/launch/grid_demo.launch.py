from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bringup_share = get_package_share_directory("aeris_bringup")
    launch_dir = os.path.join(bringup_share, "launch")
    default_grid = os.path.join(
        bringup_share, "config", "grids", "city_grid.yaml"
    )
    grid_file = LaunchConfiguration("grid_file")
    start_gui = LaunchConfiguration("start_gui")

    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "grid_navigation.launch.py")
        ),
        launch_arguments={"grid_file": grid_file}.items(),
    )

    gui_node = Node(
        package="aeris_grid_gui",
        executable="grid_gui",
        name="aeris_grid_gui",
        output="screen",
        condition=IfCondition(start_gui),
        additional_env={"QT_QPA_PLATFORM": "xcb"},
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
            DeclareLaunchArgument(
                "start_gui",
                default_value="true",
                description="Open the PyQt5 goal-selection GUI",
            ),
            navigation_stack,
            gui_node,
        ]
    )
