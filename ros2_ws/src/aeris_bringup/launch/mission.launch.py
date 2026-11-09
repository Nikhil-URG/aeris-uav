from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bringup_share = get_package_share_directory("aeris_bringup")
    default_mission = os.path.join(
        bringup_share, "config", "missions", "inspection_alpha.yaml"
    )

    mission_file = LaunchConfiguration("mission_file")
    auto_start = LaunchConfiguration("auto_start")

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
        parameters=[os.path.join(bringup_share, "config", "offboard_mission.yaml")],
    )

    mission_node = Node(
        package="aeris_mission_manager",
        executable="mission_manager",
        name="aeris_mission_manager",
        output="screen",
        parameters=[
            {
                "mission_file": ParameterValue(mission_file, value_type=str),
                "auto_start": ParameterValue(auto_start, value_type=bool),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mission_file",
                default_value=default_mission,
                description="Absolute path to an AERIS mission YAML",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="false",
                description="Start the mission automatically after loading",
            ),
            telemetry_node,
            offboard_node,
            mission_node,
        ]
    )
