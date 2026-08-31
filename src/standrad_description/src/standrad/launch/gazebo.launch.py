from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("standard"))
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))

    model = LaunchConfiguration("model")
    entity_name = LaunchConfiguration("entity_name")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=str(package_share / "urdf" / "standard.urdf"),
                description="Absolute path to the Standard robot URDF",
            ),
            DeclareLaunchArgument(
                "entity_name",
                default_value="standard",
                description="Gazebo entity name",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(gazebo_share / "launch" / "gazebo.launch.py")
                )
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_footprint_base",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "base_footprint",
                    "--child-frame-id",
                    "base_link",
                ],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_standard",
                output="screen",
                arguments=["-entity", entity_name, "-file", model],
            ),
        ]
    )
