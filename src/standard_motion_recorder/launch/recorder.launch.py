from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("standard_motion_recorder"), "config", "recorder.yaml"])
    return LaunchDescription([
        Node(
            package="standard_motion_recorder",
            executable="motion_recorder_node",
            name="motion_recorder",
            output="screen",
            parameters=[config],
        )
    ])
