import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def _launch_demo(context):
    variant = LaunchConfiguration("variant").perform(context)
    hardware = LaunchConfiguration("hardware").perform(context)
    builder = MoveItConfigsBuilder("standard", package_name="robot_movelt_config")

    if variant == "v2":
        builder = builder.robot_description(mappings={"variant": "v2"})
        builder = builder.robot_description_semantic(
            file_path="config/standard_v2.srdf"
        )

    moveit_config = builder.to_moveit_configs()

    if hardware == "v3":
        move_group = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
        )
        rviz = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=[
                "-d",
                str(moveit_config.package_path / "config" / "moveit.rviz"),
            ],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        )
        wait_for_move_group = ExecuteProcess(
            cmd=[
                sys.executable,
                str(moveit_config.package_path / "launch/wait_for_move_group.py"),
            ],
            output="screen",
        )
        return [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", "base_footprint"],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[moveit_config.robot_description],
            ),
            Node(
                package="bw_serial",
                executable="mantis_comm_node",
                name="mantis_comm_node",
                output="screen",
                parameters=[{
                    "port_name": LaunchConfiguration("port_name"),
                    "baud_rate": LaunchConfiguration("baud_rate"),
                    "publish_frequency": 1000,
                    "protocol_version": "v3",
                    "robot_version": "standard",
                    "v3_feedback_publish_frequency_hz": 250.0,
                }],
            ),
            Node(
                package="standard_v3_moveit",
                executable="trajectory_bridge",
                name="trajectory_bridge",
                output="screen",
                parameters=[
                    PathJoinSubstitution([
                        FindPackageShare("standard_v3_moveit"),
                        "config",
                        "bridge.yaml",
                    ]),
                    {
                        "auto_enable": LaunchConfiguration("auto_enable"),
                        "home_on_start": LaunchConfiguration("home_on_start"),
                    },
                ],
            ),
            Node(
                package="standard_motion_recorder",
                executable="motion_recorder_node",
                name="motion_recorder",
                output="screen",
                condition=IfCondition(LaunchConfiguration("motion_recorder")),
                parameters=[PathJoinSubstitution([
                    FindPackageShare("standard_motion_recorder"),
                    "config",
                    "recorder.yaml",
                ])],
            ),
            move_group,
            wait_for_move_group,
            RegisterEventHandler(
                OnProcessExit(target_action=wait_for_move_group, on_exit=[rviz])
            ),
        ]

    launch_description = generate_demo_launch(moveit_config)

    # Match the known-good workspace: RViz must initialize only after MoveGroup
    # has finished constructing the planning scene and advertised its service.
    rviz_action = launch_description.entities[6]
    if not isinstance(rviz_action, IncludeLaunchDescription):
        raise RuntimeError("Unexpected MoveIt demo launch structure: RViz include not found")
    launch_description.entities.remove(rviz_action)

    wait_for_move_group = ExecuteProcess(
        cmd=[
            sys.executable,
            str(moveit_config.package_path / "launch/wait_for_move_group.py"),
        ],
        output="screen",
    )
    launch_description.add_action(wait_for_move_group)
    launch_description.add_action(
        RegisterEventHandler(
            OnProcessExit(target_action=wait_for_move_group, on_exit=[rviz_action])
        )
    )
    return launch_description.entities

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "variant",
                default_value="v1",
                choices=["v1", "v2"],
                description="Robot model variant",
            ),
            DeclareLaunchArgument(
                "hardware",
                default_value="mock",
                choices=["mock", "v3"],
                description="Execution backend: mock simulation or V3 real robot",
            ),
            DeclareLaunchArgument("port_name", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baud_rate", default_value="2000000"),
            DeclareLaunchArgument("auto_enable", default_value="true"),
            DeclareLaunchArgument("home_on_start", default_value="true"),
            DeclareLaunchArgument("motion_recorder", default_value="true"),
            OpaqueFunction(function=_launch_demo),
        ]
    )
