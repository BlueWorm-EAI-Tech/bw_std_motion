from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


VALID_VARIANTS = ("v1", "v2")


def launch_robot(context, package_share):
    variant = LaunchConfiguration("variant").perform(context).lower()
    model_override = LaunchConfiguration("model").perform(context)

    if variant not in VALID_VARIANTS:
        raise ValueError(
            f"Unsupported robot variant '{variant}'. Choose one of: "
            + ", ".join(VALID_VARIANTS)
        )

    model = model_override or (
        f"{package_share}/urdf/standard.urdf"
        if variant == "v1"
        else f"{package_share}/urdf/standard_{variant}.urdf"
    )
    with open(model, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(gui),
            arguments=[model],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            condition=UnlessCondition(gui),
            arguments=[model],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("standrad_description")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "variant",
                default_value="v1",
                choices=list(VALID_VARIANTS),
                description="Robot model variant",
            ),
            DeclareLaunchArgument(
                "model",
                default_value="",
                description="Optional absolute URDF path overriding the selected variant",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the joint state publisher GUI",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=f"{package_share}/config/standard.rviz",
                description="Absolute path to an RViz2 configuration file",
            ),
            OpaqueFunction(function=launch_robot, args=[package_share]),
        ]
    )
