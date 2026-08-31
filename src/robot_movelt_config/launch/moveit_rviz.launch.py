from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("standard", package_name="robot_movelt_config")
        .robot_description(mappings={"variant": LaunchConfiguration("variant")})
        .robot_description_semantic(
            file_path="config/standard.srdf.xacro",
            mappings={"variant": LaunchConfiguration("variant")},
        )
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)
