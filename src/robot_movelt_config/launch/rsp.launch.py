from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("standard", package_name="robot_movelt_config")
        .robot_description(mappings={"variant": LaunchConfiguration("variant")})
        .to_moveit_configs()
    )
    return generate_rsp_launch(moveit_config)
