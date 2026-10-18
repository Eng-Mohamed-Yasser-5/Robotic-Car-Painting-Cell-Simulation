from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "ur10",
            package_name="robot_moveit_config",
        )
        .to_moveit_configs()
    )

    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict["use_sim_time"] = True

    moveit_config.robot_description["use_sim_time"] = True

    return generate_move_group_launch(moveit_config)

