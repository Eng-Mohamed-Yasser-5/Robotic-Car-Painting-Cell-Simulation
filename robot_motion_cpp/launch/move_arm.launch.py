from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "my_robot",
            package_name="robot_moveit_config",
        )
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="robot_motion_cpp",
            executable="simple_collision",
            output="screen",
            parameters=[moveit_config.to_dict(),
                        {'use_sim_time': True}],
        )
    ])