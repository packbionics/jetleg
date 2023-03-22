from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    moveit_config = MoveItConfigsBuilder("jetleg_wheeled_testrig", package_name="wheeled_testrig_config").to_moveit_configs()

    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )

    return ld
