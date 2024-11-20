from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("jetleg", package_name="jetleg_moveit_config").to_moveit_configs()

    # MoveGroupInterface demo executable
    moveit_config.robot_description['use_sim_time'] = True
    moveit_config.robot_description_semantic['use_sim_time'] = True
    moveit_config.robot_description_kinematics['use_sim_time'] = True

    move_group_demo = Node(
        name="move_group_interface_tutorial",
        package="jetleg_planner",
        executable="move_group_interface_tutorial",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([move_group_demo])