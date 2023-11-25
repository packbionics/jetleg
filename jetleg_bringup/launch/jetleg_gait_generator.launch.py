from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    # Node for controlling joints
    gait_generator_node = Node(
        package='jetleg_control',
        executable='jetleg_gait_generator.py',
        output='screen'
    )

    ld = LaunchDescription()

    # Mark the actions to execute
    ld.add_action(gait_generator_node)

    return ld