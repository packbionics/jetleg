from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    gait_generator_node = Node(
        package='jetleg_control',
        executable='jetleg_gait_generator.py',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    control_node = Node(
        package='jetleg_control',
        executable='forwarder.py',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    ld.add_action(gait_generator_node)
    ld.add_action(control_node)

    return ld