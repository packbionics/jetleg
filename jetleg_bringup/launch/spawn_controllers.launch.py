from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    controller_list = ['leg_controller', 'leg_intact_controller', 'joint_state_broadcaster']
    for controller in controller_list:
        ld.add_action(Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=[controller]
        ))


    return ld