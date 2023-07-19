from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # rviz_arg = DeclareLaunchArgument('rvizconfig')
    rviz = LaunchConfiguration('rvizconfig')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz],
    )

    ld = LaunchDescription()

    # ld.add_action(rviz_arg)
    ld.add_action(rviz_node)

    return ld