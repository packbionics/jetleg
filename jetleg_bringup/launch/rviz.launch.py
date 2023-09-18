from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    rviz_config = LaunchConfiguration('rvizconfig')
    rviz_config_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value="",
        description="Specifies Rviz Config file to use with RVIZ 2"
    )

    rviz_toggle = LaunchConfiguration('rviz')
    rviz_toggle_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Toggles the startup of the RVIZ visualization'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz_toggle)
    )

    ld = LaunchDescription()

    ld.add_action(rviz_config_arg)
    ld.add_action(rviz_toggle_arg)

    ld.add_action(rviz_node)

    return ld