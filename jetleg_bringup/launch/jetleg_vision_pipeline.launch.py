import os
from ament_index_python.packages import get_package_share_path

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def add_launch_argument(name: str, default: str, description: str) -> tuple:
    launch_config = LaunchConfiguration(name)

    launch_arg = DeclareLaunchArgument(
        name, 
        default_value=default,
        description=description
    )
    
    return launch_config, launch_arg

def add_launch_file(package_name: str, launch_name: str, conditional=None):
    package = FindPackageShare(package_name)

    launch_condition = None
    if conditional is not None:
        launch_condition = IfCondition(
            conditional
        )

    launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([package, '/launch', '/' + launch_name]),
        condition=launch_condition
    )

    return launch_file


def generate_launch_description():

    jetleg_bringup_path = get_package_share_path('jetleg_bringup')
    default_rviz_config_path = os.path.join(jetleg_bringup_path, 'config', 'jetleg_vision.rviz')

    use_rviz, use_rviz_arg = add_launch_argument(
        'use_rviz',
        default='False',
        description='Specifies to use RVIZ for visualization'
    )

    rvizconfig, rvizconfig_arg = add_launch_argument(
        'rvizconfig',
        default=default_rviz_config_path,
        description='Specifies the RVIZ config file to use for visualization'
    )

    jetleg_vision = add_launch_file('jetleg_vision', 'jetleg_vision.launch.py')
    pybullet_sim = add_launch_file('jetleg_bringup', 'jetleg_vision_pybullet_ros.launch.py')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rvizconfig],
        condition=IfCondition(use_rviz)
    )

    ld = LaunchDescription()

    ld.add_action(rvizconfig_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(pybullet_sim)
    ld.add_action(jetleg_vision)
    ld.add_action(rviz_node)
    
    return ld
