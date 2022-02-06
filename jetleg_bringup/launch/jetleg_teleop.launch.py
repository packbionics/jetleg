
import os

from ament_index_python.packages import (get_package_share_directory,
                                         get_package_share_path)
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_path = get_package_share_path('jetleg_description')
    default_rviz_config_path = description_path / 'rviz/urdf.rviz'
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                    description='Absolute path to rviz config file')
    
    teleop_node = Node(
        package='jetleg_control',
        executable='jetleg_teleop_key',
        # create a new terminal window for receiving keys, xterm must be installed first
        prefix = 'xterm -e', 
        output='screen'
    )
    pybullet_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pybullet_ros'), 'launch'),
            '/jetleg_pybullet_ros.launch.py'])
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    return LaunchDescription([rviz_arg, teleop_node, pybullet_sim, rviz_node])
