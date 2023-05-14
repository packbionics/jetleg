import os

from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():    
    teleop_node = Node(
        package='jetleg_control',
        executable='jetleg_gazebo_gait_generator',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_path('jetleg_bringup'), 'launch'),
            '/jetleg_gazebo.launch.py']
            ),
    )
    
    return LaunchDescription([teleop_node, gazebo_sim])