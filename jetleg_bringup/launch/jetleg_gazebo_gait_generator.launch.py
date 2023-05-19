import os

from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_path('jetleg_bringup'), 'launch'),
            '/jetleg_gazebo.launch.py'
        ])
    )

    gait_generator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_path('jetleg_control'), 'launch'),
            '/jetleg_gait_generator.launch.py'
        ])
    )

    ld = LaunchDescription()

    ld.add_action(gazebo_sim)
    ld.add_action(gait_generator)
    
    return ld