import os
from ament_index_python.packages import get_package_share_path

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    jetleg_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("jetleg_vision"), 
            '/launch/'
            'jetleg_vision.launch.py'
        ])
    )


    pybullet_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('jetleg_bringup'), 
            '/launch/',
            'jetleg_vision_pybullet_ros.launch.py'
        ])
    )

    ld = LaunchDescription()

    ld.add_action(pybullet_sim)
    ld.add_action(jetleg_vision)
    
    return ld
