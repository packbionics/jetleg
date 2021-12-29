import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration


def generate_launch_description():
    
    share_dir = get_package_share_directory('jetleg_vision')

    stereo_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('image_pub'),
                'launch',
                'stereo_image_pub.launch.py'
            )
        ])
    )

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('orb_slam2_ros'),
                'ros/launch',
                'orb_slam2_imx21983_stereo_launch.py'
            )
        ])
    )

    return LaunchDescription([
        stereo_publisher,
        slam_node
    ])
