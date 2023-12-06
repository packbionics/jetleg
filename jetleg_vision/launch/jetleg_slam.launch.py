import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

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
