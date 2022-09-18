
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    zed2i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('jetleg_vision'), 'launch'),
            '/jetleg_zed2i.launch.py'])
    )
    
    pointcloud_proc = Node(
        package='jetleg_vision',
        executable='pointcloud_proc'
    )

    return LaunchDescription([zed2i_launch, pointcloud_proc])
