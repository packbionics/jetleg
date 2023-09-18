from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pointcloud_proc = Node(
        package="jetleg_vision",
        executable="jetleg_pointcloud_proc.py",
        remappings=[('/zed2i/zed_node/point_cloud/cloud_registered', '/camera/points')]
    )

    ld = LaunchDescription()

    ld.add_action(pointcloud_proc)

    return ld
