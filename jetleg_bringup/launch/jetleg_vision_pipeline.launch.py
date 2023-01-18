import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    jetleg_bringup_dir = get_package_share_directory('jetleg_bringup')
    jetleg_pointcloud_launch_dir = os.path.join(jetleg_bringup_dir, 'launch')

    jetleg_pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([jetleg_pointcloud_launch_dir,
            '/jetleg_pointcloud_vision.launch.py'])
    )

    node_parameters = {
        'X_MIN': -1.0,
        'X_MAX':  1.0,

        'Y_MIN':  -1.5,
        'Y_MAX':  1.5,

        'Z_MIN':  0.6,
        'Z_MAX':  3.0,
    }

    jetleg_pointcloud_proc_cpp = Node(
        package='jetleg_vision',
        executable='jetleg_pointcloud_proc',
        parameters=[node_parameters],
        remappings=[('/pointcloud', '/points'),
                    ('camera_state', 'camera/state')]
    )

    image_to_map_node = Node(
        package='map_to_jpeg',
        executable='image_to_map_node',
        remappings=[('image', 'traversibility')]
    )

    return LaunchDescription([jetleg_pointcloud_launch, jetleg_pointcloud_proc_cpp, image_to_map_node])
