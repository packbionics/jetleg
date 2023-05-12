import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    jetleg_bringup_dir = get_package_share_directory('jetleg_bringup')
    jetleg_vision_dir = get_package_share_directory('jetleg_vision')

    jetleg_pointcloud_launch_dir = os.path.join(jetleg_bringup_dir, 'launch')

    jetleg_pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([jetleg_pointcloud_launch_dir,
            '/jetleg_pointcloud_vision.launch.py'])
    )

    pointcloud_proc_params_file = os.path.join('config/params.yaml')
    pointcloud_proc_params = os.path.join(jetleg_vision_dir, pointcloud_proc_params_file)

    jetleg_pointcloud_proc_cpp = Node(
        package='jetleg_vision',
        executable='jetleg_pointcloud_proc',
        parameters=pointcloud_proc_params,
        remappings=[('/pointcloud', '/points'),
                    ('camera_state', 'camera/state')]
    )

    image_to_map_node = Node(
        package='map_to_jpeg',
        executable='image_to_map_node',
        parameters=[{'frame_id': 'transformed_map'}],
        remappings=[('image', 'traversibility')]
    )

    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0.0', '1.0', '0.0', '0.0', 'camera_link', 'partial_transformed_map']
    )

    another_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-0.5733227', '0.0', '0.0', '0.8193297', 'partial_transformed_map', 'partial_partial_transformed_map']
    )

    third_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '3', '-1', '0', '0.0', '0.0', '1', 'partial_partial_transformed_map', 'transformed_map']
    )

    return LaunchDescription([jetleg_pointcloud_launch, 
                                jetleg_pointcloud_proc_cpp, 
                                image_to_map_node, 
                                static_transform, 
                                another_transform,
                                third_transform])
