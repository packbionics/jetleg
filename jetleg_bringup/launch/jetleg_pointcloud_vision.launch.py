import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    jetleg_bringup_dir = get_package_share_directory('jetleg_bringup')
    jetleg_vision_cpp_dir = get_package_share_directory('jetleg_vision_cpp')

    rviz_config_file = os.path.join(jetleg_vision_cpp_dir, 'config/pybullet_pointcloud_vision_config.rviz')
    jetleg_bringup_launch_dir = os.path.join(jetleg_bringup_dir, 'launch')

    pybullet_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([jetleg_bringup_launch_dir,
            '/jetleg_vision_example.launch.py'])
    )
    
    # launch plugin through rclcpp_components container
    pointcloud_xyz_node = ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyz_rgb_node',
                    remappings=[('depth_registered/image_rect', '/camera/depth/image_raw'),
                                ('rgb/camera_info', '/camera/info'),
                                ('rgb/image_rect_color', 'camera/image_raw')]
                ),
            ],
            output='screen',
    )
    
    rviz_config = Node(package='rviz2',
                       executable='rviz2',
                       output='screen',
                       arguments=['-d', rviz_config_file]
                    )

    return LaunchDescription([pybullet_sim_launch, pointcloud_xyz_node, rviz_config])