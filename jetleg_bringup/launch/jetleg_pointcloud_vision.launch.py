import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    pybullet_ros_dir = get_package_share_directory('pybullet_ros')
    jetleg_vision_cpp_dir = get_package_share_directory('jetleg_vision_cpp')

    rviz_config_file = os.path.join(jetleg_vision_cpp_dir, 'config/pybullet_pointcloud_vision_config.rviz')
    pybullet_launch_dir = os.path.join(pybullet_ros_dir, 'launch')

    pybullet_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pybullet_launch_dir,
            '/vision_example.launch.py'])
        )

    # create a static tf2 transform publisher
    pointcloud_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0.5', '0.5', '0.5', '0.5', 'camera_link', 'pc2']
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
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[('image_rect', '/camera/depth/image_raw'),
                                ('camera_info', '/camera/info'),
                                ('image', '/camera/depth/converted_image')]
                ),
            ],
            output='screen',
    )
    
    rviz_config = Node(package='rviz2',
                       executable='rviz2',
                       output='screen',
                       arguments=['-d', rviz_config_file]
                    )

    return LaunchDescription([pointcloud_tf, pybullet_sim_launch, pointcloud_xyz_node, rviz_config])