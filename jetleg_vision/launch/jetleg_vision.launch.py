import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    jetleg_vision_dir = get_package_share_directory('jetleg_vision')

    pointcloud_proc_params_file = os.path.join('config/params.yaml')
    pointcloud_proc_params = os.path.join(jetleg_vision_dir, pointcloud_proc_params_file)

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


    jetleg_pointcloud_proc = Node(
        package='jetleg_vision',
        executable='jetleg_pointcloud_proc.py',
        # parameters=[pointcloud_proc_params],
        remappings=[('/zed2i/zed_node/point_cloud/cloud_registered', '/points'),
                    ('camera_state', 'camera/state')]
    )

    jetleg_pointcloud_proc_cpp = Node(
        package='jetleg_vision',
        executable='jetleg_pointcloud_proc',
        parameters=[pointcloud_proc_params],
        remappings=[('/pointcloud', '/points'),
                    ('camera_state', 'camera/state')]
    )

    return LaunchDescription([pointcloud_xyz_node,
                                jetleg_pointcloud_proc])
