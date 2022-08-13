from http.server import executable
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
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

    gen_pointcloud = Node(package='pointcloud_proc_cpp',
                          executable='gen_pointcloud',
                          output='screen',
                          remappings=[('image', 'camera/depth/image_raw'),
                                      ('pointcloud', 'camera/point_cloud'),
                                      ('camera_state', 'camera/state'),
                                      ('camera_params', 'camera/params')]
                    )

    jetleg_pointcloud_proc = Node(package='jetleg_vision_cpp',
                                  executable='jetleg_pointcloud_proc',
                                  output='screen',
                                  remappings=[('pointcloud', 'camera/point_cloud'),
                                             ('camera_state', 'camera/state')]
                            )
    
    rviz_config = Node(package='rviz2',
                       executable='rviz2',
                       output='screen',
                       arguments=['-d', rviz_config_file]
                    )

    return LaunchDescription([pybullet_sim_launch, gen_pointcloud, jetleg_pointcloud_proc, rviz_config])