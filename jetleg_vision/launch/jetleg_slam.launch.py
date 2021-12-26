import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration


def generate_launch_description():
    
    share_dir = get_package_share_directory('jetleg_vision')
    
    path_to_vocabulary_launch_arg = DeclareLaunchArgument(
        "path_to_vocabulary",
        default_value=TextSubstitution(
            text=os.path.join(
                share_dir,
                "config/ORBvoc.txt"
            )
        )
    )
    path_to_settings_launch_arg = DeclareLaunchArgument(
        "path_to_settings",
        default_value=TextSubstitution(
            text=os.path.join(
                share_dir,
                "config/imx21983_slam_config.yaml"
            )
        )
    )
    
    do_rectify_launch_arg = DeclareLaunchArgument(
        "do_rectify",
        default_value=TextSubstitution(text="False")
    )
    
    return LaunchDescription([
        path_to_vocabulary_launch_arg,
        path_to_settings_launch_arg,
        do_rectify_launch_arg,
        Node(
            package='ros2_orbslam',
            executable='stereo',
            remappings=[
                ('/camera/right', '/camera/right/image_raw'),
                ('/camera/left', '/camera/right/image_raw'),
            ],
            parameters=[{
                'vocabulary_file_path': LaunchConfiguration('path_to_vocabulary'),
                'settings_file_path': LaunchConfiguration('path_to_settings'),
                'do_rectify': LaunchConfiguration('do_rectify'),
            }
            ]
        )
    ])
