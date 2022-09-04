import os

from ament_index_python.packages import (get_package_share_directory,
                                         get_package_share_path)
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_directory_path = get_package_share_directory('jetleg_description')

    bringup_path = get_package_share_path('jetleg_bringup')
    models_to_load = bringup_path / 'resource/jetleg_standalone_description.yaml'

    model_path = os.path.join(description_directory_path, 'urdf/jetleg_standalone.xacro')

    redefined_launch_arguments = {
        'models_to_load': str(models_to_load),
        'model': model_path
    }
    
    pybullet_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pybullet_ros'), 'launch'),
            '/jetleg_pybullet_ros.launch.py']
            ),
        launch_arguments=redefined_launch_arguments.items()
    )
    
    return LaunchDescription([pybullet_sim])