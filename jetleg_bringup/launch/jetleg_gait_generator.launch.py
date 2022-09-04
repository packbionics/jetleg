import os

from ament_index_python.packages import (get_package_share_directory,
                                         get_package_share_path)
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_path = get_package_share_path('jetleg_description')
    description_directory_path = get_package_share_directory('jetleg_description')

    default_rviz_config_path = description_path / 'rviz/urdf.rviz'
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                    description='Absolute path to rviz config file')

    bringup_path = get_package_share_path('jetleg_bringup')
    models_to_load = bringup_path / 'resource/model_descriptions.yaml'

    model_path = os.path.join(description_directory_path, 'urdf/wheeled_testrig.xacro')

    redefined_launch_arguments = {
        'models_to_load': str(models_to_load),
        'model': model_path
    }
    
    teleop_node = Node(
        package='jetleg_control',
        executable='jetleg_gait_generator',
        output='screen'
    )
    pybullet_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pybullet_ros'), 'launch'),
            '/jetleg_pybullet_ros.launch.py']
            ),
        launch_arguments=redefined_launch_arguments.items()
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    return LaunchDescription([rviz_arg, teleop_node, pybullet_sim, rviz_node])