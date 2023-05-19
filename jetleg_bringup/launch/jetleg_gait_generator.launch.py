import os

from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_path = get_package_share_path('jetleg_description')
    default_rviz_config_path = description_path / 'rviz/urdf.rviz'
    model_path = os.path.join(description_path, 'urdf/jetleg_wheeled_testrig.xacro')

    launch_arguments = {
        'model': model_path
    }

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=str(default_rviz_config_path), 
        description='Absolute path to rviz config file'
    )
    
    gait_generator_node = Node(
        package='jetleg_control',
        executable='jetleg_gait_generator.py',
        output='screen'
    )

    teleop_node = Node(
        package='jetleg_control',
        executable='forwarder.py',
        output='screen'
    )
    
    pybullet_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_path('jetleg_bringup'), 'launch'),
            '/jetleg_pybullet_ros.launch.py'
        ]),
        launch_arguments=launch_arguments.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    ld = LaunchDescription()

    ld.add_action(rviz_arg)
    ld.add_action(gait_generator_node)
    ld.add_action(teleop_node)
    ld.add_action(pybullet_sim)
    ld.add_action(rviz_node)

    return ld