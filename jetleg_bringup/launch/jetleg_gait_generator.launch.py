import os

from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_path = get_package_share_path('jetleg_description')
    model_path = os.path.join(description_path, 'urdf/jetleg_wheeled_testrig.xacro')

    launch_arguments = {
        'model': model_path,
        'use_pybullet_env': 'False'
    }
    
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
    
    sim_env = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_path('jetleg_bringup'), 'launch'),
            '/jetleg_bringup.launch.py'
        ]),
        launch_arguments=launch_arguments.items()
    )
    
    ld = LaunchDescription()

    ld.add_action(gait_generator_node)
    ld.add_action(teleop_node)
    ld.add_action(sim_env)

    return ld