import os

from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_path = PathJoinSubstitution([
        FindPackageShare('jetleg_description'), 
        'urdf/', 
        'jetleg_testrig_vision.xacro'
    ])

    use_pybullet_env = LaunchConfiguration("use_pybullet_env")
    use_pybullet_env_arg = DeclareLaunchArgument("use_pybullet_env", default_value="False")

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
            FindPackageShare('jetleg_bringup'), 
            '/launch/',
            'jetleg_bringup.launch.py'
        ]),
        launch_arguments={
            'model': model_path,
            'use_pybullet_env': use_pybullet_env
        }.items()
    )
    
    ld = LaunchDescription()

    ld.add_action(use_pybullet_env_arg)

    ld.add_action(gait_generator_node)
    ld.add_action(teleop_node)
    ld.add_action(sim_env)

    return ld