from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def add_launch_argument(name: str, default: str, description: str) -> tuple:
    launch_config = LaunchConfiguration(name)

    launch_arg = DeclareLaunchArgument(
        name, 
        default_value=default,
        description=description
    )
    
    return launch_config, launch_arg

def add_launch_file(package_name: str, launch_name: str, conditional=None):
    package = FindPackageShare(package_name)

    launch_condition = None
    if conditional is not None:
        launch_condition = IfCondition(
            conditional
        )

    launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([package, '/launch', '/' + launch_name]),
        condition=launch_condition
    )

    return launch_file


def generate_launch_description():

    _, use_rviz_arg = add_launch_argument(
        'use_rviz',
        default='True',
        description='Specifies to use RVIZ for visualization'
    )

    sim_launch = add_launch_file('jetleg_bringup', 'jetleg_bringup.launch.py')
    rsp = add_launch_file('jetleg_bringup', 'rsp.launch.py')

    pointcloud_proc = Node(
        package="jetleg_vision",
        executable="jetleg_pointcloud_proc.py",
        remappings=[('/zed2i/zed_node/point_cloud/cloud_registered', '/camera/points')]
    )

    ld = LaunchDescription()

    ld.add_action(use_rviz_arg)
    ld.add_action(sim_launch)
    ld.add_action(rsp)
    ld.add_action(pointcloud_proc)

    return ld
