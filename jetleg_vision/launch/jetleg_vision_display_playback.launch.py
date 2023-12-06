import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    # Camera name. Can be different from camera model,
    # used to distinguish camera in multi-camera systems
    camera_name = 'zed2i'

    # Rviz2 Configurations to be loaded by ZED Node
    config_rviz2 = os.path.join(
        get_package_share_directory('jetleg_description'),
        'rviz',
        'zed2i_visualize.rviz'
    )

    # Set LOG format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'

    # Rviz2 node
    rviz2_node = Node(
        package='rviz2',
        namespace=camera_name,
        executable='rviz2',
        name=camera_name+'_rviz2',
        output='screen',
        arguments=[["-d"], [config_rviz2]],
        parameters=[{'use_sim_time': False}]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()

    ld.add_action(rviz2_node)

    return ld
