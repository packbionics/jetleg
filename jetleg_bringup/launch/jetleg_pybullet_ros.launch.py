import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from packbionics_launch_utils.launch_utils import add_launch_argument

def generate_launch_description():

    pybullet_ros_dir = get_package_share_directory('pybullet_ros')
    jetleg_description_dir = get_package_share_directory('jetleg_description')

    pybullet_ros_launch_dir = os.path.join(pybullet_ros_dir, 'launch')
    
    bringup_robot_example_path = os.path.join(pybullet_ros_launch_dir, 'bringup_robot_example.launch.py')
    default_model_path = os.path.join(jetleg_description_dir, 'urdf/jetleg_testrig.xacro')

    default_rviz_config_path = os.path.join(jetleg_description_dir, 'rviz/urdf.rviz')

    _, model_arg = add_launch_argument('model', default=default_model_path, description='Robot model to be loaded into simulation')
    rviz, rviz_arg = add_launch_argument(
        'rvizconfig', 
        default=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    bringup_robot_example = IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup_robot_example_path))

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz],
    )


    ld = LaunchDescription()

    ld.add_action(model_arg)
    ld.add_action(rviz_arg)

    ld.add_action(bringup_robot_example)
    ld.add_action(rviz_node)

    return ld
