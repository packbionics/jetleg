import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pybullet_ros_dir = get_package_share_directory('pybullet_ros')
    jetleg_description_dir = get_package_share_directory('jetleg_description')

    pybullet_ros_launch_dir = os.path.join(pybullet_ros_dir, 'launch')
    
    bringup_robot_example_path = os.path.join(pybullet_ros_launch_dir, 'bringup_robot_example.launch.py')

    config_file_path = os.path.join(pybullet_ros_dir, "config/pybullet/leg_control_params.yaml")
    single_jetleg_xacro_path = os.path.join(jetleg_description_dir, 'urdf/jetleg_single.xacro')

    launch_arguments = {
        'config_file': config_file_path,
        'model': single_jetleg_xacro_path
    }

    jetleg_pybullet_ros = IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup_robot_example_path), launch_arguments=launch_arguments.items())

    return LaunchDescription([jetleg_pybullet_ros])