import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def add_launch_argument(name: str, default: str, description: str) -> tuple:
    launch_config = LaunchConfiguration(name)

    launch_arg = DeclareLaunchArgument(
        name, 
        default_value=default,
        description=description
    )
    
    return launch_config, launch_arg

def add_launch_file(package_name: str, launch_name: str):
    package = FindPackageShare(package_name)

    launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([package, '/launch', '/' + launch_name]),
    )

    return launch_file

def generate_launch_description():

    pybullet_ros_dir = get_package_share_directory('pybullet_ros')
    jetleg_description_dir = get_package_share_directory('jetleg_description')    

    config_file_path = os.path.join(pybullet_ros_dir, "config/pybullet/jetleg_pybullet_vision_params.yaml")
    model_config_file_path = os.path.join(pybullet_ros_dir, "config/environment/vision_example.yaml")
    testrig_vision_xacro_path = os.path.join(jetleg_description_dir, 'urdf/jetleg_testrig_vision.xacro')

    _, config_file_arg = add_launch_argument(
        'config_file',
        default=config_file_path, 
        description='Path to Pybullet plugin config file'
    )
    _, model_config_file_arg = add_launch_argument(
        'model_config_file',
        default=model_config_file_path,
        description='Describes the environment models to load into the environment'
    )
    _, model_arg = add_launch_argument(
        'model',
        default=testrig_vision_xacro_path,
        description='Robot model to load into the simulation'
    )

    jetleg_pybullet_ros = add_launch_file('pybullet_ros', 'bringup_robot_example.launch.py')

    ld = LaunchDescription()

    ld.add_action(config_file_arg)
    ld.add_action(model_config_file_arg)
    ld.add_action(model_arg)

    ld.add_action(jetleg_pybullet_ros)

    return ld
