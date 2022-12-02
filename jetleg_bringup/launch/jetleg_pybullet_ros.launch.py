import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration

def generate_launch_description():

    model = LaunchConfiguration('model')

    pybullet_ros_dir = get_package_share_directory('pybullet_ros')
    jetleg_description_dir = get_package_share_directory('jetleg_description')

    pybullet_ros_launch_dir = os.path.join(pybullet_ros_dir, 'launch')
    
    bringup_robot_example_path = os.path.join(pybullet_ros_launch_dir, 'bringup_robot_example.launch.py')
    default_model_path = os.path.join(jetleg_description_dir, 'urdf/jetleg_testrig.xacro')

    launch_arguments = {
        'model': model
    }

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model_path
    )

    bringup_robot_example = IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup_robot_example_path), launch_arguments=launch_arguments.items())

    return LaunchDescription([model_arg, bringup_robot_example])
