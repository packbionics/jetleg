from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


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

    # Toggles the use of Pybullet
    pybullet_arg = DeclareLaunchArgument(
        name='pybullet',
        default_value='False',
        description='Toggles between using Pybullet or Gazebo for physics simulation'
    )

    pybullet_launch = LaunchConfiguration('pybullet')

    # Starts with Gazebo if pybullet is false
    gazebo = add_launch_file("jetleg_bringup", 'jetleg_gazebo.launch.py', conditional=PythonExpression(['not ', pybullet_launch]))
    
    # Starts with Pybullet if pybullet is true
    pybullet_launch = add_launch_file("jetleg_bringup", 'jetleg_pybullet_ros.launch.py', conditional=pybullet_launch)

    # Mark actions for launch
    ld = LaunchDescription()

    ld.add_action(pybullet_arg)

    ld.add_action(gazebo)
    ld.add_action(pybullet_launch)

    return ld