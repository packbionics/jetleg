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

    use_pybullet_env_arg = DeclareLaunchArgument(
        name='use_pybullet_env',
        default_value='True',
        description='Toggles between using Pybullet or Gazebo for physics simulation'
    )

    use_pybullet_env = LaunchConfiguration('use_pybullet_env')

    gazebo = add_launch_file("jetleg_bringup", 'jetleg_gazebo.launch.py', conditional=PythonExpression(['not ', use_pybullet_env]))
    pybullet = add_launch_file("jetleg_bringup", 'jetleg_pybullet_ros.launch.py', conditional=use_pybullet_env)

    ld = LaunchDescription()

    ld.add_action(use_pybullet_env_arg)

    ld.add_action(gazebo)
    ld.add_action(pybullet)

    return ld