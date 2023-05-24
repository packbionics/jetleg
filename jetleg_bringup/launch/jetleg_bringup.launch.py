from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    jetleg_bringup_path = FindPackageShare("jetleg_bringup")
    use_pybullet_env = LaunchConfiguration('use_pybullet_env')

    use_pybullet_env_arg = DeclareLaunchArgument(
        "use_pybullet_env", 
        default_value='False',
        description="Toggles between using Pybullet or Gazebo for physics simulation")

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([jetleg_bringup_path, '/launch', '/jetleg_gazebo.launch.py']),
                condition=IfCondition(
                    PythonExpression(
                        ['not ', use_pybullet_env]
                    )
                )
    )

    pybullet = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([jetleg_bringup_path, '/launch', '/jetleg_pybullet_ros.launch.py']),
                condition=IfCondition(use_pybullet_env)
    )

    ld = LaunchDescription()

    ld.add_action(use_pybullet_env_arg)

    ld.add_action(gazebo)
    ld.add_action(pybullet)

    return ld