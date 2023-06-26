from launch import LaunchDescription
from launch.substitutions import PythonExpression

from packbionics_launch_utils.launch_utils import add_launch_argument, add_launch_file


def generate_launch_description():

    use_pybullet_env, use_pybullet_env_arg = add_launch_argument(
        'use_pybullet_env', 
        default='True', 
        description="Toggles between using Pybullet or Gazebo for physics simulation"
    )

    gazebo = add_launch_file("jetleg_bringup", 'jetleg_gazebo.launch.py', conditional=PythonExpression(['not ', use_pybullet_env]))
    pybullet = add_launch_file("jetleg_bringup", 'jetleg_pybullet_ros.launch.py', conditional=use_pybullet_env)

    ld = LaunchDescription()

    ld.add_action(use_pybullet_env_arg)

    ld.add_action(gazebo)
    ld.add_action(pybullet)

    return ld