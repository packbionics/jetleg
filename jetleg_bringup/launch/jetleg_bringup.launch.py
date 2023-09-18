from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    jetleg_bringup_share = FindPackageShare("jetleg_bringup")

    # Specify RVIZ config
    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig', 
        default_value=PathJoinSubstitution([jetleg_bringup_share, 'config/jetleg_gazebo.rviz']),
    )
    ld.add_action(rviz_config_arg)

    # Starts with Gazebo
    jetleg_sim_name = "jetleg_gazebo.launch.py"

    jetleg_launch_path = PathJoinSubstitution([
        jetleg_bringup_share,
        'launch',
        jetleg_sim_name
    ])

    # Launch Gazebo simulation
    jetleg_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(jetleg_launch_path),
    )
    ld.add_action(jetleg_gazebo_launch)

    # Launch motion planning
    gait_generator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([jetleg_bringup_share, 'launch/jetleg_gait_generator.launch.py'])
        )
    )
    ld.add_action(gait_generator)

    # Launch vision subsystem
    jetleg_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([jetleg_bringup_share, 'launch/jetleg_vision_pipeline.launch.py'])
        )
    )
    ld.add_action(jetleg_vision)

    return ld