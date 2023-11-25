from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    jetleg_bringup_share = FindPackageShare("jetleg_bringup")

    # Specify robot path
    model_path = PathJoinSubstitution([
        FindPackageShare("jetleg_moveit_config"),
        "config", "jetleg_wheeled_testrig.urdf.xacro"
    ])
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=model_path
    )
    ld.add_action(model_arg)

    # Begin publishing robot state
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("jetleg_bringup"),
                "launch/rsp.launch.py"
            ])
        )
    )
    ld.add_action(rsp)

    # Launch simulation / physical system
    jetleg_sim_name = "jetleg_gazebo.launch.py"

    jetleg_launch_path = PathJoinSubstitution([
        jetleg_bringup_share,
        'launch',
        jetleg_sim_name
    ])
    jetleg_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(jetleg_launch_path),
    )
    ld.add_action(jetleg_gazebo_launch)

    # Spawn ros2_controllers
    spawn_controls = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetleg_bringup'), 'launch/spawn_controllers.launch.py'])
        )
    )
    ld.add_action(spawn_controls)

    # Specify RVIZ config
    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig', 
        default_value=PathJoinSubstitution([jetleg_bringup_share, 'config/jetleg_gazebo.rviz']),
    )
    ld.add_action(rviz_config_arg)

    # Optionally show Rviz2 Display
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetleg_bringup'), 'launch/rviz.launch.py'])
        )
    )
    ld.add_action(rviz)

    return ld