from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("jetleg_moveit_config"),
                "launch/move_group.launch.py"
            ])
        )
    )
    ld.add_action(move_group)

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("jetleg_moveit_config"),
                "launch/moveit_rviz.launch.py"
            ])
        )
    )
    ld.add_action(moveit_rviz)

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("jetleg_moveit_config"),
                "launch/rsp.launch.py"
            ])
        )
    )
    ld.add_action(rsp)

    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("jetleg_moveit_config"),
                "launch/spawn_controllers.launch.py"
            ])
        )
    )
    # ld.add_action(spawn_controllers)

    static_virtual_joint_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("jetleg_moveit_config"),
                "launch/static_virtual_joint_tfs.launch.py"
            ])
        )
    )
    ld.add_action(static_virtual_joint_tfs)


    return ld
