from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # Start up impedance controller
    hip_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hip_controller"]
    )
    ld.add_action(hip_controller)

    # Start teleop process
    teleop_controller = Node(
        package="rqt_joint_trajectory_controller",
        executable="rqt_joint_trajectory_controller"
    )
    ld.add_action(teleop_controller)

    return ld