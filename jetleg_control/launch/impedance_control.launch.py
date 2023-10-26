from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # Start up impedance controller
    impedance_params_file = PathJoinSubstitution([
        FindPackageShare("jetleg_control"),
        "config", "impedance_control_params.yaml"
    ])

    impedance_controller = Node(
        package="jetleg_control",
        executable="impedance_controller.py",
        parameters=[impedance_params_file]
    )
    ld.add_action(impedance_controller)

    return ld