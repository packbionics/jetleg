from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # Start up impedance controller
    impedance_params_file = PathJoinSubstitution([
        FindPackageShare("jetleg_control"),
        "config", "impedance_params.yaml"
    ])

    impedance_updater = Node(
        package="jetleg_control",
        executable="send_impedance_update.py",
        parameters=[impedance_params_file],
        remappings=[("commands", "jetleg_controller/commands")]
    )
    ld.add_action(impedance_updater)

    return ld