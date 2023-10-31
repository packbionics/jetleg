from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # Start up impedance controller
    impedance_params_file = PathJoinSubstitution([
        FindPackageShare("jetleg_control"),
        "config", "impedance_params.yaml"
    ])

    # Modify default parameter with argument passed during launch
    idx_arg = DeclareLaunchArgument("idx", default_value=TextSubstitution(text="0"))
    ld.add_action(idx_arg)

    idx = LaunchConfiguration("idx")

    impedance_updater = Node(
        package="jetleg_control",
        executable="send_impedance_update.py",
        parameters=[impedance_params_file] + [{"idx": idx}],
        remappings=[("commands", "jetleg_controller/commands")]
    )
    ld.add_action(impedance_updater)

    return ld