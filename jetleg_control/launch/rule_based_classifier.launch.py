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

    rule_based_classifier = Node(
        package="jetleg_control",
        executable="rule_based_classifier.py",
        parameters=[impedance_params_file]
    )
    ld.add_action(rule_based_classifier)

    return ld