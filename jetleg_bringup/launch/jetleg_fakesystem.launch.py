from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    # define the robot to model and control

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([FindPackageShare('jetleg_description'), 'urdf/jetleg_testrig_vision.xacro']),
        description='Path to used XACRO / URDF file describing robot'
    )
    ld.add_action(model_arg)

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetleg_bringup'), 'launch/rsp.launch.py'])
        )
    )
    ld.add_action(rsp)

    # start the rviz simulation

    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution([FindPackageShare('jetleg_bringup'), 'config/jetleg_fakesystem.rviz']),
        description='Path to RVIZ 2 config file'
    )
    ld.add_action(rviz_config_arg)

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetleg_bringup'), 'launch/rviz.launch.py'])
        )
    )
    ld.add_action(rviz)

    # start up the control manager to interact with ros2 controllers

    model = LaunchConfiguration('model')
    robot_description = Command(['xacro', ' ', model])

    ros2_control_params = PathJoinSubstitution([FindPackageShare('jetleg_description'), 'ros2_control/ros2_controllers.yaml'])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, ros2_control_params]
    )
    ld.add_action(controller_manager)

    # spawn ros2 controllers to interact with robot

    activate_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetleg_bringup'), 'launch/spawn_controllers.launch.py'])
        )
    )
    ld.add_action(activate_controllers)

    return ld