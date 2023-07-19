"""
Launches Gazebo and spawns a jetleg model
"""

from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, PythonExpression, Command, LaunchConfiguration


def add_launch_argument(name: str, default: str, description: str) -> tuple:
    launch_config = LaunchConfiguration(name)

    launch_arg = DeclareLaunchArgument(
        name, 
        default_value=default,
        description=description
    )
    
    return launch_config, launch_arg

def add_launch_file(package_name: str, launch_name: str, conditional=None):
    package = FindPackageShare(package_name)

    launch_condition = None
    if conditional is not None:
        launch_condition = IfCondition(
            conditional
        )

    launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([package, '/launch', '/' + launch_name]),
        condition=launch_condition
    )

    return launch_file

def generate_launch_description():

    jetleg_description_path = FindPackageShare("jetleg_description")
    jetleg_bringup_path = FindPackageShare("jetleg_bringup")

    xacro_path = PathJoinSubstitution([jetleg_description_path, 'urdf/jetleg_testrig_vision.xacro'])

    controller_list = ['leg_controller', 'leg_intact_controller', 'joint_state_broadcaster']

    # arguments to pass when spawning model
    model_description = ['-entity', 'jetleg_wheeled_testrig', '-topic', '/robot_description']
    model_pos = ['-x', '0.0', '-y', '0.0', '-z', '1.0']
    model_orientation = ['-R', '0.0', '-P', '0.0', '-Y', '0.0']

    spawn_params = model_description + model_pos + model_orientation

    rviz_config, rviz_config_arg = add_launch_argument(
        'rviz_config',
        default=PathJoinSubstitution([jetleg_bringup_path, 'config/jetleg_gazebo.rviz']),
        description='Specifies the absolute path of the RVIZ config used for default RVIZ visualization'
    )
    rviz_toggle, rviz_toggle_arg = add_launch_argument(
        'rviz',
        default='True',
        description='Toggles the startup of the RVIZ visualization'
    )
    _, gui_toggle_arg = add_launch_argument(
        'gui',
        default='false',
        description='Toggles the Gazebo client'
    )

    model, model_arg = add_launch_argument(
        'model',
        default=xacro_path,
        description='Absolute path to xacro file'
    )

    robot_urdf = Command(['xacro', ' ', model])

    gazebo = add_launch_file('gazebo_ros', 'gazebo.launch.py')

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=spawn_params,
        output='screen'
    )
    
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(
            PythonExpression(
                [rviz_toggle]
            )
        ),
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(rviz_config_arg)
    ld.add_action(rviz_toggle_arg)
    ld.add_action(gui_toggle_arg)
    ld.add_action(model_arg)

    ld.add_action(gazebo)
    ld.add_action(rsp)
    ld.add_action(spawn_entity)
    ld.add_action(rviz)

    for controller in controller_list:
        ld.add_action(Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=[controller]
        ))

    return ld
