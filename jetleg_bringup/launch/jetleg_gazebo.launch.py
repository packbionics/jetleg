"""
Launches Gazebo and spawns a jetleg model
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

def generate_launch_description():

    gazebo_ros_path = FindPackageShare("gazebo_ros")
    jetleg_description_path = get_package_share_directory("jetleg_description")
    jetleg_bringup_path = FindPackageShare("jetleg_bringup")

    xacro_path = os.path.join(jetleg_description_path, 'urdf/jetleg_testrig_vision.xacro')

    urdf_model = xacro.parse(open(xacro_path))
    xacro.process_doc(urdf_model)

    controller_list = ['leg_controller', 'leg_intact_controller', 'joint_state_broadcaster']

    # arguments to pass when spawning model
    model_description = ['-entity', 'jetleg_wheeled_testrig', '-topic', '/robot_description']
    model_pos = ['-x', '0.0', '-y', '0.0', '-z', '1.0']
    model_orientation = ['-R', '0.0', '-P', '0.0', '-Y', '0.0']

    spawn_params = model_description + model_pos + model_orientation

    rviz_config = LaunchConfiguration("rviz_config")
    rviz_toggle = LaunchConfiguration("rviz")
    gui_toggle = LaunchConfiguration("gui")

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config", 
        default_value=PathJoinSubstitution([jetleg_bringup_path, 'config/jetleg_gazebo.rviz']),
        description="Specifies the absolute path of the RVIZ config used for default RVIZ visualization"
    )
    rviz_toggle_arg = DeclareLaunchArgument(
        "rviz", 
        default_value="True",
        description="Toggles the startup of the RVIZ visualization"
    )
    gui_toggle_arg = DeclareLaunchArgument(
        "gui", 
        default_value="false",
        description="Toggles the Gazebo client"
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gazebo_ros_path, '/launch/gazebo.launch.py']),
                launch_arguments={
                    'gui': gui_toggle
                }.items()
    )

    spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        arguments=spawn_params,
                        output='screen'
    )
    
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_model.toxml()}]
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
