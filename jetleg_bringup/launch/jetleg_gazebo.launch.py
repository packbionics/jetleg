from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def add_rviz(ld: LaunchDescription):
    jetleg_bringup_path = FindPackageShare("jetleg_bringup")

    # Specify RVIZ config
    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig', 
        default_value=PathJoinSubstitution([jetleg_bringup_path, 'config/jetleg_gazebo.rviz']),
        description='Specifies the absolute path of the RVIZ config used for default RVIZ visualization'
    )
    ld.add_action(rviz_config_arg)

    # Launch Rviz2
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetleg_bringup'), 'launch/rviz.launch.py'])
        ),
    )
    ld.add_action(rviz)

def add_rsp(ld: LaunchDescription):

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetleg_bringup'), 'launch/rsp.launch.py'])
        )
    )

    ld.add_action(rsp)

def add_gazebo_sim(ld: LaunchDescription):
    gui_toggle_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Toggles the Gazebo client'
    )
    ld.add_action(gui_toggle_arg)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch/gazebo.launch.py'])
        )
    )

    ld.add_action(gazebo)

    # arguments to pass when spawning model
    model_description = ['-entity', 'jetleg_wheeled_testrig', '-topic', '/robot_description']
    model_pos = ['-x', '0.0', '-y', '0.0', '-z', '1.0']
    model_orientation = ['-R', '0.0', '-P', '0.0', '-Y', '0.0']


    spawn_params = model_description + model_pos + model_orientation

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=spawn_params,
        output='screen'
    )
    ld.add_action(spawn_entity)


def spawn_controllers(ld: LaunchDescription):
    spawn_controls = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetleg_bringup'), 'launch/spawn_controllers.launch.py'])
        )
    )

    ld.add_action(spawn_controls)


def generate_launch_description():

    ld = LaunchDescription()

    jetleg_description_path = FindPackageShare("jetleg_description")
    xacro_path = PathJoinSubstitution([jetleg_description_path, 'urdf/jetleg_testrig_vision.xacro'])

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=xacro_path,
        description='Absolute path to xacro file'
    )

    staircase_obs_world = PathJoinSubstitution([
        FindPackageShare("gazebo_env"),
        'world/staircase_obstacle.world'
    ])

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=staircase_obs_world,
        description="Path to world file to load into Gazebo"
    )

    ld.add_action(world_arg)
    ld.add_action(model_arg)

    add_gazebo_sim(ld=ld)
    add_rviz(ld=ld)
    add_rsp(ld=ld)

    spawn_controllers(ld=ld)

    return ld
