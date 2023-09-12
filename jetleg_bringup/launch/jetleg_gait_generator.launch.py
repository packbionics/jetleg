from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution

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
    model_path = PathJoinSubstitution([
        FindPackageShare('jetleg_description'), 
        'urdf/', 
        'jetleg_testrig_vision.xacro'
    ])

    _, model_arg = add_launch_argument('model', default=model_path, description='Robot model loaded into simulation')

    gait_generator_node = Node(
        package='jetleg_control',
        executable='jetleg_gait_generator.py',
        output='screen'
    )

    teleop_node = Node(
        package='jetleg_control',
        executable='forwarder.py',
        output='screen'
    )
    
    sim_env = add_launch_file('jetleg_bringup', 'jetleg_bringup.launch.py')
    
    ld = LaunchDescription()

    ld.add_action(model_arg)
    ld.add_action(gait_generator_node)
    # ld.add_action(teleop_node)
    ld.add_action(sim_env)

    return ld