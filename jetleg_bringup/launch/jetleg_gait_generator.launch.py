from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    model_path = PathJoinSubstitution([
        FindPackageShare('jetleg_description'), 
        'urdf/', 
        'jetleg_testrig_vision.xacro'
    ])

    model_arg = DeclareLaunchArgument(
        'model', 
        default_value=model_path,
        description='Robot model loaded into simulation'
    )

    gait_generator_node = Node(
        package='jetleg_control',
        executable='jetleg_gait_generator.py',
        output='screen'
    )
    
    jetleg_bringup_share = FindPackageShare('jetleg_bringup')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([jetleg_bringup_share, '/launch', '/' + 'jetleg_bringup.launch.py']),
    )
    
    ld = LaunchDescription()

    ld.add_action(model_arg)
    ld.add_action(gait_generator_node)

    ld.add_action(sim_launch)

    return ld