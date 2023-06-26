from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from packbionics_launch_utils.launch_utils import add_launch_argument, add_launch_file

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
    ld.add_action(teleop_node)
    ld.add_action(sim_env)

    return ld