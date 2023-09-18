
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    teleop_node = Node(
        package='jetleg_control',
        executable='jetleg_teleop_key.py',
        # create a new terminal window for receiving keys, xterm must be installed first
        prefix = 'xterm -e', 
        output='screen'
    )
        
    ld = LaunchDescription()

    ld.add_action(teleop_node)
    
    return ld
