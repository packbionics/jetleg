
from launch_ros.actions import Node
from launch import LaunchDescription

from packbionics_launch_utils.launch_utils import add_launch_file

def generate_launch_description():
    
    teleop_node = Node(
        package='jetleg_control',
        executable='jetleg_teleop_key.py',
        # create a new terminal window for receiving keys, xterm must be installed first
        prefix = 'xterm -e', 
        output='screen'
    )
    
    pybullet_sim = add_launch_file('jetleg_bringup', 'jetleg_pybullet_ros.launch.py')
    
    ld = LaunchDescription()

    ld.add_action(teleop_node)
    ld.add_action(pybullet_sim)
    
    return ld
