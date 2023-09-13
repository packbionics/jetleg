
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    teleop_node = Node(
        package='jetleg_control',
        executable='jetleg_teleop_key.py',
        # create a new terminal window for receiving keys, xterm must be installed first
        prefix = 'xterm -e', 
        output='screen'
    )

    jetleg_bringup_share = FindPackageShare('jetleg_bringup')

    pybullet_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([jetleg_bringup_share, '/launch', '/' + 'jetleg_pybullet_ros.launch.py']),
    )
        
    ld = LaunchDescription()

    ld.add_action(teleop_node)
    ld.add_action(pybullet_sim)
    
    return ld
