"""
Adds a robot_state_publisher to manage robot configuration
"""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    # synchronizes ROS clock with simulation clock if true
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # specifies the XACRO/URDF file to utilize
    model = LaunchConfiguration('model')

    robot_urdf = Command(['xacro', ' ', model])

    rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 
                         'robot_description': robot_urdf}],
    )

    ld = LaunchDescription()
    ld.add_action(rsp)

    return ld