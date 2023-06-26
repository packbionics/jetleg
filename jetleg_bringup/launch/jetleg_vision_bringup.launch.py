from launch_ros.actions import Node
from launch import LaunchDescription

from packbionics_launch_utils.launch_utils import add_launch_file

def generate_launch_description():

    zed2i_launch = add_launch_file('jetleg_vision', 'jetleg_zed2i.launch.py')
    
    pointcloud_proc = Node(
        package='jetleg_vision',
        executable='pointcloud_proc',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(zed2i_launch)
    ld.add_action(pointcloud_proc)

    return ld
