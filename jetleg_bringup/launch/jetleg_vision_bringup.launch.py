from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    jetleg_vision_share = FindPackageShare('jetleg_vision')

    zed2i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([jetleg_vision_share, '/launch', '/' + 'jetleg_zed2i.launch.py']),
    )
    
    pointcloud_proc = Node(
        package='jetleg_vision',
        executable='pointcloud_proc',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(zed2i_launch)
    ld.add_action(pointcloud_proc)

    return ld
