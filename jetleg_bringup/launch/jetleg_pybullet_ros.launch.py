from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def add_pybullet_sim(ld: LaunchDescription):

    # Retrieve simulation launch file
    pybullet_ros_share = FindPackageShare('pybullet_ros')
    bringup_robot_example_path = PathJoinSubstitution([pybullet_ros_share, 'launch/bringup_robot_example.launch.py'])

    # assign launch description to be executed
    bringup_robot_example = IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup_robot_example_path))
    ld.add_action(bringup_robot_example)

def add_rsp(ld: LaunchDescription):

    # Adds a robot_state_publisher to manage robot configuration
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('jetleg_bringup'),
                    'launch/rsp.launch.py'
                ]
            )
        )
    )

    ld.add_action(rsp)
    

def add_visualization(ld: LaunchDescription):

    jetleg_description_dir = FindPackageShare('jetleg_description')
    default_rviz_config_path = PathJoinSubstitution([jetleg_description_dir, 'rviz/urdf.rviz'])

    rviz_arg = DeclareLaunchArgument(
        'rvizconfig', 
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )

    ld.add_action(rviz_arg)

    # Adds an Rviz2 visualization

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('jetleg_bringup'),
                    'launch/rviz.launch.py'
                ]
            )
        )
    )

    ld.add_action(rviz)

def generate_launch_description():

    ld = LaunchDescription()

    model_arg = DeclareLaunchArgument(
        'model', 
        default_value=PathJoinSubstitution([FindPackageShare('jetleg_description'), 'urdf/jetleg_testrig.xacro']),
        description='Robot model to be loaded into simulation'
    )

    ld.add_action(model_arg)

    add_pybullet_sim(ld=ld)
    add_rsp(ld=ld)
    add_visualization(ld=ld)

    return ld
