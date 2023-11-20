from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

  model_path = PathJoinSubstitution([
    FindPackageShare("jetleg_moveit_config"),
    "config", "jetleg_wheeled_testrig.urdf.xacro"
  ])

  model = LaunchConfiguration("model")

  ld = LaunchDescription()

  # Specify RVIZ config
  rviz_config_arg = DeclareLaunchArgument(
    'rvizconfig', 
    default_value=PathJoinSubstitution([FindPackageShare("jetleg_bringup"), 'config/jetleg_gazebo.rviz']),
  )
  ld.add_action(rviz_config_arg)
  
  # Specify robot path
  model_arg = DeclareLaunchArgument(
    "model",
    default_value=model_path
  )
  ld.add_action(model_arg)

  # Spawn ros2_controllers
  spawn_controls = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('jetleg_bringup'), 'launch/spawn_controllers.launch.py'])
    )
  )
  ld.add_action(spawn_controls)

  # Optionally show Rviz2 Display
  rviz = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('jetleg_bringup'), 'launch/rviz.launch.py'])
    )
  )
  ld.add_action(rviz)

  # Begin publishing robot state
  rsp = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          PathJoinSubstitution([
              FindPackageShare("jetleg_bringup"),
              "launch/rsp.launch.py"
          ])
      )
  )
  ld.add_action(rsp)

  robot_urdf = Command(['xacro', ' ', model])
  controller_manager_params = PathJoinSubstitution([
    FindPackageShare("jetleg_moveit_config"),
    "config", "ros2_controllers.yaml"
  ])

  controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
      {"robot_description": robot_urdf},
      controller_manager_params]
  )
  ld.add_action(controller_manager)

  return ld