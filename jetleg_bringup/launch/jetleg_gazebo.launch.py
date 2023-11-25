from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

  ld = LaunchDescription()

  # Specify world environment in Gazebo
  world_path = PathJoinSubstitution([
    FindPackageShare("jetleg_bringup"), 
    'world/staircase_obstacle.world'
  ])
  world_arg = DeclareLaunchArgument(
    "world",
    default_value=world_path
  )
  ld.add_action(world_arg)

  # Toggle Gazebo GUI client
  gui_arg = DeclareLaunchArgument(
     "gui",
     default_value="false"
  )

  ld.add_action(gui_arg)

  # Add Gazebo to launch step
  gazebo_launch_path = PathJoinSubstitution([
    FindPackageShare("gazebo_ros"),
    "launch/gazebo.launch.py"
  ])
  gazebo_ros = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(gazebo_launch_path)
  )
  ld.add_action(gazebo_ros)

  # Load Jetleg into the world
  spawn_jetleg = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([FindPackageShare("jetleg_bringup"), 'launch/spawn_jetleg.launch.py'])
    )
  )
  ld.add_action(spawn_jetleg)

  return ld