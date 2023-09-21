from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

  # Path to staircase description
  world_path = PathJoinSubstitution([
    FindPackageShare("jetleg_bringup"), 
    'world/staircase_obstacle.world'
  ])

  # Path to top-level gazebo launch file
  gazebo_launch_path = PathJoinSubstitution([
    FindPackageShare("gazebo_ros"),
    "launch/gazebo.launch.py"
  ])

  gazebo_ros = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(gazebo_launch_path)
  )

  world_arg = DeclareLaunchArgument(
    "world",
    default_value=world_path
  )

  gui_arg = DeclareLaunchArgument(
     "gui",
     default_value="false"
  )

  spawn_controls = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('jetleg_bringup'), 'launch/spawn_controllers.launch.py'])
    )
  )

  rviz = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('jetleg_bringup'), 'launch/rviz.launch.py'])
    )
  )

  spawn_jetleg = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([FindPackageShare("jetleg_bringup"), 'launch/spawn_jetleg.launch.py'])
    )
  )

  ld = LaunchDescription()

  # Specify world environment in Gazebo
  ld.add_action(world_arg)

  # Toggle Gazebo GUI client
  ld.add_action(gui_arg)

  # Add Gazebo to launch step
  ld.add_action(gazebo_ros)

  # # Spawn ros2_controllers
  # ld.add_action(spawn_controls)

  # Optionally show Rviz2 Display
  ld.add_action(rviz)

  # Load Jetleg into the world
  ld.add_action(spawn_jetleg)  

  return ld