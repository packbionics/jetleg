from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration

from launch_ros.actions import Node
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

  model_path = PathJoinSubstitution([
    FindPackageShare("jetleg_moveit_config"),
    "config", "jetleg_wheeled_testrig.urdf.xacro"
  ])

  model = LaunchConfiguration("model")

  ld = LaunchDescription()

  # Specify world environment in Gazebo
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

  # Specify robot path
  model_arg = DeclareLaunchArgument(
    "model",
    default_value=model_path
  )
  ld.add_action(model_arg)

  # Add Gazebo to launch step
  gazebo_ros = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(gazebo_launch_path)
  )
  ld.add_action(gazebo_ros)

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