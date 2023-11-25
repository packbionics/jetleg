from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

  model = LaunchConfiguration("model")

  ld = LaunchDescription()

  # Generate the urdf format from xacro file
  robot_urdf = Command(['xacro', ' ', model])
  controller_manager_params = PathJoinSubstitution([
    FindPackageShare("jetleg_moveit_config"),
    "config", "ros2_controllers.yaml"
  ])

  # Start up a standalone controller manager with the generated urdf content
  controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
      {"robot_description": robot_urdf},
      controller_manager_params]
  )
  ld.add_action(controller_manager)

  return ld