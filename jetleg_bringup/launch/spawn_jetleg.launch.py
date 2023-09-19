from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

  ld = LaunchDescription()

  # Publish robot description as topic
  
  jetleg_description_path = FindPackageShare("jetleg_description")
  xacro_path = PathJoinSubstitution([jetleg_description_path, 'urdf/jetleg_testrig_vision.xacro'])

  model_arg = DeclareLaunchArgument(
        'model',
        default_value=xacro_path,
        description='Absolute path to xacro file'
  )

  rsp = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          PathJoinSubstitution([FindPackageShare('jetleg_bringup'), 'launch/rsp.launch.py'])
      )
  )

  ld.add_action(model_arg)
  ld.add_action(rsp)

  # Spawn Jetleg in Gazebo

  # arguments to pass when spawning model
  model_description = ['-entity', 'jetleg_wheeled_testrig', '-topic', '/robot_description']
  model_pos = ['-x', '0.0', '-y', '0.0', '-z', '1.0']
  model_orientation = ['-R', '0.0', '-P', '0.0', '-Y', '0.0']


  spawn_params = model_description + model_pos + model_orientation

  spawn_entity = Node(
      package='gazebo_ros', 
      executable='spawn_entity.py',
      arguments=spawn_params,
      output='screen'
  )
  ld.add_action(spawn_entity)

  return ld