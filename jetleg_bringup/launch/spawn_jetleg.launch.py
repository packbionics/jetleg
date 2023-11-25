from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

  ld = LaunchDescription()
  
  # Arguments describing how to spawn model
  model_description = ['-entity', 'jetleg_wheeled_testrig', '-topic', '/robot_description']
  model_pos = ['-x', '0.0', '-y', '0.0', '-z', '1.0']
  model_orientation = ['-R', '0.0', '-P', '0.0', '-Y', '0.0']

  spawn_params = model_description + model_pos + model_orientation

  # Spawn Jetleg in Gazebo
  spawn_entity = Node(
      package='gazebo_ros', 
      executable='spawn_entity.py',
      arguments=spawn_params,
      output='screen'
  )
  ld.add_action(spawn_entity)

  return ld