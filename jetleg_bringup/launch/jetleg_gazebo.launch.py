# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Launches Gazebo and spawns a jetleg model
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    gazebo_ros_path = get_package_share_directory("gazebo_ros")
    jetleg_description_path = get_package_share_directory("jetleg_description")

    xacro_path = os.path.join(jetleg_description_path, 'urdf/jetleg_wheeled_testrig.xacro')

    urdf_model = xacro.process_file(xacro_path)
    urdf_model = urdf_model.toxml()

    ld = LaunchDescription()
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gazebo_ros_path, '/launch/gazebo.launch.py']),
             )

    # arguments to pass when spawning model
    model_description = ['-entity', 'jetleg_wheeled_testrig', '-topic', '/robot_description']
    model_pos = ['-x', '0.0', '-y', '0.0', '-z', '1.0']
    model_orientation = ['-R', '0.0', '-P', '0.0', '-Y', '0.0']

    spawn_params = model_description + model_pos + model_orientation

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=spawn_params,
                        output='screen')
    
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_model}]
    )
    
    ld.add_action(gazebo)
    ld.add_action(rsp)
    ld.add_action(spawn_entity)

    return ld
