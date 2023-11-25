# Copyright (c) 2023 Pack Bionics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following
# conditions:
#
# The above copyright notice and this permission notice shall be included in all copies
# or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
# CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
# OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

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
            PathJoinSubstitution([
                FindPackageShare("jetleg_bringup"),
                'launch/spawn_jetleg.launch.py'
            ])
        )
    )
    ld.add_action(spawn_jetleg)

    return ld
