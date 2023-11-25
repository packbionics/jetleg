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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    jetleg_bringup_share = FindPackageShare("jetleg_bringup")

    # Specify robot path
    model_path = PathJoinSubstitution([
        FindPackageShare("jetleg_moveit_config"),
        "config", "jetleg_wheeled_testrig.urdf.xacro"
    ])
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=model_path
    )
    ld.add_action(model_arg)

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

    # Launch simulation / physical system
    jetleg_sim_name = "jetleg_gazebo.launch.py"

    jetleg_launch_path = PathJoinSubstitution([
        jetleg_bringup_share,
        'launch',
        jetleg_sim_name
    ])
    jetleg_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(jetleg_launch_path),
    )
    ld.add_action(jetleg_gazebo_launch)

    # Spawn ros2_controllers
    spawn_controls = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('jetleg_bringup'),
                'launch/spawn_controllers.launch.py'
            ])
        )
    )
    ld.add_action(spawn_controls)

    # Specify RVIZ config
    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution([jetleg_bringup_share, 'config/jetleg_gazebo.rviz']),
    )
    ld.add_action(rviz_config_arg)

    # Optionally show Rviz2 Display
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetleg_bringup'), 'launch/rviz.launch.py'])
        )
    )
    ld.add_action(rviz)

    return ld
