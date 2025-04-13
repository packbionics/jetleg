# Copyright 2025 Pack Bionics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description():

    container = Node(
        name='image_container',
        package='rclcpp_components',
        executable='component_container',
        output='both',
    )

    load_composable_nodes = LoadComposableNodes(
        target_container='image_container',
        composable_node_descriptions=[
            ComposableNode(
                 package='pcl_ros',
                plugin='pcl_ros::PCDPublisher',
                name='PCDPublisher'
            ),
            # ComposableNode(
            #     package='image_tools',
            #     plugin='image_tools::ShowImage',
            #     name='showimage',
            #     remappings=[('/image', '/burgerimage')],
            #     parameters=[{'history': 'keep_last'}],
            #     extra_arguments=[{'use_intra_process_comms': True}]
            # ),
        ],
    )

    return LaunchDescription([container, load_composable_nodes])
