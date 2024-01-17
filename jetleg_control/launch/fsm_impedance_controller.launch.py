# Copyright 2023 Pack Bionics
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


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # Find impedance controller launch path
    impedance_controller_path = PathJoinSubstitution([
        FindPackageShare("jetleg_control"),
        "launch", "impedance_control.launch.py"
    ])
    impedance_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(impedance_controller_path)
    )
    ld.add_action(impedance_controller)

    # Find ruled-based classifier launch path
    rule_based_classifier_path = PathJoinSubstitution([
        FindPackageShare("jetleg_control"),
        "launch", "rule_based_classifier.launch.py"
    ])
    rule_based_classifier = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rule_based_classifier_path)
    )
    ld.add_action(rule_based_classifier)

    return ld
