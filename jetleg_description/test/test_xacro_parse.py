# Copyright (c) 2023 Pack Bionics
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

import xacro
import os

from xml.parsers.expat import ExpatError
from ament_index_python.packages import get_package_share_directory


def test_parse_jetleg_wheeled_testrig():

    # Compute the absolute path to the robot description
    jetleg_description_share = get_package_share_directory("jetleg_description")
    robot_description_path = os.path.join(
        jetleg_description_share,
        "urdf/jetleg_wheeled_testrig.xacro"
    )

    # Attempt to parse the file as a XACRO file
    try:
        _ = xacro.process_file(robot_description_path)
        assert True
    except ExpatError:
        assert False, f"{robot_description_path} is a mal-formed xacro file"


def test_parse_jetleg():

    # Compute the absolute path to the robot description
    jetleg_description_share = get_package_share_directory("jetleg_description")
    robot_description_path = os.path.join(
        jetleg_description_share,
        "urdf/jetleg.xacro"
    )

    # Attempt to parse the file as a XACRO file
    try:
        _ = xacro.process_file(robot_description_path)
        assert True
    except ExpatError:
        assert False, f"{robot_description_path} is a mal-formed xacro file"


def test_parse_jetleg_testrig():

    # Compute the absolute path to the robot description
    jetleg_description_share = get_package_share_directory("jetleg_description")
    robot_description_path = os.path.join(
        jetleg_description_share,
        "urdf/jetleg_testrig.xacro"
    )

    # Attempt to parse the file as a XACRO file
    try:
        _ = xacro.process_file(robot_description_path)
        assert True
    except ExpatError:
        assert False, f"{robot_description_path} is a mal-formed xacro file"


def test_parse_jetleg_testrig_vision():

    # Compute the absolute path to the robot description
    jetleg_description_share = get_package_share_directory("jetleg_description")
    robot_description_path = os.path.join(
        jetleg_description_share,
        "urdf/jetleg_testrig_vision.xacro"
    )

    # Attempt to parse the file as a XACRO file
    try:
        _ = xacro.process_file(robot_description_path)
        assert True
    except ExpatError:
        assert False, f"{robot_description_path} is a mal-formed xacro file"
