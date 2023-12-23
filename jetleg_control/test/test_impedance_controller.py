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


import numpy as np

import rclpy
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from jetleg_control.controllers.impedance_controller import ImpedanceController


def test_impedance_update():

    # Initialize the context to utilize ROS 2 features
    rclpy.init()

    # Create the controller to receive the request
    controller = ImpedanceController()

    # Create a node to communicate with the impedance controller node
    node = rclpy.create_node("test_impedance_update_node")

    # Create a client to send the service request
    pub = node.create_publisher(Float64MultiArray, "impedance_params", qos_profile_system_default)

    # Create a service request to test service behavior
    msg = Float64MultiArray()

    # Populate the request with necessary data
    msg.data.fromlist([0.0, 0.0])
    msg.data.fromlist([0.0, 0.0])
    msg.data.fromlist([0.0, 0.0])

    msg.layout.dim.append(MultiArrayDimension(size=2))
    msg.layout.dim.append(MultiArrayDimension(size=2))
    msg.layout.dim.append(MultiArrayDimension(size=2))

    # Send the message
    pub.publish(msg)
    rclpy.spin_once(controller.node)

    assert controller.stiffness is not None, "Stiffness should be a non-null value"
    assert controller.damping is not None, "Damping should be a non-null value"
    assert controller.equilibrium is not None, "Equilibrium should be a non-null value"

    assert np.all(controller.stiffness == np.array([0.0, 0.0])), "Expected stiffness: [0.0, 0.0]"
    assert np.all(controller.damping == np.array([0.0, 0.0])), "Expected damping: [0.0, 0.0]"
    assert np.all(controller.equilibrium == np.array([0.0, 0.0])), "Expected equilibrium: [0.0, 0.0]"

    # Release ROS 2 resources
    rclpy.shutdown()
