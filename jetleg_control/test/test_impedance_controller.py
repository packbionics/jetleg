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


import rclpy

from packbionics_interfaces.srv import UpdateImpedance

from jetleg_control.controllers.impedance_controller import ImpedanceController


def test_impedance_update():

    # Initialize the context to utilize ROS 2 features
    rclpy.init()

    # Create the controller to receive the request
    controller = ImpedanceController()

    # Create a node to communicate with the impedance controller node
    node = rclpy.create_node("test_impedance_update_node")

    # Create a client to send the service request
    client = node.create_client(UpdateImpedance, "update_impedance")

    # Create a service request to test service behavior
    request = UpdateImpedance.Request()

    # Populate the request with necessary data
    request.stiffness.fromlist([0.0, 0.0])
    request.damping.fromlist([0.0, 0.0])
    request.equilibrium.fromlist([0.0, 0.0])

    # Send the request asynchronously
    future = client.call_async(request)

    rclpy.spin_once(controller.node)

    # Wait for up to 1 seconds until a response is received
    rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)

    # Check if a response was received
    assert future.done()

    # Release ROS 2 resources
    rclpy.shutdown()

    pass
