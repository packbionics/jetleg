#!/usr/bin/python3

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
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from packbionics_interfaces.srv import UpdateImpedance

from dataclasses import dataclass


@dataclass
class ImpedanceParams:
    stiffness: float
    damping: float
    equilibrium: float


def compute_command(position: float, velocity: float, params: ImpedanceParams) -> float:
    error = params.equilibrium - position
    return params.stiffness * error - params.damping * velocity


class ImpedanceController:
    """Contains behavior for receiving and forwarding impedance control parameters."""

    def __init__(self):
        """Construct an ImpedanceController object."""
        # Create a ROS 2 node to interface with the rest of ROS 2
        self.node = rclpy.create_node("impedance_controller")

        # Access the controllable joints
        self.node.declare_parameter("joints", [''])
        self.joints = self.node.get_parameter(
            "joints").get_parameter_value().string_array_value

        self.impedance_params = list()
        for _ in self.joints:
            self.impedance_params.append(ImpedanceParams(0.0, 0.0, 0.0))

        self.joint_state = None

        # Create a publisher that sends commands to the forward controller
        self.forward_pub = self.node.create_publisher(
            Float64MultiArray, "commands", qos_profile_system_default)

        # Create timer to periodically send commands to the forward controller
        RATE = 0.01
        self.command_timer = self.node.create_timer(RATE, self.command_callback)

        # Create a subscriber to the joint states topic
        self.joint_state_sub = self.node.create_subscription(
            JointState, "joint_states", self.joint_state_callback, qos_profile_system_default)

        # Create an action service that receives requests to change impedance parameters
        self.impedance_service = self.node.create_service(
            UpdateImpedance, "update_impedance", self.update_impedance_callback)

    def update_impedance_callback(self, req: UpdateImpedance.Request,
                                  resp: UpdateImpedance.Response):
        # Acknowledge request
        self.node.get_logger().info("Request received")
        self.node.get_logger().info(repr(req))

        # Ensure the request is properly formatted
        if len(req.stiffness) != len(req.damping) or len(req.stiffness) != len(req.equilibrium):
            return

        # Ensure number of parameters corresponds to number of controllable joints
        if len(req.stiffness) != len(self.joints):
            return

        # Use values given in the request
        for i, _ in enumerate(self.impedance_params):
            self.impedance_params[i].stiffness = req.stiffness[i]
            self.impedance_params[i].damping = req.damping[i]
            self.impedance_params[i].equilibrium = req.equilibrium[i]

        self.node.get_logger().info("Request handled")

        return resp

    def command_callback(self):
        """Update the command inputs to the system."""
        # Make sure there is an internal representation of the joint state
        if self.joint_state is None:
            return

        # Populate message with command values
        for joint_idx, _ in enumerate(self.joints):
            signal = compute_command(
                self.joint_state.position[joint_idx],
                self.joint_state.position[joint_idx],
                self.impedance_params[joint_idx]
            )
            self.msg.data[joint_idx] = signal

        self.publisher.publish(self.msg)

    def joint_state_callback(self, msg: JointState):
        """Update the internal representation of the joint state."""
        self.joint_state = msg


def main():

    # Initialize ROS 2
    rclpy.init()

    # Create an instance of the controller
    controller = ImpedanceController()

    # Execute the node tasks
    controller.node.get_logger().info("Spinning ImpedanceController...")
    rclpy.spin(controller.node)

    # Release ROS 2 resources
    rclpy.shutdown()


if __name__ == "__main__":
    main()
