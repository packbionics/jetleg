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

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class ImpedanceController:
    """Contains behavior for receiving and forwarding impedance control parameters."""

    def __init__(self):
        """Construct an ImpedanceController object."""
        # Create a ROS 2 node to interface with the rest of ROS 2
        self.node = rclpy.create_node("impedance_controller")

        # Maintain latest joint positions and velocities
        self.joint_state = None

        # Maintain latest impedance parameters
        self.stiffness = None
        self.damping = None
        self.equilibrium = None

        # Create a publisher that sends commands to the forward controller
        self.forward_pub = self.node.create_publisher(
            Float64MultiArray, "commands", qos_profile_system_default)

        # Create a timer to periodically send commands to the forward controller
        RATE = 0.01
        self.command_timer = self.node.create_timer(RATE, self.command_callback)

        # Create a subscriber to the joint states topic
        self.joint_state_sub = self.node.create_subscription(
            JointState, "joint_states", self.joint_state_callback, qos_profile_system_default)

        # Create a subscriber that receives requests to change impedance parameters
        self.impedance_service = self.node.create_subscription(
            Float64MultiArray,
            "impedance_params",
            self.update_impedance_callback,
            qos_profile_system_default
        )

    def update_impedance_callback(self, msg: Float64MultiArray):
        """Update the current impedance parameters based on latest received message."""
        # Number of state variables per parameter
        stride = msg.layout.dim[1].stride

        # Extract values for each of the three parameters per state variable
        self.stiffness = np.array(msg.data[0: stride])
        self.damping = np.array(msg.data[stride: 2 * stride])
        self.equilibrium = np.array(msg.data[2 * stride: 3 * stride])

    def command_callback(self):
        """Update the command inputs to the system."""
        print("Command callback has been called")
        # Make sure there is an internal representation of the joint state
        if self.joint_state is None:
            return
        if self.stiffness is None or self.damping is None or self.equilibrium is None:
            return

        # TODO: Describe relevant joints with parameters rather than hardcode values
        mapping = {"knee_joint": 0, "ankle_joint": 1}

        x = np.zeros((2), dtype=np.float64)
        x_dot = np.zeros((2), dtype=np.float64)

        for i, joint_name in enumerate(self.joint_state.name):
            if joint_name in mapping:
                idx = mapping[joint_name]

                x[idx] = self.joint_state.position[i]
                x_dot[idx] = self.joint_state.velocity[i]

        # Compute the input signal
        signal = ImpedanceController.compute_command(
            x,
            x_dot,
            self.stiffness,
            self.damping,
            self.equilibrium
        )

        # Populate message with command values
        msg = Float64MultiArray()
        msg.data.fromlist(signal.tolist())

        self.forward_pub.publish(msg)

    def joint_state_callback(self, msg: JointState):
        """Update the internal representation of the joint state."""
        self.joint_state = msg

    def compute_command(x: np.array, x_dot: np.array,
                        k: np.array, d: np.array, eq: np.array) -> np.array:
        return k * (eq - x) - d * x_dot

def main():

    # Initialize ROS 2
    rclpy.init()

    # Create an instance of the controller
    controller = ImpedanceController()

    # Allow the process to cleanly exit after KeyboardInterrupt
    try:
        # Execute the node tasks
        rclpy.spin(controller.node)
    except KeyboardInterrupt:
        pass