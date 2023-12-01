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
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from packbionics_interfaces.srv import UpdateImpedance

from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class ImpedanceParams:
    stiffness: float
    damping: float
    equilibrium: float


def compute_command(position: float, velocity: float, params: ImpedanceParams) -> float:
    error = params.equilibrium - position
    return params.stiffness * error - params.damping * velocity


class ImpedanceParamsManager:

    def __init__(self, node: Node, n_joints: int):

        # Stores the node which shall manage the impedance update service
        self.node = node

        # Stores the number of controllable joints
        self.n_joints = n_joints

        # Stores the parameters used for each joint to control system
        self.impedance_params = None

    def update_impedance_callback(self, req: UpdateImpedance.Request,
                                  resp: UpdateImpedance.Response):

        # Acknowledge request
        self.node.get_logger().info("Request received")
        self.node.get_logger().info(repr(req))

        # Ensure the request is properly formatted
        if len(req.stiffness) != len(req.damping) or len(req.stiffness) != len(req.equilibrium):
            return

        # Ensure number of parameters corresponds to number of controllable joints
        if len(req.stiffness) != self.n_joints:
            return

        # Use values given in the request
        if self.impedance_params is None:
            self.impedance_params = list()

            for i in range(len(req.stiffness)):
                new_params = ImpedanceParams(
                    req.stiffness[i], req.damping[i], req.equilibrium[i])
                self.impedance_params.append(new_params)
        else:

            for i in range(len(req.stiffness)):
                self.impedance_params[i].stiffness = req.stiffness[i]
                self.impedance_params[i].damping = req.damping[i]
                self.impedance_params[i].equilibrium = req.equilibrium[i]

        self.node.get_logger().info("Request handled")

        return resp


class JointStateManager:
    def __init__(self, msg: JointState):
        self.msg = msg

    def joint_state_callback(self, msg: JointState):
        self.msg = msg

    def get_joint_state(self, name: str) -> Tuple[float, float]:
        joint_idx = 0
        while joint_idx < len(self.msg.name) and self.msg.name[joint_idx] != name:
            joint_idx += 1

        if joint_idx == len(self.msg.name):
            return None
        return self.msg.position[joint_idx], self.msg.velocity[joint_idx]


class CmdPubManager:
    def __init__(self, joints: List[str], publisher: Publisher,
                 ipm: ImpedanceParamsManager, jsm: JointStateManager):

        # Publisher to the commands topic
        self.publisher = publisher

        # Maintains current list of impedance parameters
        self.ipm = ipm

        # Maintains the current joint state
        self.jsm = jsm

        # Maintains list of controllable joints
        self.joints = joints

        # Maintains the most recent command to sent to the low-level controller
        self.msg = Float64MultiArray()

        self.msg.data.extend([0.0] * len(self.joints))

    def command_callback(self):

        # Wait to receive initial list of parameters before proceeding
        if self.ipm.impedance_params is None:
            return

        # Populate message with command values
        for joint_idx in range(len(self.joints)):
            joint_state = self.jsm.get_joint_state(self.joints[joint_idx])
            if joint_state is None:
                continue

            signal = compute_command(
                joint_state[0], joint_state[1], self.ipm.impedance_params[joint_idx])
            self.msg.data[joint_idx] = signal

        self.publisher.publish(self.msg)


def main():

    # Initialize ROS 2
    rclpy.init()

    # Create a ROS 2 node to interface with the rest of ROS 2
    node = rclpy.create_node("impedance_controller")

    # Access the controllable joints
    node.declare_parameter(
        "joints",
        [''],
        ParameterDescriptor(
            name="joints",
            type=ParameterType.PARAMETER_STRING_ARRAY
        ))
    joints = node.get_parameter(
        "joints").get_parameter_value().string_array_value

    ipm = ImpedanceParamsManager(node, len(joints))
    jsm = JointStateManager(None)

    # Create a publisher that sends commands to the forward controller
    forward_pub = node.create_publisher(
        Float64MultiArray, "commands", qos_profile_system_default)
    cpm = CmdPubManager(joints, forward_pub, ipm, jsm)

    # Create a subscriber to the joint states topic
    _ = node.create_subscription(
        JointState, "joint_states", jsm.joint_state_callback, qos_profile_system_default)

    # Create timer to periodically send commands to the forward controller
    RATE = 0.01
    _ = node.create_timer(RATE, cpm.command_callback)

    # Create a service that receives requests to change impedance parameters
    _ = node.create_service(
        UpdateImpedance, "update_impedance", ipm.update_impedance_callback)

    # Execute the node tasks
    rclpy.spin(node)

    # Release ROS 2 resources
    rclpy.shutdown()


if __name__ == "__main__":
    main()
