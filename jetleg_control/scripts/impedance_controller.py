#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from packbionics_interfaces.srv import UpdateImpedance

from dataclasses import dataclass
from typing import List

@dataclass
class ImpedanceParams:
    stiffness: float
    damping: float
    equilibrium: float


def compute_torque(position: float, velocity: float, params: ImpedanceParams) -> float:
    error = params.equilibrium - position
    return params.stiffness * error - params.damping * velocity

class ImpedanceParamsManager:

    def __init__(self, impedance_params: List[UpdateImpedance]):

        # Stores the parameters used for each joint to control system
        self.impedance_params = impedance_params

    def update_impedance_callback(self, req: UpdateImpedance.Request, resp: UpdateImpedance.Response):
        
        # Reset the impedance parameters on each update
        self.impedance_params.clear()

        # Use values given in the request
        for i in range(len(req.stiffness)):
            self.impedance_params.append(req.stiffness[i], req.damping[i], req.equilibrium[i])

class JointStateManager:
    def __init__(self, msg: JointState):
        self.msg = msg

    def joint_state_callback(self, msg: JointState):
        self.msg = msg

class CmdPubManager:
    def __init__(self, publisher: Publisher, ipm: ImpedanceParamsManager, jsm: JointStateManager):
        self.publisher = publisher
        self.ipm = ipm
        self.jsm = jsm

    def command_callback(self):
        pass


def main():

    # Initialize ROS 2
    rclpy.init()

    # Create a ROS 2 node to interface with the rest of ROS 2
    node = rclpy.create_node("impedance_controller")

    ipm = ImpedanceParamsManager(None)
    jsm = JointStateManager(None)

    # Create a publisher that sends commands to the forward controller
    forward_pub = node.create_publisher(Float64MultiArray, "command", qos_profile_system_default)
    cpm = CmdPubManager(forward_pub, ipm, jsm)

    # Create a subscriber to the joint states topic
    joint_state_sub = node.create_subscription(JointState, "joint_states", jsm.joint_state_callback)

    # Create timer to periodically send commands to the forward controller
    RATE = 0.01
    timer = node.create_timer(RATE, cpm.command_callback)

    # Create a service that receives requests to change impedance parameters
    update_impedance_service = node.create_service(UpdateImpedance, "update_impedance", ipm.update_impedance_callback)

    # Execute the node tasks
    rclpy.spin(node)

    # Release ROS 2 resources
    rclpy.shutdown()


if __name__ == "__main__":
    main()