#!/usr/bin/python3

import rclpy
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


def compute_torque(position: float, velocity: float, params: ImpedanceParams) -> float:
    error = params.equilibrium - position
    return params.stiffness * error - params.damping * velocity


class ImpedanceParamsManager:

    def __init__(self, n_joints: int):

        # Stores the number of controllable joints
        self.n_joints = n_joints

        # Stores the parameters used for each joint to control system
        self.impedance_params = None

    def update_impedance_callback(self, req: UpdateImpedance.Request, resp: UpdateImpedance.Response):
        
        # Ensure the request is properly formatted
        if len(req.stiffness) != len(req.damping) or len(req.stiffness) != len(req.equilibrium):
            return

        # Ensure number of parameters corresponds to number of controllable joints
        if len(req.stiffness) != self.n_joints:
            return
        
        if self.impedance_params is None:
            self.impedance_params = list()

            # Use values given in the request
            for i in range(len(req.stiffness)):
                self.impedance_params.append(req.stiffness[i], req.damping[i], req.equilibrium[i])
        else:

            # Use values given in the request
            for i in range(len(req.stiffness)):
                self.impedance_params[i].stiffness = req.stiffness[i]
                self.impedance_params[i].damping = req.damping[i]
                self.impedance_params[i].equilibrium = req.equilibrium[i]


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
    def __init__(self, joints: List[str], publisher: Publisher, ipm: ImpedanceParamsManager, jsm: JointStateManager, msg: Float64MultiArray = None):
        
        # Publisher to the commands topic
        self.publisher = publisher

        # Maintains current list of impedance parameters
        self.ipm = ipm

        # Maintains the current joint state
        self.jsm = jsm

        # Maintains list of controllable joints
        self.joints = joints

        # Maintains the most recent command to send to the low-level controller
        self.msg = msg

        self.msg.data.extend([0.0] * len(self.joints))


    def command_callback(self):

        # Populate message with command values
        for joint_idx in range(len(self.joints)):
            joint_state = self.jsm.get_joint_state(self.joints[joint_idx])
            if joint_state is None:
                continue

            signal = compute_torque(joint_state[0], joint_state[1], self.ipm.impedance_params[joint_idx])
            self.msg.data[joint_idx] = signal

        self.publisher.publish(self.msg)


def main():

    # Initialize ROS 2
    rclpy.init()

    # Create a ROS 2 node to interface with the rest of ROS 2
    node = rclpy.create_node("impedance_controller")

    # Access the controllable joints
    joints = node.get_parameter("joints")

    ipm = ImpedanceParamsManager(len(joints))
    jsm = JointStateManager(None)

    # Create a publisher that sends commands to the forward controller
    forward_pub = node.create_publisher(Float64MultiArray, "command", qos_profile_system_default)
    cpm = CmdPubManager(joints, forward_pub, ipm, jsm)

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