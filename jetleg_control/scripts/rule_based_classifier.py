#!/usr/bin/python3

import rclpy
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
from rclpy.client import Client
from jetleg_control.classifier import RuleBasedClassifier
from jetleg_control.rule_list import RuleList
from jetleg_control.gait_mode import GaitMode, GaitPhase
from jetleg_control.data import SensorData
from std_msgs.msg import Float64
from rcl_interfaces.srv import GetParameters
from packbionics_interfaces.srv import UpdateImpedance

from array import array
from typing import List
from dataclasses import dataclass

import numpy as np

@dataclass
class RosParams:
    phases: str
    joints: str
    stiffness: np.array
    damping: np.array
    equilibrium: np.array

def add_client(node: Node, srv_type, srv_name) -> Client:
    client = node.create_client(srv_type, srv_name)
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    return client


class ClassifierNode(Node):

    def __init__(self, joint_names: List[str]):
        super().__init__('rule_based_classifier')

        # Define the points of communication for the node
        self.client = add_client(self, UpdateImpedance, 'update_impedance')
        self.subscriber = self.create_subscription(Float64, 'sensor_data', self.sub_callback, qos_profile_system_default)

        # Retrieve configuration parameters
        self.declare_config_parameters()
        self.params = self.read_parameters()

        self.params.joints = joint_names

        # Define the rate of impedance update
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Maintain a list of gait modes
        gait_phases = ClassifierNode.gen_gait_phases(self.params)
        self.gait_modes = [GaitMode(gait_phases)]

        # Maintain a list of gait phases for each gait mode
        self.rule_list = RuleList()
        
        # Define a loop transition to remain in current state
        first_stay = lambda currentPhase, recentInfo: currentPhase == self.gait_modes[0].get(0) and recentInfo.imu_mean < 0
        self.rule_list.add_rule(first_stay,(0,0))
        
        # Define a transition for moving to next state
        first_advance = lambda currentPhase, recentInfo: currentPhase == self.gait_modes[0].get(0) and recentInfo.imu_mean >= 0
        self.rule_list.add_rule(first_advance,(0,1))

        # Define a loop transition to remain in 2nd state
        second_stay = lambda currentPhase, recentInfo: currentPhase == self.gait_modes[0].get(1) and recentInfo.imu_mean >= 0
        self.rule_list.add_rule(second_stay, (0, 1))

        # Define a transition for moving back to first state
        second_advance = lambda currentPhase, recentInfo: currentPhase == self.gait_modes[0].get(1) and recentInfo.imu_mean < 0
        self.rule_list.add_rule(second_advance, (0, 0))

        # Create a classifier to determine gait phase
        self.classifier = RuleBasedClassifier(self.gait_modes[0].get(0), self.rule_list, self.gait_modes)
        self.currentData = None

        # Maintain state of the ROS 2 client
        self.future = None

    def timer_callback(self):
        if self.currentData is None: 
            return

        # Check if the client has not sent a request or received a response by the server
        if self.future is None or self.future.done():

            # Classify the gait phase given the current data
            gait_phase = self.classifier.classify(sensor_data=self.currentData)

            # Describe a service request to update impedance parameters
            request = UpdateImpedance.Request()

            # TODO: Fill in parameters to send to server
            request.stiffness = array('d', gait_phase.stiffness)
            request.damping = array('d', gait_phase.damping)
            request.equilibrium = array('d', gait_phase.equilibrium)
            
            self.future = self.client.call_async(request)
            self.get_logger().info('Impedance Update request made: %s' % repr(request))

    def sub_callback(self, msg):
        self.currentData = SensorData(msg.data)

    def declare_config_parameters(self: Node):
        self.declare_parameter("phases", [""])

        self.declare_parameter("stiffness", [0.0])
        self.declare_parameter("damping", [0.0])
        self.declare_parameter("equilibrium", [0.0])

    def read_parameters(self: Node) -> RosParams:

        # Retrieve list of gait phases
        phases = self.get_parameter("phases").get_parameter_value().string_array_value

        # Retrieve ordered list of impedance parameters
        stiffness = self.get_parameter("stiffness").get_parameter_value().double_array_value
        damping = self.get_parameter("damping").get_parameter_value().double_array_value
        equilibrium = self.get_parameter("equilibrium").get_parameter_value().double_array_value
        
        return RosParams(phases, None, stiffness, damping, equilibrium)

    def gen_gait_phases(params: RosParams) -> List[GaitPhase]:
        
        phases = list()
        num_joints = len(params.joints)

        # Iterate over each gait phase
        for idx in range(0, len(params.stiffness), num_joints):

            # Access parameters for each phase at a time
            phase_stiffness = params.stiffness[idx: idx + num_joints]
            phase_damping = params.damping[idx: idx + num_joints]
            phase_equilibrum = params.equilibrium[idx: idx + num_joints]

            current_phase = GaitPhase(phase_stiffness, phase_damping, phase_equilibrum)
            phases.append(current_phase)

        return phases

def query_joint_names(node, client):
    # Retrieve list of controlled joints
    joint_param_request = GetParameters.Request()

    joint_param_request.names = ["joints"]

    future = client.call_async(joint_param_request)
    rclpy.spin_until_future_complete(node, future)

    parameter_values = future.result().values
    return parameter_values[0].string_array_value


def main(args=None):
    rclpy.init(args=args)

    # Retrieve joint names from running impedance controller node
    parameter_node = rclpy.create_node("joint_names_requester")
    parameter_client = add_client(parameter_node, GetParameters, '/impedance_controller/get_parameters')

    joint_names = query_joint_names(parameter_node, parameter_client)

    # Destroy node after completing its task
    parameter_node.destroy_node()

    # Create a classifier to determine gait phase in real-time
    rule_based_classifier = ClassifierNode(joint_names)

    rclpy.spin(rule_based_classifier)

    # Destroy the classifier node explicitly on program shutdown
    rule_based_classifier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()