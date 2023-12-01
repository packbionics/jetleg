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


# Import ROS client libraries
import rclpy
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node

# Import jetleg_control modules
from jetleg_control.classifier import RuleBasedClassifier
from jetleg_control.rule_list import RuleList
from jetleg_control.gait_mode import GaitMode
from jetleg_control.data import SensorData
from jetleg_control.helper import add_client, gen_gait_phases, query_joint_names
from jetleg_control.classifier_parameters import rule_based_classifier as classifier_parameters

# Import ROS interface stubs
from std_msgs.msg import Float64
from rcl_interfaces.srv import GetParameters

from packbionics_interfaces.srv import UpdateImpedance

# Import general Python 3 modules
from array import array
from typing import List


class ClassifierNode(Node):
    """
    Reads sensor input to predict the most likely gait mode and gait phase.

    The impedance parameters of the corresponding phase is then
    sent to the impedance controller server as a request.
    """

    def __init__(self, joint_names: List[str]):
        super().__init__('rule_based_classifier')

        # Define the points of communication for the node
        self.client = add_client(self, UpdateImpedance, 'update_impedance')
        self.subscriber = self.create_subscription(
            Float64, 'sensor_data', self.sub_callback, qos_profile_system_default)

        # Retrieve configuration parameters
        param_listener = classifier_parameters.ParamListener(self)
        params = param_listener.get_params()

        # Define the rate of impedance update
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Maintain a list of gait modes
        gait_phases = gen_gait_phases(params, joint_names)
        self.gait_modes = [GaitMode(gait_phases)]

        # Maintain a list of gait phases for each gait mode
        self.rule_list = RuleList()

        # Define a loop transition to remain in current state
        def first_stay(currentPhase, recentInfo): return currentPhase == self.gait_modes[0].get(
            0) and recentInfo.imu_mean < 0
        self.rule_list.add_rule(first_stay, (0, 0))

        # Define a transition for moving to next state
        def first_advance(currentPhase, recentInfo): return currentPhase == self.gait_modes[0].get(
            0) and recentInfo.imu_mean >= 0
        self.rule_list.add_rule(first_advance, (0, 1))

        # Define a loop transition to remain in 2nd state
        def second_stay(currentPhase, recentInfo): return currentPhase == self.gait_modes[0].get(
            1) and recentInfo.imu_mean >= 0
        self.rule_list.add_rule(second_stay, (0, 1))

        # Define a transition for moving back to first state
        def second_advance(currentPhase, recentInfo):
            return currentPhase == self.gait_modes[0].get(1) and recentInfo.imu_mean < 0
        self.rule_list.add_rule(second_advance, (0, 0))

        # Create a classifier to determine gait phase
        self.classifier = RuleBasedClassifier(
            self.gait_modes[0].get(0), self.rule_list, self.gait_modes)
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

            # Fill in parameters to send to server
            request.stiffness = array('d', gait_phase.stiffness)
            request.damping = array('d', gait_phase.damping)
            request.equilibrium = array('d', gait_phase.equilibrium)

            self.future = self.client.call_async(request)
            self.get_logger().info('Impedance Update request made: %s' % repr(request))

    def sub_callback(self, msg):
        self.currentData = SensorData(msg.data)


def get_joint_names() -> List[str]:
    parameter_node = rclpy.create_node("joint_names_requester")
    parameter_client = add_client(
        parameter_node, GetParameters, '/impedance_controller/get_parameters')

    joint_names = query_joint_names(parameter_node, parameter_client)

    # Destroy node after completing its task
    parameter_node.destroy_node()
    return joint_names


def main(args=None):
    rclpy.init(args=args)

    # Retrieve joint names from running impedance controller node
    joint_names = get_joint_names()

    # Create a classifier to determine gait phase in real-time
    rule_based_classifier = ClassifierNode(joint_names)

    rclpy.spin(rule_based_classifier)

    # Destroy the classifier node explicitly on program shutdown
    rule_based_classifier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
