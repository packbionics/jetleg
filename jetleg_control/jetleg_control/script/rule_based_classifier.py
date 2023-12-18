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
from rclpy.client import Client

# Import jetleg_control modules
from jetleg_control.classifier import RuleBasedClassifier
from jetleg_control.gait_phase import GaitPhase
from jetleg_control.data import SensorData
from jetleg_control.classifier_parameters import classifier_params

from std_msgs.msg import Float64

from jetleg_interfaces.srv import UpdateImpedance


def add_client(node: Node, srv_type, srv_name) -> Client:
    """Create ROS client and waiting for service availability."""
    client = node.create_client(srv_type, srv_name)
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    return client


def gen_gait_phases(params: classifier_params.Params):

    phases = list()
    num_joints = len(params.stiffness) // len(params.phases)

    # Iterate over each gait phase
    for idx in range(0, len(params.stiffness), num_joints):

        # Access parameters for each phase at a time
        phase_stiffness = params.stiffness[idx: idx + num_joints]
        phase_damping = params.damping[idx: idx + num_joints]
        phase_equilibrum = params.equilibrium[idx: idx + num_joints]

        current_phase = GaitPhase(
            phase_stiffness, phase_damping, phase_equilibrum)
        phases.append(current_phase)

    return phases


class ClassifierNode(Node):
    """
    Reads sensor input to predict the most likely gait mode and gait phase.

    The impedance parameters of the corresponding phase is then
    sent to the impedance controller server as a request.
    """

    def __init__(self):
        """Construct a ClassifierNode object."""
        super().__init__('classifier_node')

        # Define the points of communication for the node
        self.client = add_client(self, UpdateImpedance, 'update_impedance')
        self.subscriber = self.create_subscription(
            Float64, 'sensor_data', self.sub_callback, qos_profile_system_default)

        # Retrieve configuration parameters
        param_listener = classifier_params.ParamListener(self)
        params = param_listener.get_params()

        # Maintain a list of gait modes
        gait_phases = gen_gait_phases(params)

        # Define transitions from first state
        def first_phase_transitions(data: SensorData):
            if data.imu_mean < 0:
                return gait_phases[0]
            else:
                return gait_phases[1]

        # Define transitions from second state
        def second_phase_transitions(data: SensorData):
            if data.imu_mean >= 0:
                return gait_phases[1]
            else:
                return gait_phases[0]

        # Combine individual state transitions into transition model
        def perform_transition(phase: GaitPhase, data: SensorData):
            if phase == gait_phases[0]:
                return first_phase_transitions(data)
            else:
                return second_phase_transitions(data)

        # Create a classifier to determine gait phase
        self.classifier = RuleBasedClassifier(gait_phases[0], perform_transition)

    def sub_callback(self, msg):
        self.currentData = SensorData(msg.data)


def main(args=None):
    rclpy.init(args=args)

    # Create a classifier to determine gait phase in real-time
    classifier_params = ClassifierNode()

    rclpy.spin(classifier_params)

    # Destroy the classifier node explicitly on program shutdown
    classifier_params.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
