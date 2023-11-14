#!/usr/bin/python3

import rclpy
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
from jetleg_control.classifier import RuleBasedClassifier
from jetleg_control.rule_list import RuleList
from jetleg_control.gait_mode import GaitMode, GaitPhase
from jetleg_control.data import SensorData
from std_msgs.msg import Float64
from packbionics_interfaces.srv import UpdateImpedance

from array import array

class ClassifierNode(Node):

    def __init__(self):
        super().__init__('classifier_tester')

        # Define the points of communication for the node
        self.client = self.create_client(UpdateImpedance, 'update_impedance')
        self.subscriber = self.create_subscription(Float64, 'sensor_data', self.sub_callback, qos_profile_system_default)
        
        # Define the rate of impedance update
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Maintain a list of gait modes
        self.gait_modes = [GaitMode([GaitPhase([1.0, 1.0], [0.5, 0.5], [0.0, 0.0]), GaitPhase([1.0, 1.0], [0.5, 0.5], [0.2, 0.1])])]

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
            self.get_logger().info('Impedance Update request made')

    def sub_callback(self, msg):
        self.currentData = SensorData(msg.data)


def main(args=None):
    rclpy.init(args=args)

    classifier_tester = ClassifierNode()

    rclpy.spin(classifier_tester)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    classifier_tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()