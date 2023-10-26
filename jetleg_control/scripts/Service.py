#!/usr/bin/python3
import sys
from array import array as array
from packbionics_interfaces.srv._update_impedance import UpdateImpedance
import rclpy
from rclpy.node import Node

import math


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(UpdateImpedance, '/jetleg_controller/impedance_params')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = UpdateImpedance.Request()

    def send_request(self, stiffness, damping, equilibrium):
        self.req.stiffness = stiffness
        self.req.damping = damping
        self.req.equilibrium = equilibrium
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()

    stiffness = [1000.0,1000.0,1000.0]
    damping = [0.0 , 0.0 , 0.0]
    equilibrium = [math.radians((40 - 15) * 0.8) , math.radians(60), math.radians(102 - 90)]
    stiffness = array('d', stiffness)
    damping = array('d', damping)
    equilibrium = array('d', equilibrium)

    response = minimal_client.send_request(stiffness, damping, equilibrium)
    minimal_client.get_logger().info(
        'Result of add_two_ints: for'  
        )

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()