#!/usr/bin/python3

import rclpy
from rclpy.client import Client

from packbionics_interfaces.srv import UpdateImpedance

import time
import numpy as np

from typing import List
from array import array


def send_request(client: Client, stiffness: List[float], damping: List[float], equilibrium: List[float]):
    request = UpdateImpedance.Request()

    request.stiffness = array('d', stiffness)
    request.damping = array('d', damping)
    request.equilibrium = array('d', equilibrium)

    future = client.call_async(request)
    return future

def main():
    rclpy.init()

    node = rclpy.create_node("impedance_updater")

    client = node.create_client(UpdateImpedance, "update_impedance")

    stiffness = [40.0, 40.0, 40.0]
    damping = [40.0, 40.0, 40.0]

    equilibrium = np.array([
        [0, 0, 0],
        [40, 0, -12],
        [-40, 90, 2.5],
        [-40, 0, 2.5]
    ])

    equilibrium = np.radians(equilibrium)

    node.declare_parameter("idx", 0)
    idx = node.get_parameter("idx").get_parameter_value().integer_value

    # equilibrium = [math.radians(40 - 15) * 0.8, math.radians(60), math.radians(102 - 90)]

    future = send_request(client, stiffness, damping, equilibrium[idx])

    node.get_logger().info("Request sent")
    rclpy.spin_until_future_complete(node, future)

    node.get_logger().info("Result received")

    rclpy.shutdown()


if __name__ == "__main__":
    main()