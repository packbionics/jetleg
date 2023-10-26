#!/usr/bin/python3

import rclpy
from rclpy.client import Client

from packbionics_interfaces.srv import UpdateImpedance

import math

from typing import List
from array import array


def send_request(client: Client, stiffness: List[float], damping: List[float], equilibrium: List[float]):
    request = UpdateImpedance.Request()

    request.stiffness = array('d', stiffness)
    request.damping = array('d', damping)
    request.equilibrium = array('d', equilibrium)

    return client.call_async(request)

def main():
    rclpy.init()

    node = rclpy.create_node("impedance_updater")

    client = node.create_client(UpdateImpedance, "update_impedance")

    stiffness = [10.0, 10.0, 10.0]
    damping = [5.0, 5.0, 5.0]
    equilibrium = [math.radians(40 - 15) * 0.8, math.radians(60), math.radians(102 - 90)]

    future = send_request(client, stiffness, damping, equilibrium)
    rclpy.spin_until_future_complete(node, future)

    node.get_logger().info("Result received")

    rclpy.shutdown()


if __name__ == "__main__":
    main()