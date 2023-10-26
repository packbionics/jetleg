#!/usr/bin/python3

import rclpy
import rclpy.qos
from rclpy.node import Node
from rclpy.publisher import Publisher

from array import array

from std_msgs.msg import Float64MultiArray

def main():
    rclpy.init()

    pub: Publisher

    node = rclpy.create_node("commander")
    pub = node.create_publisher(Float64MultiArray, "/jetleg_controller/commands", rclpy.qos.qos_profile_system_default)

    while True:
        msg = Float64MultiArray()
        msg.data = array('d', [100.0, 100.0, 100.0])

        pub.publish(msg)


if __name__ == "__main__":
    main()