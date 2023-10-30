#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.client import Client

from packbionics_interfaces.srv import UpdateImpedance

import numpy as np

from typing import List
from array import array
from dataclasses import dataclass


@dataclass
class RosParams:
    phases: str
    joints: str
    stiffness: np.array
    damping: np.array
    equilibrium: np.array


def send_request(client: Client, stiffness: List[float], damping: List[float], equilibrium: List[float]):
    request = UpdateImpedance.Request()

    request.stiffness = array('d', stiffness)
    request.damping = array('d', damping)
    request.equilibrium = array('d', equilibrium)

    future = client.call_async(request)
    return future    

def declare_parameters(node: Node):
    node.declare_parameter("phases", [""])
    node.declare_parameter("joints", [""])

    node.declare_parameter("stiffness", [0.0])
    node.declare_parameter("damping", [0.0])
    node.declare_parameter("equilibrium", [0.0])

def read_parameters(node: Node) -> RosParams:
    params = RosParams(None, None, None, None, None)

    # Retrieve list of gait phases
    params.phases = node.get_parameter("phases").get_parameter_value().string_array_value

    # Retrieve list of controlled joints
    params.joints = node.get_parameter("joints").get_parameter_value().string_array_value

    # Retrieve ordered list of impedance parameters
    params.stiffness = node.get_parameter("stiffness").get_parameter_value().double_array_value
    params.damping = node.get_parameter("damping").get_parameter_value().double_array_value
    params.equilibrium = node.get_parameter("equilibrium").get_parameter_value().double_array_value
    
    return params

def get_stiffness(params: RosParams) -> np.array:
    return np.reshape(params.stiffness, (len(params.phases), len(params.joints)))

def get_damping(params: RosParams) -> np.array:
    return np.reshape(params.damping, (len(params.phases), len(params.joints)))

def get_equilibrium(params: RosParams) -> np.array:
    return np.reshape(params.equilibrium, (len(params.phases), len(params.joints)))

def main():
    rclpy.init()

    node = rclpy.create_node("impedance_updater")

    client = node.create_client(UpdateImpedance, "update_impedance")

    declare_parameters(node)
    params = read_parameters(node)

    stiffness = get_stiffness(params)
    damping = get_damping(params)
    equilibrium = get_equilibrium(params)

    node.declare_parameter("idx", 0)
    idx = node.get_parameter("idx").get_parameter_value().integer_value

    future = send_request(client, stiffness[idx], damping[idx], equilibrium[idx])

    node.get_logger().info("Request sent")
    rclpy.spin_until_future_complete(node, future)

    node.get_logger().info("Result received")

    rclpy.shutdown()


if __name__ == "__main__":
    main()