#!/usr/bin/env python3

from typing import Iterable

import rclpy

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rcl_interfaces.msg import ParameterDescriptor


def create_leg_trajectory(positions: Iterable, names: Iterable) -> JointTrajectory:

    n_positions = len(positions)
    n_names = len(names)

    # validate parameters
    if n_positions == 0 or n_names == 0:
        raise ValueError(f"Length of positions: {n_positions} or names: {n_names} is of incorrect length")

    leg_trajectory_msg = JointTrajectory()
    trajectory_point = JointTrajectoryPoint()

    # define joint commands
    trajectory_point.positions.append(positions[0])
    trajectory_point.positions.append(positions[1])
    trajectory_point.positions.append(positions[2])

    leg_trajectory_msg.header.frame_id = "base_link"
    leg_trajectory_msg.points.append(trajectory_point)

    # define mapping from joint angles to joint
    leg_trajectory_msg.joint_names.append(names[0])
    leg_trajectory_msg.joint_names.append(names[1])
    leg_trajectory_msg.joint_names.append(names[2])

    return leg_trajectory_msg

def respond_to_control(msg: JointState, joint_cmds: dict, pub1: Publisher, pub2: Publisher):

    n = len(msg.name)
    if n == 0:
        return

    # update latest joint commands
    for i in range(n):
        joint_name = msg.name[i]
        joint_cmd = msg.position[i]

        joint_cmds.update({joint_name: joint_cmd})

    # publish current joint commands
    names = ("vertical_rail_to_mount", "knee_joint", "ankle_joint")
    positions = (joint_cmds.get(names[0], 0.0), joint_cmds.get(names[1], 0.0), joint_cmds.get(names[2], 0.0))

    leg_traj = create_leg_trajectory(positions, names)

    names = ("vertical_rail_to_mount_intact", "knee_joint_intact", "ankle_joint_intact")
    positions = (joint_cmds.get(names[0], 0.0), joint_cmds.get(names[1], 0.0), joint_cmds.get(names[2], 0.0))

    leg_traj_intact = create_leg_trajectory(positions, names)

    pub1.publish(leg_traj)
    pub2.publish(leg_traj_intact)


def main(argv=None):

    # stores the most current received joint commands
    joint_commands = dict()

    rclpy.init(args=argv)

    node = Node('forwarder')
    node.get_logger().info("Node constructed")

    # TODO: read parameters

    # collection of publishers for each specified joint
    signal_pub1 = node.create_publisher(JointTrajectory, 'leg_controller/joint_trajectory', qos_profile_system_default)
    signal_pub2 = node.create_publisher(JointTrajectory, 'leg_intact_controller/joint_trajectory', qos_profile_system_default)

    # define subscription callback
    joint_control_cb = lambda msg: respond_to_control(msg, joint_commands, signal_pub1, signal_pub2)

    # subscriber which reads from topic for control signals for any specified joint
    node.create_subscription(JointState, 'joint_control', joint_control_cb, qos_profile_system_default)

    # execute defined callbacks
    node.get_logger().info("Spinning node...")

    try:
        while True:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exiting spin loop...")

    # clean up ros2 resources
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()