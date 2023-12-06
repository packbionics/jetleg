#!/usr/bin/env python3

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


from typing import Iterable

import rclpy

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def create_leg_trajectory(positions: Iterable, names: Iterable) -> JointTrajectory:

    n_positions = len(positions)
    n_names = len(names)

    # validate parameters
    if n_positions == 0 or n_names == 0:
        raise ValueError(
            f"Length of positions: {n_positions} "
            f"or names: {n_names} is of incorrect length")

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


def publish_trajectory_control(names: tuple, names_intact: tuple,
                               joint_cmds: dict, pub1: Publisher, pub2: Publisher):

    positions = (joint_cmds.get(names[0], 0.0), joint_cmds.get(
        names[1], 0.0), joint_cmds.get(names[2], 0.0))

    leg_traj = create_leg_trajectory(positions, names)

    positions = (joint_cmds.get(names_intact[0], 0.0), joint_cmds.get(
        names_intact[1], 0.0), joint_cmds.get(names_intact[2], 0.0))

    leg_traj_intact = create_leg_trajectory(positions, names_intact)

    pub1.publish(leg_traj)
    pub2.publish(leg_traj_intact)


def publish_pybullet_control(names: tuple, joint_cmds: dict, leg_pub: list, leg_intact_pub: list):
    cmds = list()

    for name in names:
        msg = Float64()
        msg.data = joint_cmds.get(name, 0.0)

        cmds.append(msg)

    for i in range(0, 3):
        leg_pub[i].publish(cmds[i])
    for i in range(3, 6):
        leg_intact_pub[i - 3].publish(cmds[i])


def respond_to_control(msg: JointState, joint_cmds: dict, pub1: Publisher,
                       pub2: Publisher, leg_pub: Publisher, leg_intact_pub: Publisher):

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
    names_intact = ("vertical_rail_to_mount_intact",
                    "knee_joint_intact", "ankle_joint_intact")

    publish_trajectory_control(names, names_intact, joint_cmds, pub1, pub2)
    publish_pybullet_control(names + names_intact,
                             joint_cmds, leg_pub, leg_intact_pub)


def main(argv=None):

    # stores the most current received joint commands
    joint_commands = dict()

    rclpy.init(args=argv)

    node = Node('forwarder')
    node.get_logger().info("Node constructed")

    # TODO: read parameters

    # collection of publishers for each specified joint
    signal_pub1 = node.create_publisher(
        JointTrajectory, 'leg_controller/joint_trajectory', qos_profile_system_default)
    signal_pub2 = node.create_publisher(
        JointTrajectory, 'leg_intact_controller/joint_trajectory', qos_profile_system_default)

    prosthetic_leg_pub = list()
    intact_leg_pub = list()

    prosthetic_leg_pub.append(
        node.create_publisher(
            Float64,
            'vertical_rail_to_mount_position_controller/command',
            qos_profile_system_default
        )
    )
    prosthetic_leg_pub.append(
        node.create_publisher(
            Float64,
            'knee_joint_position_controller/command',
            qos_profile_system_default
        )
    )
    prosthetic_leg_pub.append(
        node.create_publisher(
            Float64,
            'ankle_joint_position_controller/command',
            qos_profile_system_default
        )
    )

    intact_leg_pub.append(
        node.create_publisher(
            Float64,
            'vertical_rail_to_mount_intact_position_controller/command',
            qos_profile_system_default
        )
    )
    intact_leg_pub.append(
        node.create_publisher(
            Float64,
            'knee_joint_intact_position_controller/command',
            qos_profile_system_default
        )
    )
    intact_leg_pub.append(
        node.create_publisher(
            Float64,
            'ankle_joint_intact_position_controller/command',
            qos_profile_system_default
        )
    )

    # define subscription callback
    def joint_control_cb(msg): return respond_to_control(
        msg, joint_commands, signal_pub1, signal_pub2, prosthetic_leg_pub, intact_leg_pub)

    # subscriber which reads from topic for control signals for any specified joint
    node.create_subscription(JointState, 'joint_control',
                             joint_control_cb, qos_profile_system_default)

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
