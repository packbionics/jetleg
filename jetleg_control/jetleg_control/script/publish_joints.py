#!/usr/bin/env python3

# Copyright 2024 Pack Bionics
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

import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


class JetlegJointPublisher(Node):
    def __init__(self):
        super().__init__("jetleg_joint_publisher")
        self.get_logger().info("Constructing node...")

        # Populate JointState msg with initial values of joint states
        joint_names = ["knee_joint", "ankle_joint"]
        self.joint_states = JointState()
        for name in joint_names:
            self.joint_states.name.append(name)

            # All state variables for each joint is 0 by default
            self.joint_states.position.append(0.0)
            self.joint_states.velocity.append(0.0)
            self.joint_states.effort.append(0.0)

        # Create subscriber to read pose data estimated from the IMU
        self.subscriber = self.create_subscription(
            PoseStamped, "estimated_imu_pose", self.estimate_joint_state, qos_profile_system_default)


        # Create publisher to output estimated joint positions ( and/or velocities )
        self.publisher = self.create_publisher(
            JointState, "joint_states", self.estimate_pose, qos_profile_system_default)

    def estimate_joint_state(self, msg: PoseStamped):

        # Populate Pose msg
        self.joint_states.header = msg.header

        self.joint_states.pose.position.x = self.position[0]
        self.joint_states.pose.position.x = self.position[1]
        self.joint_states.pose.position.x = self.position[2]

        self.joint_states.pose.orientation = msg.orientation

        self.publisher.publish(self.joint_states)


def main(argv=None):

    rclpy.init(args=argv)

    # Create node
    node = JetlegJointPublisher()

    try:

        node.get_logger().info("Spinning node...")
        while True:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exiting spin loop...")

    # clean up ros2 resources
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
