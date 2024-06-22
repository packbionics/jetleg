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

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped


class ImuPoseEstimator(Node):
    def __init__(self):
        super().__init__("imu_pose_estimator")
        self.get_logger().info("Constructing node...")

        self.position = np.zeros((3), dtype=np.float64)
        self.velocity = np.zeros((3), dtype=np.float64)

        self.stamped_pose = PoseStamped()

        self.orientation = None

        self.timestep_start = None
        self.timestep_end = None

        # Create subscriber for the IMU data
        self.subscriber = self.create_subscription(
            Imu, "imu_data", self.estimate_pose, qos_profile_system_default)

        # Create publisher to pass data to next node in pipeline
        self.publisher = self.create_publisher(
            PoseStamped, "estimated_imu_pose", qos_profile_system_default)

    def trap_sum(self, pos: np.array, vel: np.array, time_step: float):
        if time_step < 0.0:
            raise ValueError("Time step cannot be negative.")

        offset = vel * time_step
        return pos + offset

    def estimate_pose(self, msg: Imu):
        self.timestep_end = self.get_clock().now()

        # Skip the first msg to reset timestep computation
        if self.timestep_start is None:
            timestep = 0
        else:
            timestep = self.timestep_end.nanoseconds - self.timestep_start.nanoseconds
            timestep /= float(1e9)

        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        self.position = self.trap_sum(self.position, self.velocity, timestep)
        self.velocity = self.trap_sum(self.velocity, accel, timestep)

        self.orientation = msg.orientation

        self.timestep_start = self.timestep_end

        # Populate Pose msg
        self.stamped_pose.header = msg.header

        self.stamped_pose.pose.position.x = self.position[0]
        self.stamped_pose.pose.position.x = self.position[1]
        self.stamped_pose.pose.position.x = self.position[2]

        self.stamped_pose.pose.orientation = msg.orientation

        self.publisher.publish(self.stamped_pose)


def main(argv=None):

    rclpy.init(args=argv)

    # Create node
    node = ImuPoseEstimator()

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
