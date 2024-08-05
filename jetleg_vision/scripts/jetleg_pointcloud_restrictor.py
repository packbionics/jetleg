#!/usr/bin/python3

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


import rclpy
from rclpy.node import Node
import rclpy.qos
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

import numpy as np


class PointCloudProcessing(Node):

    def __init__(self):
        super().__init__('jetleg_pointcloud_restrictor')
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/pointcloud/input',
            self.cloud_callback,
            qos_profile_sensor_data
        )

        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/pointcloud/output',
            qos_profile_sensor_data
        )

    def cloud_callback(self, msg: PointCloud2):
        cloud_array = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width, 8))

        num_fields = 4

        cloud_array = cloud_array[:, :, :num_fields]
        cloud_array = cloud_array.reshape((
            cloud_array.shape[0] * cloud_array.shape[1], num_fields
        ))

        if cloud_array.shape[0] == 0:
            return

        cloud_array = cloud_array[:, :3]
        # cloud_array[:, [0, 1]] = cloud_array[:, [1, 0]]

        restricted_pointcloud = self.restrict_pointcloud(cloud_array)

        # Compute the size of a single point in bytes
        point_step = 0
        for _ in msg.fields[0:3]:
            point_step += 4

        # Populate output msg
        output_msg = PointCloud2()

        output_msg.header = msg.header
        output_msg.height = 1
        output_msg.width = restricted_pointcloud.shape[0]
        output_msg.fields = msg.fields[0:3]
        output_msg.point_step = point_step
        output_msg.row_step = point_step * restricted_pointcloud.shape[0]
        output_msg.data = restricted_pointcloud.tobytes()

        self.pointcloud_pub.publish(output_msg)

    def restrict_pointcloud(self, cloud_array):

        # cloud array is (N x 3) array, with each row being [x, y, z]
        # sort by x,y coordinates into heightmap image pixels

        # clip point cloud according to current position

        ## Forward direction
        x_minimum = -1.5
        x_maximum = 1.5

        ## Lateral (left/right) direction
        y_minimum = -0.4
        y_maximum = 0.4

        # Remove invalid points
        cloud_array = cloud_array[np.isfinite(cloud_array).any(axis=1)]
        cloud_array = cloud_array[~np.isnan(cloud_array).any(axis=1)]

        # y view restriction
        cloud_restricted = cloud_array[np.where(cloud_array[:, 1] <= y_maximum)]
        cloud_restricted = cloud_restricted[np.where(cloud_restricted[:, 1] >= y_minimum)]

        # x view restriction
        cloud_restricted = cloud_restricted[np.where(cloud_restricted[:, 0] <= x_maximum)]
        cloud_restricted = cloud_restricted[np.where(cloud_restricted[:, 0] >= x_minimum)]

        return cloud_restricted


def main(args=None):
    rclpy.init(args=args)

    node = PointCloudProcessing()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
