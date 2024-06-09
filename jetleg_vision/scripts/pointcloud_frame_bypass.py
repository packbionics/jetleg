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


class PointCloudProcessing(Node):

    def __init__(self):
        super().__init__('frame_id_bypass_node')

        # Retrieve the desired frame id
        self.declare_parameter('frame_id', 'camera_link')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Set up a subscriber to receive an input point cloud
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            'camera/input',
            self.cloud_callback,
            qos_profile_sensor_data
        )

        # Set up a publisher to output the updated point cloud with desired frame id
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            'camera/output',
            qos_profile_sensor_data,
        )

    def cloud_callback(self, msg: PointCloud2):
        """
        Update the frame id of the input point cloud to the frame_id property.
        """

        msg.header.frame_id = self.frame_id
        self.pointcloud_pub.publish(msg)


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
