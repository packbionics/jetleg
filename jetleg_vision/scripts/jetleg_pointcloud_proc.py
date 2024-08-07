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
from sensor_msgs.msg import PointCloud2, Image
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped

import numpy as np
import cv2
import time

from cv_bridge import CvBridge


class PointCloudProcessing(Node):

    def __init__(self):
        super().__init__('jetleg_pointcloud_proc')
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/zed2i/zed_node/point_cloud/cloud_registered',
            self.cloud_callback,
            qos_profile_sensor_data
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'camera/state',
            self.pose_callback,
            qos_profile_sensor_data
        )

        self.heightmap_publisher = self.create_publisher(
            Image,
            '/heightmap',
            10
        )
        self.traversibility_publisher = self.create_publisher(
            Image,
            '/traversibility',
            10,
        )

        self.bridge = CvBridge()

        self.time_start = time.time()
        self.pose = None

    # transforms point cloud from map frame to base_link frame
    def transform_cloud(self, cloud_array, pose):
        # get rotation from quaternion
        r = R.from_quat([self.pose.pose.orientation.x,
                        self.pose.pose.orientation.y,
                        self.pose.pose.orientation.z,
                        self.pose.pose.orientation.w])

        # remove x,y component rotation
        eulers = r.as_euler('xyz')
        eulers[0] = 0.0
        eulers[1] = 0.0
        eulers[2] = -eulers[2]
        r_xz = R.from_euler('xyz', eulers)
        # get translation vector
        translation = np.array([
            self.pose.pose.position.x,
            self.pose.pose.position.y,
            self.pose.pose.position.z
        ])

        # transform point cloud
        transformed_cloud = r_xz.apply(cloud_array) - r_xz.apply(translation)
        return transformed_cloud

    def pose_callback(self, msg):
        # print pose positions
        # self.get_logger().info('pose: %s' % str(msg.pose.position))
        self.pose = msg

    def cloud_callback(self, msg):
        cloud_array = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width, 8))

        num_fields = 4

        cloud_array = cloud_array[:, :, :num_fields]
        cloud_array = cloud_array.reshape((
            cloud_array.shape[0] * cloud_array.shape[1], num_fields
        ))

        # if self.pose is None:
        #     return

        if cloud_array.shape[0] == 0:
            return

        cloud_array = cloud_array[:, :3]
        # cloud_array[:, [0, 1]] = cloud_array[:, [1, 0]]

        cloud_array = cloud_array[np.isfinite(cloud_array).any(axis=1)]
        cloud_array = cloud_array[~np.isnan(cloud_array).any(axis=1)]
        # cloud_array = self.transform_cloud(cloud_array, self.pose)

        heightmap = self.convert_heightmap(cloud_array)
        if heightmap is not None:
            self.compute_traversibility(heightmap)

    def convert_heightmap(self, cloud_array):

        # cloud array is (N x 3) array, with each row being [x, y, z]
        # sort by x,y coordinates into heightmap image pixels

        # clip point cloud according to current position

        ## Forward direction
        x_minimum = -1.5
        x_maximum = 1.5

        ## Lateral (left/right) direction
        y_minimum = -0.4
        y_maximum = 0.4

        # y view restriction
        cloud_restricted = cloud_array[np.where(cloud_array[:, 1] <= y_maximum)]
        cloud_restricted = cloud_restricted[np.where(cloud_restricted[:, 1] >= y_minimum)]

        # x view restriction
        cloud_restricted = cloud_restricted[np.where(cloud_restricted[:, 0] <= x_maximum)]
        cloud_restricted = cloud_restricted[np.where(cloud_restricted[:, 0] >= x_minimum)]

        assert cloud_restricted.shape[0] > 0

        map_rows = int((x_maximum - x_minimum) * 42)
        map_cols = int((y_maximum - y_minimum) * 42)
        heightmap = np.zeros((map_rows, map_cols))

        idx_x = 0
        idx_y = 0

        map_rows_minus_one = map_rows - 1
        map_cols_minus_one = map_cols - 1

        x_range = (x_maximum - x_minimum) / map_rows_minus_one
        y_range = (y_maximum - y_minimum) / map_cols_minus_one

        coord_minimums = np.array([[x_minimum, y_minimum, 0.0]])
        coord_range = np.array([[x_range, y_range, 1.0]])

        cloud_restricted -= coord_minimums
        cloud_restricted /= coord_range

        for point in cloud_restricted:
            # index is (coordinate-minimum) / pixel spacing
            idx_x = int(point[0])
            idx_y = int(point[1])
            if heightmap[idx_x, idx_y] == 0:
                heightmap[idx_x, idx_y] = point[2]
            else:
                heightmap[idx_x, idx_y] = max(point[2], heightmap[idx_x, idx_y])
        
        # Switch the row and column pixels after creating the raw map
        heightmap = cv2.transpose(heightmap)

        kernel = np.array([
            [0, 0, 1, 0, 0],
            [1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1],
            [0, 0, 1, 0, 0]], dtype=np.uint8)

        heightmap_in_bytes = heightmap / np.max(heightmap)
        heightmap_in_bytes = (heightmap_in_bytes*255).astype(np.uint8)

        # floor detection
        heightmap[np.where(heightmap == 0)] = np.infty

        heights = heightmap.flatten()
        k = 10
        small_idx = np.argpartition(heights, k)

        floor_height = np.mean(heights[small_idx[:10]])
        heightmap = heightmap - floor_height
        heightmap[np.where(heightmap == np.infty)] = -10

        heightmap = cv2.dilate(heightmap, kernel, iterations=2)
        heightmap[np.where(heightmap == -10)] = np.infty

        imgmsg = self.bridge.cv2_to_imgmsg(heightmap_in_bytes)
        imgmsg.header.frame_id = 'odom'
        imgmsg.header.stamp = self.get_clock().now().to_msg()

        self.heightmap_publisher.publish(imgmsg)
        return heightmap

    def compute_traversibility(self, heightmap):
        # compute sobel gradient in x and y direction
        sobel_x = cv2.Sobel(heightmap, cv2.CV_64F, 1, 0, ksize=5)

        # compute sobel gradient in y direction
        sobel_y = cv2.Sobel(heightmap, cv2.CV_64F, 0, 1, ksize=5)

        gradient_map = np.maximum(np.abs(sobel_x), np.abs(sobel_y))
        traversibility_map = np.zeros(gradient_map.shape)

        traversibility_map[gradient_map < 5] = 1
        traversibility_map[np.where((gradient_map >= 5) & (gradient_map < 20))] = 2
        traversibility_map[np.where((gradient_map >= 20) & (gradient_map < 35))] = 4
        traversibility_map[np.where((gradient_map >= 35) & (gradient_map < 50))] = 4
        traversibility_map[np.where(heightmap == 0)] = 4

        traversibility_map_in_bytes = (traversibility_map*(255/4)).astype(np.uint8)

        imgmsg = self.bridge.cv2_to_imgmsg(traversibility_map_in_bytes)
        imgmsg.header.frame_id = 'odom'
        imgmsg.header.stamp = self.get_clock().now().to_msg()

        self.traversibility_publisher.publish(imgmsg)

        return traversibility_map


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
