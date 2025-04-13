#!/usr/bin/python3

# Copyright 2025 Pack Bionics
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
from sensor_msgs.msg import PointCloud2, PointField
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped

import numpy as np
import pandas as pd

import cv2
import time

from cv_bridge import CvBridge

from jetleg_vision_params import points_transform

from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor

import random


class PointCloudCluster(Node):

    def __init__(self):
        super().__init__('jetleg_pointcloud_cluster')

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            'points',
            self.cloud_callback,
            qos_profile_sensor_data
        )

        self.heightmap_publisher = self.create_publisher(
            PointCloud2,
            '/points/clustered',
            10
        )

        cluster_tolerance = 0.05
        min_samples = 5

        self.clusterer = DBSCAN(eps=cluster_tolerance, min_samples=min_samples)

        # Initialize 4 colors to associate with labels

        self.colors = np.zeros((4,), dtype=np.float32)
        # for i in range(self.colors.shape[0]):
        #     self.colors[i] = PointCloudCluster.generate_color()
        self.colors[0] = PointCloudCluster.create_color_float(PointCloudCluster.create_color(0, 255, 0))
        self.colors[1] = PointCloudCluster.create_color_float(PointCloudCluster.create_color(0, 0, 255))
        self.colors[2] = PointCloudCluster.create_color_float(PointCloudCluster.create_color(0, 255, 255))
        self.colors[3] = PointCloudCluster.create_color_float(PointCloudCluster.create_color(255, 255, 0))


    def filter_ground_plane(self, cloud_sample_df):
        ransac = RANSACRegressor()
        
        # Threshold for considering a point as an inlier
        # of the ground plane model
        distance_threshold = 0.05

        # Estimate the ground plane z-coordinate at each (x, y) point
        augmented_df = cloud_sample_df
        augmented_df['z_pred'] = ransac                             \
            .fit(cloud_sample_df[['x', 'y']], cloud_sample_df['z']) \
            .predict(cloud_sample_df[['x', 'y']])

        augmented_df['z_residual'] = np.abs(augmented_df['z_pred'] - augmented_df['z'])
        ground_plane = augmented_df[augmented_df['z_residual'] < distance_threshold]

        outliers = augmented_df[augmented_df['z_residual'] >= distance_threshold]

        # Return the ground plane and outliers as separate dataframes
        # in the original format
        return ground_plane[['x', 'y', 'z']], outliers[['x', 'y', 'z']]
        

    def cloud_callback(self, msg):
        cloud_array = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width, 8))
        cloud_array = cloud_array.reshape((
            cloud_array.shape[0] * cloud_array.shape[1], 8
        ))

        cloud_array = cloud_array[:, :3]

        cloud_array = cloud_array[np.isfinite(cloud_array).any(axis=1)]
        cloud_array = cloud_array[~np.isnan(cloud_array).any(axis=1)]

        # Do not process further if there are no points
        if cloud_array.shape[0] == 0:
            return

        cloud_df = pd.DataFrame(cloud_array, columns=[
            'x', 'y', 'z'
        ])

        ground_plane, cloud_sample_df = self.filter_ground_plane(cloud_sample_df=cloud_df)
        cloud_sample_df = cloud_sample_df.sample(n=10000)
        
        self.get_logger().info('Clustering points...')
        
        # # Assign cluster labels to points
        cloud_sample_df['label'] = self.clusterer.fit_predict(cloud_sample_df)
        num_labels = cloud_sample_df['label'].nunique()


        self.get_logger().info(f'Generated labels: Count {num_labels}')

        cloud_sample_df['rgb'] = cloud_sample_df['label'].apply(lambda x: self.get_color(x))        

        # point_colors = np.zeros(cluster_labels.shape[0], dtype=np.float32)
        # for i in cluster_labels:
        #     point_colors[i] = self.colors[i + 1]

        # self.get_logger().info('Assigned colors...')
        
        # Replace original point colors with label colors
        # cloud_array[:5000, 4] = point_colors

        cloud_array = cloud_sample_df.values
        # cloud_array_shape = cloud_array.shape
        # cloud_array = cloud_array.flatten().astype(dtype=np.float32)

        cloud_array = cloud_array.astype(dtype=np.float32)
        cloud_array_bytes = cloud_array.tobytes()

        self.get_logger().info('Publishing pointcloud...')

        field_size = 4

        msg.fields = msg.fields[:3]
        msg.fields.append(PointField())

        msg.fields[-1].name = 'rgb'
        msg.fields[-1].offset = msg.fields[-2].offset + field_size * 2
        msg.fields[-1].datatype = PointField.FLOAT32
        msg.fields[-1].count = 1

        self.get_logger().info(f'Offset: {msg.fields[-1].offset}')

        msg.point_step = msg.fields[-1].offset + field_size * msg.fields[-1].count
        msg.row_step = msg.point_step * cloud_array.shape[0]

        msg.data = cloud_array_bytes
        
        msg.height = 1
        msg.width = cloud_array.shape[0]

        self.heightmap_publisher.publish(msg)

    def get_color(self, label):
        if label == -1:
            outlier_color = PointCloudCluster.create_color(255, 0, 0)
            return np.frombuffer(outlier_color.tobytes(), dtype=np.float32)

        while label >= self.colors.shape[0]:
            n_colors_initial = self.colors.shape[0]
            self.colors = np.resize(self.colors, label + 1)
            
            # Add enough (random) colors to color all clusters
            for i in range(n_colors_initial, label + 1):
                color = PointCloudCluster.generate_color()
                self.colors[i] = color

        return self.colors[label]

    def create_color(r, g, b):
        color = np.zeros(4, dtype=np.uint8)

        color[0] = b
        color[1] = g
        color[2] = r

        return color

    def create_color_float(color):
        color_float = np.frombuffer(color.tobytes(), dtype=np.float32)
        return color_float

    def generate_color():
        """
        Creates a random color and returns it as a 32-bit float
        
        The color is generated from randomly sampling from the
        8-bit RGB channels.
        """

        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)

        color = PointCloudCluster.create_color(r, g, b)

        # Convert to RGB integers into a 32-bit floating number
        return PointCloudCluster.create_color_float(color)


def main(args=None):
    rclpy.init(args=args)

    node = PointCloudCluster()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
