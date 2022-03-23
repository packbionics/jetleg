import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry

import numpy as np
import cv2
import threading
from queue import Queue
import time

from cv_bridge import CvBridge


class PointCloudProcessing(Node):

    def __init__(self):
        super().__init__('jetleg_pointcloud_proc')
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/zed2i/zed_node/mapping/fused_cloud',
            self.cloud_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/zed2i/zed_node/odom',
            self.odom_callback,
            10
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

    def odom_callback(self, msg):
        # get height from odom
        height = msg.pose.pose.position.z
        #self.get_logger().info('height: %f' % height)

    def cloud_callback(self, msg):
        cloud_array = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width, 4))

        heightmap = self.convert_heightmap(cloud_array)
        if heightmap is not None:
            self.compute_traversibility(heightmap)
        
    def convert_heightmap(self, cloud_array):

        # flatten
        cloud_array = cloud_array.reshape((cloud_array.shape[0] * cloud_array.shape[1], 4))

        # remove nan values
        cloud_array = cloud_array[:, :3]
        cloud_array = cloud_array[np.isfinite(cloud_array).any(axis=1)]
        cloud_array = cloud_array[~np.isnan(cloud_array).any(axis=1)]
        #np.save('/home/packbionics/dev_ws/cloud_array.npy', cloud_array)
        # cloud array is (N x 3) array, with each row being [x, y, z]
        # sort by x,y coordinates into heightmap image pixels
        
        theta_z_upper = 55 * np.pi / 180

        # z view restriction
        cloud_restricted = cloud_array[np.where(cloud_array[:,2] <= cloud_array[:,0]*np.tan(theta_z_upper))]

        # y view restriction
        cloud_restricted = cloud_restricted[np.where(cloud_restricted[:,1] <= 1.5)]
        cloud_restricted = cloud_restricted[np.where(cloud_restricted[:,1] >= -1.5)]

        # x view restriction
        cloud_restricted = cloud_restricted[np.where(cloud_restricted[:,0] <= 1.5)]
        cloud_restricted = cloud_restricted[np.where(cloud_restricted[:,0] >= 0.0)]

        try:
            assert cloud_restricted.shape[0] > 0
            x_minimum = np.min(cloud_restricted[:,0])
            x_maximum = np.max(cloud_restricted[:,0])
            y_minimum = np.min(cloud_restricted[:,1])
            y_maximum = np.max(cloud_restricted[:,1])

            map_rows = 60
            map_cols = 60
            heightmap = np.zeros((map_rows,map_cols))

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
                    heightmap[idx_x,idx_y] = point[2]
                else:
                    heightmap[idx_x,idx_y] = max(point[2], heightmap[idx_x,idx_y])
                    
            kernel = np.array([[0,0,1,0,0],
                            [1,1,1,1,1],
                            [1,1,1,1,1],
                            [1,1,1,1,1],
                            [0,0,1,0,0]], dtype=np.uint8)

            # image erosion with kernel
            #heightmap = cv2.erode(heightmap, kernel, iterations=1)
            # image dilation with kernel
            heightmap = cv2.dilate(heightmap, kernel, iterations=2)

            heightmap_in_bytes = (heightmap*255).astype(np.uint8)

            imgmsg = self.bridge.cv2_to_imgmsg(heightmap_in_bytes)
            imgmsg.header.frame_id = 'odom'
            imgmsg.header.stamp = self.get_clock().now().to_msg()

            self.heightmap_publisher.publish(imgmsg)
            return heightmap

        except:
            self.get_logger().info('no points within range')
            return None
                
    def compute_traversibility(self, heightmap):
        # compute sobel gradient in x and y direction
        sobel_x = cv2.Sobel(heightmap, cv2.CV_64F, 1, 0, ksize=5)
        # compute sobel gradient in y direction
        sobel_y = cv2.Sobel(heightmap, cv2.CV_64F, 0, 1, ksize=5)
        
        gradient_map = np.maximum(np.abs(sobel_x), np.abs(sobel_y))
        traversibility_map = np.zeros(gradient_map.shape)

        traversibility_map[gradient_map < 5] = 1
        traversibility_map[np.where((gradient_map >=5) & (gradient_map < 20))]  = 2
        traversibility_map[np.where((gradient_map >=20) & (gradient_map < 35))] = 4
        traversibility_map[np.where((gradient_map >=35) & (gradient_map < 50))] = 4
        traversibility_map[np.where(heightmap == 0)] = 4
        
        traversibility_map_in_bytes = (traversibility_map*255).astype(np.uint8)
        
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