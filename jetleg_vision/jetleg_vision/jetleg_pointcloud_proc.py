import rclpy
from rclpy.node import Node
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
import matplotlib.pyplot as plt

import numpy as np
import cv2
import threading
from queue import Queue
import time

from cv_bridge import CvBridge


class PointCloudProcessing(Node):

    def __init__(self):
        super().__init__('jetleg_pointcloud_proc')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/zed2i/zed_node/point_cloud/cloud_registered',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(
            Image,
            '/heightmap',
            10
        )

        self.exit_signal = threading.Event()

        self.bridge = CvBridge()

        self.img_queue = Queue()
        self.img = None
        self.img_is_populated = False

        self.time_start = time.time()

    def listener_callback(self, msg):
        self.time_start = time.time()

        cloud_array = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width, 4))

        heightmap = self.convert_heightmap(cloud_array)
        for j in range(40):
            for i in range(40):
                self.get_logger().info(str(i) + ', ' + str(j) + ': ' + str(heightmap[i,j]))

        traversibility_map = self.compute_traversibility(heightmap)

        # self.get_logger().info(str(traversibility_map[:,0]))

        self.get_logger().info('Time (s) per Tick: ' + str(time.time() - self.time_start))
        
    def convert_heightmap(self, cloud_array):

        # flatten
        cloud_array = cloud_array.reshape((cloud_array.shape[0] * cloud_array.shape[1], 4))

        # remove nan values
        cloud_array = cloud_array[:, :3]
        cloud_array = cloud_array[np.isfinite(cloud_array).any(axis=1)]
        cloud_array = cloud_array[~np.isnan(cloud_array).any(axis=1)]
        
        # cloud array is (N x 3) array, with each row being [x, y, z]
        # sort by x,y coordinates into heightmap image pixels
        
        theta_z_upper = 5 * np.pi / 180

        # z view restriction
        cloud_restricted = cloud_array[np.where(cloud_array[:,2] <= cloud_array[:,0]*np.tan(theta_z_upper))]

        # y view restriction
        cloud_restricted = cloud_restricted[np.where(cloud_restricted[:,1] <= 0.6)]
        cloud_restricted = cloud_restricted[np.where(cloud_restricted[:,1] >= -0.6)]

        # x view restriction
        cloud_restricted = cloud_restricted[np.where(cloud_restricted[:,0] <= 2)]

        x_minimum = np.min(cloud_restricted[:,0])
        x_maximum = np.max(cloud_restricted[:,0])
        y_minimum = np.min(cloud_restricted[:,1])
        y_maximum = np.max(cloud_restricted[:,1])

        map_rows = 40
        map_cols = 40
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
        heightmap = cv2.erode(heightmap, kernel, iterations=1)
        # image dilation with kernel
        heightmap = cv2.dilate(heightmap, kernel, iterations=1)

        heightmap_in_bytes = (heightmap*255).astype(np.uint8)

        imgmsg = self.bridge.cv2_to_imgmsg(heightmap_in_bytes)
        imgmsg.header.frame_id = 'odom'
        imgmsg.header.stamp = self.get_clock().now().to_msg()


        self.publisher.publish(imgmsg)

        #self.img_queue.put(heightmap)
        self.img = heightmap
        if not self.img_is_populated:
            self.img_is_populated = True

        return heightmap

    def display_heightmap(self, img_queue):
        while not self.exit_signal.is_set(): 

            #if not img_queue.empty():
            #img = img_queue.get()
            if self.img_is_populated:
                plt.imshow(self.img)
                plt.show(block=False)
                plt.pause(1/30.0)
                
    def compute_traversibility(self, heightmap):
        # compute sobel gradient in x and y direction
        sobel_x = cv2.Sobel(heightmap, cv2.CV_64F, 1, 0, ksize=5)
        # compute sobel gradient in y direction
        sobel_y = cv2.Sobel(heightmap, cv2.CV_64F, 0, 1, ksize=5)

        # self.get_logger().info('Sobel X: ' + str(sobel_x[5,20]))
        # self.get_logger().info('Sobel Y: ' + str(sobel_y[5,20]))

        gradient_map = np.maximum(np.abs(sobel_x), np.abs(sobel_y))
        traversibility_map = np.zeros(gradient_map.shape)

        traversibility_map[gradient_map < 5] = 1
        traversibility_map[np.where((gradient_map >=5) & (gradient_map < 20))]  = 2
        traversibility_map[np.where((gradient_map >=20) & (gradient_map < 35))] = 4
        traversibility_map[np.where((gradient_map >=35) & (gradient_map < 50))] = 4
        traversibility_map[np.where(heightmap == 0)] = 4
        
        return traversibility_map

def main(args=None):
    rclpy.init(args=args)

    node = PointCloudProcessing()
    
    t1 = threading.Thread(target=node.display_heightmap, args=(node.img_queue,))
    #t1.start()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()