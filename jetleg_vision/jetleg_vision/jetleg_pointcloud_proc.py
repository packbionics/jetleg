import rclpy
from rclpy.node import Node
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import matplotlib.pyplot as plt

import numpy as np
import cv2
import threading
from queue import Queue
import time


class PointCloudProcessing(Node):

    def __init__(self):
        super().__init__('jetleg_pointcloud_proc')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/zed2i/zed_node/point_cloud/cloud_registered',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.exit_signal = threading.Event()

        self.img_queue = Queue()

    def listener_callback(self, msg):

        cloud_array = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width, 4))

        heightmap = self.convert_heightmap(cloud_array)
        
    def convert_heightmap(self, cloud_array):
        # flatten
        cloud_array = cloud_array.reshape((cloud_array.shape[0] * cloud_array.shape[1], 4))
        # remove nan values
        cloud_array = cloud_array[:, :3]
        cloud_array = cloud_array[np.isfinite(cloud_array).any(axis=1)]
        cloud_array = cloud_array[~np.isnan(cloud_array).any(axis=1)]
        
        # cloud array is (N x 3) array, with each row being [x, y, z]
        # sort by x,y coordinates into heightmap image pixels
        
        cloud_array[:,:2] = np.round(cloud_array[:, :2], decimals=2)

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

        for point in cloud_restricted:
            # index is (coordinate-minimum) / pixel spacing
            idx_x = int((point[0] - x_minimum) / ((x_maximum-x_minimum) / (map_cols-1)))
            idx_y = int((point[1] - y_minimum) / ((y_maximum-y_minimum) / (map_rows-1)))
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

        self.img_queue.put(heightmap)

    def display_heightmap(self, img_queue):
        while not self.exit_signal.is_set(): 

            if not img_queue.empty():
                img = img_queue.get()
                plt.imshow(img)
                plt.show(block=False)
                plt.pause(1/30.0)
                
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
        
        return traversibility_map

def main(args=None):
    rclpy.init(args=args)

    node = PointCloudProcessing()
    
    t1 = threading.Thread(target=node.display_heightmap, args=(node.img_queue,))
    t1.start()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()