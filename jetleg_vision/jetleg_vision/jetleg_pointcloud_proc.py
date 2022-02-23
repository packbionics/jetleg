import rclpy
from rclpy.node import Node
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import numpy as np


class PointCloudProcessing(Node):

    def __init__(self):
        super().__init__('jetleg_pointcloud_proc')
        self.subscription = self.create_subscription(
            PointCloud2,
            'pointcloud',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print('Received pointcloud message')
        cloud_array = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width, 4))
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = PointCloudProcessing()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()