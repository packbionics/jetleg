import rclpy
import rclpy.qos
from  rclpy.node import Node

import tf2_ros
import tf2_geometry_msgs

from sensor_msgs.msg._point_cloud2 import PointCloud2

import math


class NaiveSLAM(Node):
    """
    Receives pointcloud data from topic and outputs a collection of distinct
    points constructed from current and previous data. The algorithm assumes 
    perfect localization.
    """

    def __init__(self):
        """
        Initializes the node, publishers, subscriptions, initial map data
        """

        super().__init__('jetleg_naive_slam')

        # Stores existing map data
        self._current_map = PointCloud2()

        self._current_map.header.frame_id = 'odom'
        self._current_map.height = 1
        self._current_map.width = 0
        self._current_map.is_bigendian = False
        self._current_map.row_step = 0
        self._current_map.is_dense = False

        # tf2 buffer used to access tf2 frames
        self._tf2_buffer = tf2_ros.Buffer()
        self._tf2_listener = tf2_ros.TransformListener(self._tf2_buffer, self)
        
        # Declare subscriptions and publishers
        self._pointcloud_subscription = self.create_subscription(
            PointCloud2,
            'pointcloud',
            self.naive_slam_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        self._naive_map_publisher = self.create_publisher(
            PointCloud2,
            'naive_map',
            rclpy.qos.qos_profile_sensor_data
        )

        # Declare the node is finished construction
        self.get_logger().info('Naive SLAM node constructed...')

    def naive_slam_callback(self, msg: PointCloud2):
        """
        Merges previous pointcloud map data with data from the parameter

        :param msg: Latest pointcloud msg received from subscribed topic
        :type msg: PointCloud2
        """

        if msg.point_step != self._current_map.point_step:
            raise RuntimeError('The point step of received msg does not match point step of current map')

        # Retrieve the transformation between msg frame and map frame
        transform = self._tf2_buffer.lookup_transform(
            target_frame=self._current_map.header.frame_id,
            source_frame=msg.header.frame_id,
            time=msg.header.stamp
        )
        
        # Iterate thorugh all points in latest msg
        for msg_point_index in range(0, len(msg.data), msg.point_step):

            msg_point = tf2_geometry_msgs.Vector3Stamped()
            msg_point.header = msg.header
            msg_point.vector.x = msg.data[msg_point_index]
            msg_point.vector.y = msg.data[msg_point_index + 1]
            msg_point.vector.z = msg.data[msg_point_index + 2]

            msg_point = tf2_geometry_msgs.do_transform_vector3(vector3=msg_point, transform=transform)

            # list of point fields for a given point
            is_equal = False

            # Iterate through all points in current map
            for map_point_index in range(0, len(self._current_map.data), self._current_map.point_step):

                map_point = tf2_geometry_msgs.Vector3Stamped()
                map_point.header = self._current_map.header
                map_point.vector.x = self._current_map.data[map_point_index]
                map_point.vector.y = self._current_map.data[map_point_index + 1]
                map_point.vector.z = self._current_map.data[map_point_index + 2]
            
                is_equal = self.are_equal_points(msg_point, map_point) # Check if the points are equal

                # Exit the inner loop if there is an equal point
                if is_equal:
                    break
            
            # Merge point if there is no equal point
            if not is_equal:
                self._current_map.data.append(msg_point)
                self._current_map.row_step += self._current_map.point_step
                self._current_map.width += 1

    def are_equal_points(self, point1: tf2_geometry_msgs.Vector3Stamped, point2: tf2_geometry_msgs.Vector3Stamped):
        """
        Determines if the parameters are equal points based on values of vector.
        Equality is symmetric for the parameters.

        :param point1: First point for comparison
        :type point1: tf2_geometry_msgs.Vector3Stamped
        :param point2: Second point for comparison
        :type point2: tf2_geometry_msgs.Vector3Stamped
        :return: True if the points are equal or false otherwise
        :rtype: bool
        """

        same_x = math.isclose(point1.vector.x, point2.vector.x)
        same_y = math.isclose(point1.vector.y, point2.vector.y)
        same_z = math.isclose(point1.vector.z, point2.vector.z)

        if same_x and same_y and same_z:
            return True
        return False


def main():
    rclpy.init()

    node = NaiveSLAM()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()