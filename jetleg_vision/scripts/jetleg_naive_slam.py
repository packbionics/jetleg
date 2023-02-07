#!/usr/bin/python3

import rclpy
import rclpy.qos
import rclpy.duration
from  rclpy.node import Node

import tf2_ros
import tf2_geometry_msgs

from sensor_msgs.msg._point_cloud2 import PointCloud2
from rcl_interfaces.msg._parameter_descriptor import ParameterDescriptor

import math
import numpy as np
from scipy.spatial.transform import Rotation as R


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

        # Declare parameters
        self.declare_parameter(
            'map_frame_id',
            'odom', 
            ParameterDescriptor(
                description='Frame to which the pointcloud map is attached'
            )
        )

        # Stores existing map data
        self._current_map = PointCloud2()

        self._current_map.header.frame_id = self.get_parameter('map_frame_id').value
        self._current_map.height = 1

        # tf2 buffer used to access tf2 frames
        self._tf2_listener = tf2_ros.TransformListener(tf2_ros.Buffer(), self)
        self._to_odom_transform = tf2_geometry_msgs.TransformStamped()

        # Declare subscriptions and publishers
        self._pointcloud_subscription = self.create_subscription(
            PointCloud2,
            'points',
            self.naive_slam_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        self._naive_map_publisher = self.create_publisher(
            PointCloud2,
            'naive_map',
            10
        )

        self._is_changed = False

        # Declare the node is finished construction
        self.get_logger().info('Naive SLAM node constructed')

    def naive_slam_callback(self, msg: PointCloud2):
        """
        Executes the Naive SLAM algorithm as a callback called by a ROS 2 subscription

        :param msg: Latest pointcloud msg received from subscribed topic
        :type msg: PointCloud2
        """

        self.get_logger().info('Executing callback...')

        try:
            self._naive_slam(msg=msg)
        except RuntimeError:
            self.get_logger().info('Creating new pointcloud map...')

            # Construct new pointcloud map
            self._init_map(msg=msg)
            self.get_logger().info('New pointcloud map created')
    
        # Publish an updated pointcloud map if necessary
        if self._is_changed:
            self._current_map.header.stamp = self.get_clock().now().to_msg()

            self._naive_map_publisher.publish(self._current_map)
            self.get_logger().info('Latest pointcloud map has been published')

            self._is_changed = False
            

    def _init_map(self, msg: PointCloud2) -> PointCloud2:
        """
        Constructs a new map based on the data in the parameter

        :param msg: Latest pointcloud msg received from subscribed topic
        :type msg: PointCloud2
        :return: the updated pointcloud map
        :rtype: PointCloud2
        """        

        try:
            # Retrieve the transformation between msg frame and map frame
            self._to_odom_transform = self._tf2_listener.buffer.lookup_transform(
                target_frame=self._current_map.header.frame_id,
                source_frame=msg.header.frame_id,
                time=msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=5)
            )
        except tf2_ros.ExtrapolationException as ex:
            self.get_logger().info(repr(ex))

        # Copy the values of the msg to replace the previous pointcloud map
        self._current_map.width = msg.width * msg.height
        self._current_map.fields = msg.fields
        self._current_map.is_bigendian = msg.is_bigendian
        self._current_map.point_step = msg.point_step
        self._current_map.data = msg.data
        self._current_map.row_step = len(msg.data)

        # convert from bytes to floating-point values
        cloud_array = np.frombuffer(msg.data, dtype=np.float32)
        cloud_array = np.reshape(cloud_array, (self._current_map.row_step // msg.point_step, msg.point_step // 4))
        cloud_array = cloud_array[:,:3].T

        # extract translation and rotation from transform
        self._to_odom_translation, self._to_odom_rotation = self._construct_from_transform(self._to_odom_transform)

        # Transform each point to global frame
        cloud_array = self._transform_points(cloud_array, self._to_odom_rotation, self._to_odom_transform).astype(np.float32)
        # Convert from floating-point values to byte data
        cloud_bytes = np.frombuffer(cloud_array.T.tobytes(), dtype=np.uint8)

        # iterate through each point in the point cloud
        for idx in range(cloud_array.shape[1]):
            
            # replace position information with transformed values
            for i in range(12):
                self._current_map.data[idx * self._current_map.point_step + i] = cloud_bytes[idx * (4 * 3) + i]

        self._is_changed = True
        return self._current_map

    def _naive_slam(self, msg: PointCloud2) -> PointCloud2:
        """
        Merges previous pointcloud map data with data from the parameter

        :param msg: Latest pointcloud msg received from subscribed topic
        :type msg: PointCloud2
        :raises RuntimeError: Raises RuntimeError if the parameter's point_step does not match the existing map
        :raises RuntimeError: Raises RuntimeError if the length of the parameter's fields does not match the existing map
        :raises RuntimeError: Raises RuntimeError if the parameter's fields does not match the existing map
        :return: the updated pointcloud map
        :rtype: PointCloud2
        """        

        if msg.point_step != self._current_map.point_step:
            raise RuntimeError('The point step of received msg does not match point step of current map')
        if len(msg.fields) != len(self._current_map.fields):
            raise RuntimeError('Unequal number of PointFields')
        for idx in range(len(msg.fields)):
            same_name = msg.fields[idx].name == self._current_map.fields[idx].name
            same_offset = msg.fields[idx].offset == self._current_map.fields[idx].offset
            same_datatype = msg.fields[idx].datatype == self._current_map.fields[idx].datatype
            same_count = msg.fields[idx].count == self._current_map.fields[idx].count

            if not (same_name and same_offset and same_datatype and same_count):
                raise RuntimeError('Mismatch in PointFields')

        self.get_logger().info('Updating pointcloud map...')

        try:
            # Retrieve the transformation between msg frame and map frame
            self._to_odom_transform = self._tf2_listener.buffer.lookup_transform(
                target_frame=self._current_map.header.frame_id,
                source_frame=msg.header.frame_id,
                time=msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=5)
            )
        except tf2_ros.ExtrapolationException as ex:
            self.get_logger().info(repr(ex))

        # convert from bytes to floating-point values
        msg_cloud_array = np.frombuffer(msg.data, dtype=np.float32)
        msg_cloud_array = np.reshape(msg_cloud_array, (len(msg.data) // msg.point_step, msg.point_step // 4))
        msg_cloud_array = msg_cloud_array[:,:3].T

        # convert from bytes to floating-point values
        map_cloud_array = np.frombuffer(self._current_map.data, dtype=np.float32)
        map_cloud_array = np.reshape(map_cloud_array, (len(self._current_map.data) // self._current_map.point_step, self._current_map.point_step // 4))
        map_cloud_array = map_cloud_array[:,:3].T

        # extract translation and rotation from transform
        self._to_odom_translation, self._to_odom_rotation = self._construct_from_transform(self._to_odom_transform)

        # Transform each point to global frame
        msg_cloud_array = self._transform_points(msg_cloud_array, self._to_odom_rotation, self._to_odom_transform).astype(np.float32)
        # Convert from floating-point values to byte data
        msg_cloud_bytes = np.frombuffer(msg_cloud_array.T.tobytes(), dtype=np.uint8)

        # Iterate thorugh all points in latest msg
        for msg_idx in range(msg_cloud_array.shape[1]):

            # self.get_logger().info(str(msg_idx))

            # list of point fields for a given point
            is_equal = False
            # Iterate through all points in current map
            for map_idx in range(map_cloud_array.shape[1]):
            
                is_equal =  math.isclose(msg_cloud_array[0][msg_idx], map_cloud_array[0][map_idx], rel_tol=0.01) and \
                            math.isclose(msg_cloud_array[1][msg_idx], map_cloud_array[1][map_idx], rel_tol=0.01) and \
                            math.isclose(msg_cloud_array[2][msg_idx], map_cloud_array[2][map_idx], rel_tol=0.01) # Check if points are equal

                # Exit the inner loop if there is an equal point
                if is_equal:
                    break
            
            # Merge point if there is no equal point
            if not is_equal:
                for i in range(12):
                    self._current_map.data.append(msg_cloud_bytes[msg_idx * (4 * 3) + i])
                
                self._current_map.row_step += self._current_map.point_step
                self._current_map.width += 1

                self._is_changed = True
            
        # self.get_logger().info('Done!')
        
        return self._current_map

    def _construct_from_transform(self, transform: tf2_geometry_msgs.TransformStamped) -> tuple:
        """
        Constructs a translation and rotation matrix from a Transform

        :param transform: transform from one frame to another
        :type transform: tf2_geometry_msgs.TransformStamped
        :return: tuple with two elements, the first element is the translation vector and the second is a rotation matrix
        :rtype: tuple
        """

        rotation = R.from_quat([
            self._to_odom_transform.transform.rotation.x,
            self._to_odom_transform.transform.rotation.y,
            self._to_odom_transform.transform.rotation.z,
            self._to_odom_transform.transform.rotation.w
        ]).as_matrix()

        translation = np.array([
            self._to_odom_transform.transform.translation.x,
            self._to_odom_transform.transform.translation.y,
            self._to_odom_transform.transform.translation.z,
        ]).reshape(3, 1)

        return translation, rotation        

    def _transform_points(self, point_set: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
        """
        Transforms a collection of points given a specified rotation and translation.
        The rotation should be a (n, n) matrix where n is the dimensions of a single point.
        The translation should be a (n, 1) matrix. The translation should
        be expressed in terms of the resulting frame.

        :param point_set: set of points to transform
        :type point_set: np.ndarray
        :param rotation: desired orientation expressed in the original frame
        :type rotation: np.ndarray
        :param translation: desired translation expressed in the resulting frame
        :type translation: np.ndarray
        :return: transformed points
        :rtype: np.ndarray
        """

        point_set = np.matmul(self._to_odom_rotation, point_set)
        point_set = point_set + self._to_odom_translation

        return point_set

    def _are_equal_points(self, point1: tf2_geometry_msgs.Vector3Stamped, point2: tf2_geometry_msgs.Vector3Stamped) -> bool:
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

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

if __name__ == '__main__':
    main()