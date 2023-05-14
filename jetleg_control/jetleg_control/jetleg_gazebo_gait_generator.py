#!/usr/bin/env python3

import math
from datetime import datetime

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from scipy import interpolate

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        
class JetLegGait(Node):
    
    #defines positions data
    #postions: [hip, knee, ankle]
    def __init__(self):
        rclpy.init()
        super().__init__('jetleg_gait_generator')

        self.position_multiplier = np.pi / 180.0

        self.max_position = 100.0
        self.delta_position = 2 * np.pi/180.0
        self.positions = np.array([0.0,0.0,0.0])
        self.positions_intact = np.array([0.0,0.0,0.0])
        self.joint_limit_max = np.array([np.pi/4.0, np.pi/2.0, np.pi/4.0])
        self.joint_limit_min = np.array([-np.pi/4.0, 0.0, -np.pi/8.0])

        self.declare_parameter('leg_trajectory_topic', value='/leg_controller/joint_trajectory')
        self.leg_trajectory_topic = self.get_parameter('leg_trajectory_topic').get_parameter_value().string_value

        self.declare_parameter('leg_intact_trajectory_topic', value='/leg_intact_controller/joint_trajectory')
        self.leg_intact_trajectory_topic = self.get_parameter('leg_intact_trajectory_topic').get_parameter_value().string_value
        
        self.leg_trajectory_publisher = self.create_publisher(JointTrajectory,
                                               self.leg_trajectory_topic,
                                               10)        

        self.leg_intact_trajectory_publisher = self.create_publisher(JointTrajectory,
                                        self.leg_intact_trajectory_topic,
                                        10)        


        self.x_points = np.linspace(-0.5,1.5,21)
        self.knee_setpoints_quick = [8,18,20,12,10,16,38,60,55,18] #[12,25,30,28,20,22,50,70,50,20]
        self.knee_setpoints_quick = np.array(self.knee_setpoints_quick[5:] + self.knee_setpoints_quick + self.knee_setpoints_quick[:6])
        self.knee_tck_quick = interpolate.splrep(self.x_points, self.knee_setpoints_quick)
        self.hip_setpoints_quick = [40,38,30,18,3,-5,0,20,37,41]
        self.hip_setpoints_quick = (np.array(self.hip_setpoints_quick[5:] + self.hip_setpoints_quick + self.hip_setpoints_quick[:6]) - 15) * 0.8
        self.hip_tck_quick = interpolate.splrep(self.x_points, self.hip_setpoints_quick)
        self.ankle_setpoints_quick = [75,81,75,70,72,80,102,90,80,75]
        self.ankle_setpoints_quick = np.array(self.ankle_setpoints_quick[5:] + self.ankle_setpoints_quick + self.ankle_setpoints_quick[:6]) - 90
        self.ankle_tck_quick = interpolate.splrep(self.x_points, self.ankle_setpoints_quick)
        
        self.time_start = datetime.now()

        gait_pub_period = 1.0/30.0
        self.gait_timer = self.create_timer(gait_pub_period, self.pub_cmd)
        
        self.T = 4.0

    def create_leg_trajectory(self, positions, names) -> JointTrajectory:
        leg_trajectory_msg = JointTrajectory()

        trajectory_point = JointTrajectoryPoint()

        trajectory_point.positions.append(positions[0])
        trajectory_point.positions.append(positions[1])
        trajectory_point.positions.append(positions[2])

        # duration = Duration(seconds=1)
        # trajectory_point.time_from_start = duration.to_msg()

        leg_trajectory_msg.points.append(trajectory_point)

        leg_trajectory_msg.header.frame_id = "base_link"
        # leg_trajectory_msg.header.stamp = self.get_clock().now().to_msg()

        leg_trajectory_msg.joint_names.append(names[0])
        leg_trajectory_msg.joint_names.append(names[1])
        leg_trajectory_msg.joint_names.append(names[2])

        return leg_trajectory_msg

    def publish_position(self):
        names = ('vertical_rail_to_mount', 'knee_joint', 'ankle_joint')
        leg_trajectory_msg = self.create_leg_trajectory(self.positions, names)

        names = ('vertical_rail_to_mount_intact', 'knee_joint_intact', 'ankle_joint_intact')
        leg_intact_trajectory_msg = self.create_leg_trajectory(self.positions_intact, names)

        self.leg_trajectory_publisher.publish(leg_trajectory_msg)
        self.leg_intact_trajectory_publisher.publish(leg_intact_trajectory_msg)
        
        
    #process input keys, and updates positions array of node. Then, calls node's publish method.
    def pub_cmd(self):
        #TO DO: add a speed variable
        current_time = datetime.now() - self.time_start
        current_time = current_time.seconds + current_time.microseconds / 1000000.0
        
        percent = math.fmod(current_time,self.T) /self.T
        percent_shifted = math.fmod(current_time +self.T / 2,self.T) /self.T
        
        # hip angle
        self.positions[0] = - interpolate.splev(percent, self.hip_tck_quick)
        self.positions[0] = math.radians(self.positions[0])

        self.positions_intact[0] = - interpolate.splev(percent_shifted, self.hip_tck_quick)
        self.positions_intact[0] = math.radians(self.positions_intact[0])
        
        # knee angle
        self.positions[1] = interpolate.splev(percent, self.knee_tck_quick)
        self.positions[1] = math.radians(self.positions[1])

        self.positions_intact[1] = interpolate.splev(percent_shifted, self.knee_tck_quick)
        self.positions_intact[1] = math.radians(self.positions_intact[1])

        #ankle angle
        self.positions[2] = interpolate.splev(percent, self.ankle_tck_quick)
        self.positions[2] = math.radians(self.positions[2])

        self.positions_intact[2] = interpolate.splev(percent_shifted, self.ankle_tck_quick)
        self.positions_intact[2] = math.radians(self.positions_intact[2])
        
        self.publish_position()

def main():
    node = JetLegGait()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    