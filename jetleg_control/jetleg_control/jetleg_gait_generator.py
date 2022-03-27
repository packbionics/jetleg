import math
from re import S
import sys
import termios
import threading
import time
import tty
from datetime import datetime
from queue import Queue

import numpy as np
import rclpy
from rclpy.node import Node
from scipy import interpolate
from std_msgs.msg import Float64
        

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

        self.knee_joint_position_topic = '/knee_joint_position_controller/command'
        self.ankle_joint_position_topic = '/ankle_joint_position_controller/command'
        self.gantry_to_mount_position_topic = '/vertical_rail_to_mount_position_controller/command'
        
        self.knee_intact_joint_position_topic = '/knee_intact_joint_position_controller/command'
        self.ankle_intact_joint_position_topic = '/ankle_intact_joint_position_controller/command'
        self.gantry_to_mount_intact_position_topic = '/vertical_rail_to_mount_intact_position_controller/command'
        
        self.wheel_fore_left_effort_topic = '/wheel_fore_left_effort_controller/command'
        self.wheel_fore_right_effort_topic = '/wheel_fore_right_effort_controller/command'
        self.wheel_hind_left_effort_topic = '/wheel_hind_left_effort_controller/command'
        self.wheel_hind_right_effort_topic = '/wheel_hind_right_effort_controller/command'
        

        self.knee_joint_position_publisher = self.create_publisher(Float64,
                                               self.knee_joint_position_topic,
                                               10)
        self.ankle_joint_position_publisher = self.create_publisher(Float64,
                                               self.ankle_joint_position_topic,
                                               10)
        self.hip_joint_position_publisher = self.create_publisher(Float64,
                                               self.gantry_to_mount_position_topic,
                                               10)
        
        self.knee_intact_joint_position_publisher = self.create_publisher(Float64,
                                               self.knee_intact_joint_position_topic,
                                               10)
        self.ankle_intact_joint_position_publisher = self.create_publisher(Float64,
                                               self.ankle_intact_joint_position_topic,
                                               10)
        self.hip_intact_joint_position_publisher = self.create_publisher(Float64,
                                               self.gantry_to_mount_intact_position_topic,
                                               10)
        
        self.wheel_fore_left_effort_publisher = self.create_publisher(Float64,
                                                self.wheel_fore_left_effort_topic,
                                                10)
        
        self.wheel_fore_right_effort_publisher = self.create_publisher(Float64,
                                                self.wheel_fore_right_effort_topic,
                                                10)
        
        self.wheel_hind_left_effort_publisher = self.create_publisher(Float64,
                                                self.wheel_hind_left_effort_topic,
                                                10)
        
        self.wheel_hind_right_effort_publisher = self.create_publisher(Float64,
                                                self.wheel_hind_right_effort_topic,
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
        """
        self.knee_setpoints_comfort = [8,18,20,12,10,16,38,60,55,18]
        self.knee_setpoints_comfort = np.array(self.knee_setpoints_comfort[5:] + self.knee_setpoints_comfort + self.knee_setpoints_comfort[:6])
        self.knee_tck_comfort = interpolate.splrep(self.x_points, self.knee_setpoints_comfort)
        self.hip_setpoints_comfort = [8,18,20,12,10,16,38,60,55,18]
        self.hip_setpoints_comfort = np.array(self.hip_setpoints_comfort[5:] + self.hip_setpoints_comfort + self.hip_setpoints_comfort[:6]) - 20
        self.hip_tck_comfort = interpolate.splrep(self.x_points, self.hip_setpoints_comfort)"""
        
        gait_pub_period = 1.0/30.0
        #self.gait_timer = self.create_timer(gait_pub_period, self.pub_cmd)
        self.wheel_timer = self.create_timer(1.0/60.0, self.publish_effort)
        
        self.T = 4.0

    def publish_position(self):
        hip_msg = Float64()
        knee_msg = Float64()
        ankle_msg = Float64()
        hip_msg_intact = Float64()
        knee_msg_intact = Float64()
        ankle_msg_intact = Float64()
        
        hip_msg.data = self.positions[0]
        knee_msg.data = self.positions[1]
        ankle_msg.data = self.positions[2]
        
        hip_msg_intact.data = self.positions_intact[0]
        knee_msg_intact.data = self.positions_intact[1]
        ankle_msg_intact.data = self.positions_intact[2]

        self.hip_joint_position_publisher.publish(hip_msg)
        self.knee_joint_position_publisher.publish(knee_msg)
        self.ankle_joint_position_publisher.publish(ankle_msg)
        
        self.hip_intact_joint_position_publisher.publish(hip_msg_intact)
        self.knee_intact_joint_position_publisher.publish(knee_msg_intact)
        self.ankle_intact_joint_position_publisher.publish(ankle_msg_intact)   
        
    def publish_effort(self):
        wheel_fore_left_effort_msg = Float64()
        wheel_fore_right_effort_msg = Float64()
        wheel_hind_left_effort_msg = Float64()
        wheel_hind_right_effort_msg = Float64()
        
        wheel_fore_left_effort_msg.data = 10.0
        wheel_fore_right_effort_msg.data = 10.0
        wheel_hind_left_effort_msg.data = 10.0
        wheel_hind_right_effort_msg.data = 10.0
        
        self.wheel_fore_left_effort_publisher.publish(wheel_fore_left_effort_msg)
        self.wheel_fore_right_effort_publisher.publish(wheel_fore_right_effort_msg)
        self.wheel_hind_left_effort_publisher.publish(wheel_hind_left_effort_msg)
        self.wheel_hind_right_effort_publisher.publish(wheel_hind_right_effort_msg)
        self.get_logger().info("Publishing effort")
        
    #process input keys, and updates positions array of node. Then, calls node's publish method.
    def pub_cmd(self):
        #TO DO: add a speed variable
        current_time = datetime.now() - time_start
        current_time = current_time.seconds + current_time.microseconds / 1000000.0
        
        percent = math.fmod(current_time,self.T) /self.T
        percent_shifted = math.fmod(current_time +self.T / 2,self.T) /self.T
        
        # hip angle
        self.positions[0] = - interpolate.splev(percent, self.hip_tck_quick) * math.pi / 180.0
        self.positions_intact[0] = - interpolate.splev(percent_shifted, self.hip_tck_quick) * math.pi / 180.0
        
        # knee angle
        self.positions[1] = interpolate.splev(percent, self.knee_tck_quick) * math.pi / 180.0
        self.positions_intact[1] = interpolate.splev(percent_shifted, self.knee_tck_quick) * math.pi / 180.0

        #ankle angle
        self.positions[2] = interpolate.splev(percent, self.ankle_tck_quick) * math.pi / 180.0
        self.positions_intact[2] = interpolate.splev(percent_shifted, self.ankle_tck_quick) * math.pi / 180.0
        
        self.publish_position()

time_start = datetime.now()

def main():
    node = JetLegGait()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    