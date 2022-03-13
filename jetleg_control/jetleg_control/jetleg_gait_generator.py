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
        self.joint_limit_max = np.array([np.pi/4.0, np.pi/2.0, np.pi/4.0])
        self.joint_limit_min = np.array([-np.pi/4.0, 0.0, -np.pi/8.0])

        self.knee_joint_position_topic = '/knee_joint_position_controller/command'
        self.ankle_joint_position_topic = '/ankle_joint_position_controller/command'
        self.gantry_to_mount_position_topic = '/gantry_to_mount_position_controller/command'

        self.knee_joint_position_publisher = self.create_publisher(Float64,
                                               self.knee_joint_position_topic,
                                               10)
        self.ankle_joint_position_publisher = self.create_publisher(Float64,
                                               self.ankle_joint_position_topic,
                                               10)
        self.hip_joint_position_publisher = self.create_publisher(Float64,
                                               self.gantry_to_mount_position_topic,
                                               10)
        
        
        self.x_points = np.linspace(-0.5,1.5,21)
        self.knee_setpoints = [16,38,60,55,18,8,18,20,12,10,16,38,60,55,18,8,18,20,12,10,16]
        self.knee_tck = interpolate.splrep(self.x_points, self.knee_setpoints)
        self.hip_setpoints = [-5,0,20,37,41,40,38,30,18,3,-5,0,20,37,41,40,38,30,18,3,-5]
        self.hip_tck = interpolate.splrep(self.x_points, self.hip_setpoints)
        self.ankle_setpoints = []

    def publish_position(self):
        hip_msg = Float64()
        knee_msg = Float64()
        ankle_msg = Float64()
        
        hip_msg.data = self.positions[0]
        knee_msg.data = self.positions[1]
        ankle_msg.data = self.positions[2]

        self.hip_joint_position_publisher.publish(hip_msg)
        self.knee_joint_position_publisher.publish(knee_msg)
        self.ankle_joint_position_publisher.publish(ankle_msg)   
        
    #process input keys, and updates positions array of node. Then, calls node's publish method.
    def pub_cmd(self, T=2.0):
        while True:
            #the period is 4 seconds
            #TO DO: add a speed variable
            current_time = datetime.now() - time_start
            current_time = current_time.seconds + current_time.microseconds / 1000000.0
            
            percent = math.fmod(current_time, T) / T
            
            # hip angle
            self.positions[0] = interpolate.splev(percent, self.hip_tck) * math.pi / 180.0
            
            # knee angle
            self.positions[1] = interpolate.splev(percent, self.knee_tck) * math.pi / 180.0

            #ankle angle
            self.positions[2] = math.sin(np.pi/2 * current_time) * np.pi * 3/16 + np.pi/16#add function call to calculated needed angle
            
            self.publish_position()

            time.sleep(1/30.0)

time_start = datetime.now()

def main():

    node = JetLegGait()
    node.pub_cmd(T=1.0)

    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    