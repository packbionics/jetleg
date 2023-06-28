#!/usr/bin/env python3

import math
from datetime import datetime

from array import array
import numpy as np
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from scipy import interpolate

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

        
class JetLegGait(Node):
    
    #defines positions data
    #postions: [hip, knee, ankle]
    def __init__(self):
        super().__init__('jetleg_gait_generator')

        self.positions = np.array([0.0,0.0,0.0])
        self.positions_intact = np.array([0.0,0.0,0.0])
        
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_control', qos_profile_system_default)
        self.follow_trajectory_client = ActionClient(self, FollowJointTrajectory, 'leg_controller/follow_joint_trajectory')

        self.x_points = np.linspace(-0.5,1.5,21)
        self.knee_setpoints_quick = np.array([16,38,60,55,18,8,18,20,12,10,16,38,60,55,18,8,18,20,12,10,16], dtype=np.float32)
        self.knee_tck_quick = interpolate.splrep(self.x_points, self.knee_setpoints_quick)
        self.hip_setpoints_quick = np.array([-16.0,-12.0,4.0,17.6,20.8,20.0,18.4,12.0,2.4,-9.6,-16.0,-12.0,4.0,17.6,20.8,20.0,18.4,12.0,2.4,-9.6,-16.0])
        self.hip_tck_quick = interpolate.splrep(self.x_points, self.hip_setpoints_quick)
        self.ankle_setpoints_quick = np.array([-10, 12, 0, -10, -15, -15, -9, -15, -20, -18, -10, 12, 0, -10, -15, -15,  -9, -15, -20, -18, -10], dtype=np.float32)
        self.ankle_tck_quick = interpolate.splrep(self.x_points, self.ankle_setpoints_quick)
        
        self.time_start = datetime.now()

        GAIT_PUB_PERIOD = 1.0/30.0
        self.gait_timer = self.create_timer(GAIT_PUB_PERIOD, self.pub_cmd)
        
        self.T = 4.0

        leg_joint_trajectory = JetLegGait.gen_joint_trajectory_from(self.hip_setpoints_quick, self.knee_setpoints_quick, self.ankle_setpoints_quick, self.T)

    def publish_position(self):
        names = ('vertical_rail_to_mount', 'knee_joint', 'ankle_joint', 'vertical_rail_to_mount_intact', 'knee_joint_intact', 'ankle_joint_intact')
        positions = (self.positions[0], self.positions[1], self.positions[2], self.positions_intact[0], self.positions_intact[1], self.positions_intact[2])

        joint_state_msg = JointState()

        for i in range(len(names)):
            joint_state_msg.name.append(names[i])
            joint_state_msg.position.append(positions[i])

        self.joint_state_publisher.publish(joint_state_msg)
        
        
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

    def gen_joint_trajectory_from(thigh_setpoints: list, knee_setpoints: list, ankle_setpoints: list, gait_period: float):
        if gait_period <= 0:
            raise ValueError(f"Gait period must be a stricly positive value. Given: {gait_period}")
        
        setpoint_counts = [len(thigh_setpoints), len(knee_setpoints), len(ankle_setpoints)]
        if setpoint_counts[0] != setpoint_counts[1] or setpoint_counts[0] != setpoint_counts[2]:
            setpoint_log = f"{setpoint_counts[0]}, {setpoint_counts[1]}, {setpoint_counts[2]}"
            raise ValueError("Number of setpoints must match across all joints. Given: " + setpoint_log)
        
        joint_traj = JointTrajectory()

        joint_traj.joint_names = ['vertical_rail_to_mount', 'knee_joint', 'ankle_joint']

        # number of intervals is the number of setpoints less 1
        time_per_interval = (len(thigh_setpoints) - 1) / gait_period

        for i in range(setpoint_counts[0]):
            time_from_start = time_per_interval * i
            traj_point = JointTrajectoryPoint()

            traj_point.time_from_start.sec = int(time_from_start)
            traj_point.time_from_start.nanosec = int((time_from_start % 1) * 1e9)
            traj_point.positions = array('d', np.radians([thigh_setpoints[i], knee_setpoints[i], ankle_setpoints[i]]))

            joint_traj.points.append(traj_point)

        return joint_traj


def main():
    rclpy.init()
    node = JetLegGait()

    try:
        while True:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exiting spin loop...")

    # clean up ROS 2 resources
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()