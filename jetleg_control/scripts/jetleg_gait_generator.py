#!/usr/bin/env python3

from array import array
import numpy as np
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

        
class JetLegGait(Node):
    
    #defines positions data
    #postions: [hip, knee, ankle]
    def __init__(self):
        super().__init__('jetleg_gait_generator')
        
        self.follow_trajectory_client = ActionClient(self, FollowJointTrajectory, 'leg_controller/follow_joint_trajectory')
        self.follow_trajectory_client_intact = ActionClient(self, FollowJointTrajectory, 'leg_intact_controller/follow_joint_trajectory')

        self.hip_setpoints_quick = np.array([40,38,30,18,3,-5,0,20,37,41], dtype=np.float32)
        self.hip_setpoints_quick = -(self.hip_setpoints_quick - 15) * 0.8

        self.knee_setpoints_quick = np.array([8,18,20,12,10,16,38,60,55,18], dtype=np.float32)

        self.ankle_setpoints_quick = np.array([95,81,75,70,72,80,102,90,80,75], dtype=np.float32)
        self.ankle_setpoints_quick = self.ankle_setpoints_quick - 90
                
        self.T = 4.0

        joint_names = ['vertical_rail_to_mount', 'knee_joint', 'ankle_joint']
        joint_names_intact = ['vertical_rail_to_mount_intact', 'knee_joint_intact', 'ankle_joint_intact']

        self.leg_joint_trajectory = JetLegGait.gen_joint_trajectory_from(joint_names, self.hip_setpoints_quick, self.knee_setpoints_quick, self.ankle_setpoints_quick, self.T)
        
        self.hip_setpoints_quick = np.concatenate((self.hip_setpoints_quick[5:], self.hip_setpoints_quick[:5]))
        self.knee_setpoints_quick = np.concatenate((self.knee_setpoints_quick[5:], self.knee_setpoints_quick[:5]))
        self.ankle_setpoints_quick = np.concatenate((self.ankle_setpoints_quick[5:], self.ankle_setpoints_quick[:5]))
        
        self.leg_intact_joint_trajectory = JetLegGait.gen_joint_trajectory_from(joint_names_intact, self.hip_setpoints_quick, self.knee_setpoints_quick, self.ankle_setpoints_quick, self.T)

    def send_goal(self, trajectory: JointTrajectory, action_client: ActionClient):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        action_client.wait_for_server()

        return action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Complete')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def gen_joint_trajectory_from(joint_names: list, thigh_setpoints: list, knee_setpoints: list, ankle_setpoints: list, gait_period: float):
        if gait_period <= 0:
            raise ValueError(f"Gait period must be a stricly positive value. Given: {gait_period}")
        
        setpoint_counts = [len(thigh_setpoints), len(knee_setpoints), len(ankle_setpoints)]
        if setpoint_counts[0] != setpoint_counts[1] or setpoint_counts[0] != setpoint_counts[2]:
            setpoint_log = f"{setpoint_counts[0]}, {setpoint_counts[1]}, {setpoint_counts[2]}"
            raise ValueError("Number of setpoints must match across all joints. Given: " + setpoint_log)
        
        joint_traj = JointTrajectory()

        # joint_traj.joint_names = ['vertical_rail_to_mount', 'knee_joint', 'ankle_joint']
        joint_traj.joint_names = joint_names

        # number of intervals is the number of setpoints less 1
        time_per_interval = gait_period / (len(thigh_setpoints) - 1)

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

    while True:

        future = node.send_goal(node.leg_joint_trajectory, node.follow_trajectory_client)
        rclpy.spin_until_future_complete(node, future=future)

        future1 = node.send_goal(node.leg_intact_joint_trajectory, node.follow_trajectory_client_intact)
        rclpy.spin_until_future_complete(node, future=future1)

        goal_handle = future.result()
        future = goal_handle.get_result_async()

        goal_handle = future1.result()
        future1 = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(node, future=future)
        rclpy.spin_until_future_complete(node, future=future1)

    node.get_logger().info('Close program')

    # clean up ROS 2 resources
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()