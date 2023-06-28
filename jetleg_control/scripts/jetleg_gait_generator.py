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

        self.hip_setpoints_quick = np.array([16.0,12.0,-4.0,-17.6,-20.8,-20.0,-18.4,-12.0,-2.4,9.6,16.0,12.0,-4.0,-17.6,-20.8,-20.0,-18.4,-12.0,-2.4,9.6,16.0], dtype=np.float32)
        self.knee_setpoints_quick = np.array([16,38,60,55,18,8,18,20,12,10,16,38,60,55,18,8,18,20,12,10,16], dtype=np.float32)
        self.ankle_setpoints_quick = np.array([-10, 12, 0, -10, -15, -15, -9, -15, -20, -18, -10, 12, 0, -10, -15, -15,  -9, -15, -20, -18, -10], dtype=np.float32)
                
        self.T = 8.0

        self.leg_joint_trajectory = JetLegGait.gen_joint_trajectory_from(self.hip_setpoints_quick, self.knee_setpoints_quick, self.ankle_setpoints_quick, self.T)

    def send_goal(self, trajectory: JointTrajectory):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.follow_trajectory_client.wait_for_server()

        self.future = self.follow_trajectory_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
    
        self.future.add_done_callback(self.goal_response_callback)

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
        # self.get_logger().info('Received feedback: {0}'.format(repr(feedback)))
        # self.get_logger().info('\n\n')

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
        time_per_interval = gait_period / (len(thigh_setpoints) - 1)

        for i in range(setpoint_counts[0]):
            time_from_start = time_per_interval * i
            traj_point = JointTrajectoryPoint()

            print(time_from_start)

            traj_point.time_from_start.sec = int(time_from_start)
            traj_point.time_from_start.nanosec = int((time_from_start % 1) * 1e9)
            traj_point.positions = array('d', np.radians([thigh_setpoints[i], knee_setpoints[i], ankle_setpoints[i]]))

            joint_traj.points.append(traj_point)

        return joint_traj


def main():
    rclpy.init()
    node = JetLegGait()

    node.send_goal(node.leg_joint_trajectory)
    rclpy.spin(node)

    node.get_logger().info('Close program')

    # clean up ROS 2 resources
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()