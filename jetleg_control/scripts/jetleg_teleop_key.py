#!/usr/bin/env python3

import sys
import termios
import tty
import time
import threading
import numpy as np
from queue import Queue

if sys.platform == 'win32':
    import msvcrt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

msg = """
        JetLeg Teleoperation
        ----------------------
        Moving the leg:
        w/s : Move hip
        a/d : Move knee
        q/e : Move ankle
        z/c : Move mount
        
        j/l : Move faster/slower
        k : Reset to zero
    
        CTRL-C to quit
      """

position_key_bindings = {
        'w': np.array([1.0, 0.0, 0.0, 0.0]),
        's': np.array([-1.0, 0.0, 0.0, 0.0]),
        'a': np.array([0.0, 1.0, 0.0, 0.0]),
        'd': np.array([0.0, -1.0, 0.0, 0.0]),
        'q': np.array([0.0, 0.0, 1.0, 0.0]),
        'e': np.array([0.0, 0.0, -1.0, 0.0]),
        'z': np.array([0.0, 0.0, 0.0, 1.0]),
        'c': np.array([0.0, 0.0, 0.0, -1.0])
    }
position_delta_key_bindings = {'j': -1.0, 'l': 1.0}

def get_terminal_settings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def get_key(settings, key):
    while not exit_signal.is_set():
        if sys.platform == 'win32':
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
            k_queue.put(key)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        time.sleep(0.1)

def pub_cmd(node):
    while not exit_signal.is_set():
        if not k_queue.empty():
            key = k_queue.get()
            if key:
                if key in position_key_bindings:
                    node.positions += node.position_multiplier * position_key_bindings[key]
                    node.positions = np.minimum(
                        np.maximum(node.positions, node.joint_limit_min), 
                        node.joint_limit_max)
                    #node.get_logger().info("positions: {} degs".format(node.positions * 180.0 / np.pi))
                    node.get_logger().info("positions: {} degs".format(node.positions))
                elif key in position_delta_key_bindings:
                    node.position_multiplier += node.delta_position * position_delta_key_bindings[key]
                    node.position_multiplier = min(max(node.position_multiplier, 0.0), np.pi / 6.0)
                    node.get_logger().info("new position increment: {} degs".format(node.position_multiplier * 180.0 / np.pi))
                    continue
                elif key == 'k':
                    node.positions = np.zeros(4)
                    node.position_multiplier = np.pi/180.0
                    node.get_logger().info("resetting positions")
                elif key == '\x03':
                    exit_signal.set()
                    break
        
        node.publish_position()
        time.sleep(1/30.0)

exit_signal = threading.Event()
k_queue = Queue()

class JetLegTeleop(Node):
    
    def __init__(self):
        rclpy.init()
        super().__init__('jetleg_teleop_key')

        self.position_multiplier = np.pi / 180.0

        self.delta_position = 2 * np.pi/180.0
        self.positions = np.array([0.0,0.0,0.0,0.0])
        self.joint_limit_max = np.array([np.pi/4.0, 3*np.pi/4.0, np.pi/4.0, 0.6])
        self.joint_limit_min = np.array([-np.pi/4.0, 0.0, -np.pi/8.0, 0.0])

        self.wheel_joint_position_topics = ['/wheel_fore_left_position_controller/command',
                                            '/wheel_fore_right_position_controller/command',
                                            '/wheel_hind_left_position_controller/command',
                                            '/wheel_hind_right_position_controller/command']

        self.knee_joint_position_topic = '/knee_joint_position_controller/command'
        self.ankle_joint_position_topic = '/ankle_joint_position_controller/command'
        self.gantry_to_mount_position_topic = '/vertical_rail_to_mount_position_controller/command'
        self.mount_to_leg_position_topic = '/mount_to_jetleg_position_controller/command'

        self.knee_joint_position_publisher = self.create_publisher(Float64,
                                               self.knee_joint_position_topic,
                                               10)
        self.ankle_joint_position_publisher = self.create_publisher(Float64,
                                               self.ankle_joint_position_topic,
                                               10)
        self.hip_joint_position_publisher = self.create_publisher(Float64,
                                               self.gantry_to_mount_position_topic,
                                               10)
        self.mount_to_leg_position_publisher = self.create_publisher(Float64,
                                               self.mount_to_leg_position_topic,
                                               10)

    def publish_position(self):
        hip_msg = Float64()
        knee_msg = Float64()
        ankle_msg = Float64()
        mount_msg = Float64()
        
        hip_msg.data = self.positions[0]
        knee_msg.data = self.positions[1]
        ankle_msg.data = self.positions[2]
        mount_msg.data = self.positions[3]

        self.hip_joint_position_publisher.publish(hip_msg)
        self.knee_joint_position_publisher.publish(knee_msg)
        self.ankle_joint_position_publisher.publish(ankle_msg)   
        self.mount_to_leg_position_publisher.publish(mount_msg)     

def main():
    settings = get_terminal_settings()

    node = JetLegTeleop()
    print(msg)

    t1 = threading.Thread(target=get_key, args=(settings,k_queue))
    t2 = threading.Thread(target=pub_cmd, args=(node,))

    t1.start()
    t2.start()

    try:
        while not exit_signal.is_set():  # enable children threads to exit the main thread, too
            time.sleep(0.1)  # let it breathe a little
    except KeyboardInterrupt:  # on keyboard interrupt...
        exit_signal.set() 

    t1.join()
    t2.join()

    rclpy.shutdown()

if __name__ == '__main__':
    main()