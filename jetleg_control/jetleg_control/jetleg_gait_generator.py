import sys
import termios
import tty
import time
import threading
import numpy as np
import math
from datetime import datetime
from queue import Queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

"""
position_key_bindings = {
        'q': np.array([1.0, 0.0, 0.0]),
        'w': np.array([-1.0, 0.0, 0.0]),
        'a': np.array([0.0, 1.0, 0.0]),
        's': np.array([0.0, -1.0, 0.0]),
        'z': np.array([0.0, 0.0, 1.0]),
        'x': np.array([0.0, 0.0, -1.0])
    }
position_delta_key_bindings = {'d': -1.0, 'f': 1.0}
"""

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

#process input keys, and updates positions array of node. Then, calls node's publish method.
def pub_cmd(node):
    while not exit_signal.is_set():
        #the period is 4 seconds
        #TO DO: add a speed variable
        current_time = datetime.now() - time_start
        #hip angle
        node.positions[0] = math.sin(np.pi/2 * current_time) * np.pi/4 #add function call to calculated needed angle, add Period_Constant later
        
        #knee angle
        node.positions[1] = math.sin(np.pi/2 * current_time) * np.pi/4 + np.pi/4 #add function call to calculated needed angle, offset func.? 
        
        #ankle angle
        node.positions[2] = math.sin(np.pi/2 * current_time) * np.pi * 3/16 + np.pi/16#add function call to calculated needed angle
        
        node.publish_position()
        time.sleep(1/30.0)

exit_signal = threading.Event()
k_queue = Queue()

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

time_start = datetime.now()

def main():

    node = JetLegGait()

    # t1 = threading.Thread(target=get_key, args=(settings,k_queue))
    t2 = threading.Thread(target=pub_cmd, args=(node,))

    #t1.start()
    t2.start()

    try:
        while not exit_signal.is_set():  # enable children threads to exit the main thread, too
            time.sleep(0.1)  # let it breathe a little
    except KeyboardInterrupt:  # on keyboard interrupt...
        exit_signal.set() 

    #t1.join()
    t2.join()

    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    