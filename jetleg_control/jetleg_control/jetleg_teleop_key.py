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
        q/w : Move hip
        a/s : Move knee
        z/x : Move ankle
        
        d/f : Move faster/slower
        c : Reset to zero
    
        CTRL-C to quit
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
                    node.get_logger().info("positions: {} degs".format(node.positions * 180.0 / np.pi))
                elif key in position_delta_key_bindings:
                    node.position_multiplier += node.delta_position * position_delta_key_bindings[key]
                    node.position_multiplier = min(max(node.position_multiplier, 0.0), np.pi / 6.0)
                    node.get_logger().info("new position increment: {} degs".format(node.position_multiplier * 180.0 / np.pi))
                    continue
                elif key == 'c':
                    node.positions = np.zeros(3)
                    node.position_multiplier = np.pi/180.0
                    node.get_logger().info("resetting positions")
                elif key == '\x03':
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

    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()