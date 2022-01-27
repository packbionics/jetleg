import sys
import termios
import tty
import time
import threading
from queue import Queue

if sys.platform == 'win32':
    import msvcrt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

msg = """
        JetLeg Teleoperation
        ----------------------
        Moving around:
            a   d
        z/x : Increase/Decrease max force
        c   : Recenter cart
        anything else: stop
        CTRL-C to quit
      """

effort_key_bindings = {'a': 1.0, 'd': -1.0}
effort_delta_key_bindings = {'z': 1.0, 'x': -1.0}

effort_multiplier = 1.0

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
                node.get_logger().info(key)
                if key in effort_key_bindings:
                    node.effort = node.effort_multiplier * effort_key_bindings[key]
                elif key in effort_delta_key_bindings:
                    node.effort = 0.0
                    node.effort_multiplier += node.delta_effort * effort_delta_key_bindings[key]
                    continue
                elif key == '\x03':
                    break
        
        node.publish_effort()
        time.sleep(1/30.0)

exit_signal = threading.Event()
k_queue = Queue()

class JetLegTeleop(Node):
    
    def __init__(self):
        rclpy.init()
        super().__init__('jetleg_teleop_key')

        self.msg = Float64()

        self.effort_multiplier = 50.0

        self.max_effort = 100.0
        self.delta_effort = 1
        self.effort = 0.0

        self.knee_joint_effort_topic = '/knee_joint_effort_controller/command'
        self.ankle_joint_effort_topic = '/ankle_joint_effort_controller/command'
        self.gantry_to_mount_effort_topic = '/gantry_to_mount_effort_controller/command'

        self.knee_joint_effort_publisher = self.create_publisher(Float64,
                                               self.knee_joint_effort_topic,
                                               10)
        self.ankle_joint_effort_publisher = self.create_publisher(Float64,
                                               self.ankle_joint_effort_topic,
                                               10)
        self.gantry_to_mount_effort_publisher = self.create_publisher(Float64,
                                               self.gantry_to_mount_effort_topic,
                                               10)

    def publish_effort(self):
        self.msg.data = self.effort

        self.knee_joint_effort_publisher.publish(self.msg)
        self.get_logger().info('Publishing: "%s" to topic: "%s"\n' % (self.msg.data, self.knee_joint_effort_topic))

        

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