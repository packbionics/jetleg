import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import JointState

class SingleLegController(Node):

    def __init__(self):
        super().__init__('single_leg_controller')

        self.subscriber = self.create_subscription(JointState, 'joint_states', self.callback, 10)
        self.get_logger().info('Created subscriber')

    def callback(self, message):
        self.get_logger().info(str(message.name))


def main():
    rclpy.init()
    node = SingleLegController()

    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()