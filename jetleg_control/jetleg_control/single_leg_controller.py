import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

class SingleLegController(Node):

    def __init__(self):
        super().__init__('single_leg_controller')

        self.jointStateSubscriber = self.create_subscription(JointState, 'joint_states', self.jointStateCallback, 10)
        self.get_logger().info('Created jointStateSubscriber')

        self.poseStampedSubscriber = self.create_subscription(PoseStamped, 'link_states', self.poseStampedCallback, 10)
        self.get_logger().info('Created poseStampedSubscriber')

    def jointStateCallback(self, message):
        self.get_logger().info(str(message.name))

    def poseStampedCallback(self, message):
        self.get_logger().info(str(message))


def main():
    rclpy.init()
    node = SingleLegController()

    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()