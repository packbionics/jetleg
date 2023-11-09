
#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from jetleg_control.classifier import RuleBasedClassifier
from jetleg_control.rule_list import RuleList
from jetleg_control.gait_mode import GaitMode, GaitPhase
from jetleg_control.data import SensorData
from std_msgs.msg import Float64

class ClassifierNode(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Int32, 'topic', 10)
        self.subscriber = self.create_subscription(Float64, 'sensor_data', self.sub_callback, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.List = RuleList()
        self.gaitPhase = GaitPhase()
        StayCase = lambda currentPhase, recentInfo: currentPhase == self.gaitPhase and recentInfo.imu_mean < 0
        AdvanceCase = lambda currentPhase, recentInfo: currentPhase == self.gaitPhase and recentInfo.imu_mean >= 0
        self.gaitMode = GaitMode()
        self.List.add_rule(StayCase,(0,0))
        self.List.add_rule(AdvanceCase,(0,1))
        self.List2 = [self.gaitMode]
        self.RBC = RuleBasedClassifier(self.List, self.List2, self.gaitPhase)
        self.currentData = None



    def timer_callback(self):
        if self.currentData is None: 
            return

        msg = Int32()
        msg.data = self.RBC.classify(sensor_data=self.currentData)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


    def sub_callback(self, msg):
        self.currentData = SensorData(msg.data)





def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ClassifierNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()