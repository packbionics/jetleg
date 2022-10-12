import single_leg_controller
from leg_controller import Leg_Controller
import rclpy

def main():

    rclpy.init()
    node = single_leg_controller.SingleLegController()
    leg = Leg_Controller()

    for i in range(1, 1000):
        node.spin_once()
        state = leg.get_state(node)
        action = leg.get_action(state)
        leg.remember(state, 0, 0, 0, 0, 0)
        leg.train_short_memory(state, 0, 0, 0, 0)
        leg.train_long_memory()


