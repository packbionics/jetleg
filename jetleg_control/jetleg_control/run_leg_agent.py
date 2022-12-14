import rclpy

from jetleg_control.helper import init_subplots, plot
from jetleg_control.leg_actor import LegActor

def main():

    # Initialize ROS 2
    rclpy.init()

    # Construct a client node for executing the actions of the agent
    action_node = LegActor()
    action_node.leg_agent.network.load('low_cost_tmp.pth')

    scores = []

    fig, ax = init_subplots(1, ['Score', 'Epoch'])

    # Number of training epochs to perform
    num_epochs = 300
    current_epoch = 1
    while current_epoch <= num_epochs:
        # Run the current epoch
        current_epoch, score, _ = action_node.run_epoch(current_epoch)
        action_node.get_logger().info('Epoch: ' + str(current_epoch))

        scores.append(score)
        plot([scores], fig, ax)
    
    # Clean up resources
    action_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()