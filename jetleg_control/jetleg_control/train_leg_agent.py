import rclpy

from jetleg_control.state_leg_actor import StateLegActor

def run_epoch(node, epoch):
    # Perform actions until leg falls down
    while not node.done:
        node.test_counter = 0

        # Determine action based on current state
        action = node.leg_agent.get_action(node.old_state)

        # Process action and retrieve results
        future = node.execute_action(action)
        rclpy.spin_until_future_complete(node, future)

    # Increment for next epoch
    return epoch + 1


def main():

    # Initialize ROS 2
    rclpy.init()

    # Construct a client node for executing the actions of the agent
    action_node = StateLegActor()

    # Number of training epochs to perform
    num_epochs = 300
    current_epoch = 1
    while current_epoch <= num_epochs:

        # Run the current epoch
        current_epoch = run_epoch(action_node, current_epoch)

        # action_node.get_logger().info('Training long term memory...')
        # Train leg agent after finishing one training epoch
        action_node.leg_agent.train_long_memory()

        # Reset simulation and prepare for another epoch
        action_node.get_logger().info('Sending reset request...')
        reset_future = action_node.reset_sim()

        rclpy.spin_until_future_complete(action_node, reset_future)
        action_node.done = False

        action_node.get_logger().info('Epoch: ' + str(current_epoch))
    
    # Clean up resources
    action_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()