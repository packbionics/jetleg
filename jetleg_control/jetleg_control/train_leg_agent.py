import rclpy

from jetleg_control.state_leg_actor import StateLegActor

def main():

    # Initialize ROS 2
    rclpy.init()

    # Construct a client node for executing the actions of the agent
    action_node = StateLegActor()

    # Number of training epochs to perform
    num_epochs = 200
    for _ in range(num_epochs):

        # Perform actions until leg falls down
        while not action_node.done:
            # Determine action based on current state
            action = action_node.leg_agent.get_action(action_node.old_state)

            # Process action and retrieve results
            future = action_node.execute_action(action)
            rclpy.spin_until_future_complete(action_node, future)

        action_node.get_logger().info('Training long term memory...')
        # Train leg agent after finishing one training epoch
        action_node.leg_agent.train_long_memory()

        # Reset simulation and prepare for another epoch
        action_node.done = False
    
    # Clean up resources
    action_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()