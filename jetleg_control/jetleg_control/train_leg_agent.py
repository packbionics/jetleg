import rclpy
import torch

from jetleg_control.leg_actor import LegActor

def main():
    print(f"Is CUDA supported by this system? {torch.cuda.is_available()}")
    print(f"CUDA version: {torch.version.cuda}")
    
    # Storing ID of current CUDA device
    try:
        cuda_id = torch.cuda.current_device()
        print(f"ID of current CUDA device: {torch.cuda.current_device()}")
                
        print(f"Name of current CUDA device: {torch.cuda.get_device_name(cuda_id)}")
    except RuntimeError as ex:
        print(ex)
        print('Training will utilize the CPU of the system if available')

    # Initialize ROS 2
    rclpy.init()

    # Construct a client node for executing the actions of the agent
    action_node = LegActor()

    # Track the performance of the model
    scores = []
    costs = []

    high_score = 0
    low_cost = []

    # Number of training epochs to perform
    num_epochs = 300
    current_epoch = 1
    while current_epoch <= num_epochs:
        # Run the current epoch
        current_epoch, score, cost = action_node.run_epoch(current_epoch)
        action_node.get_logger().info('Epoch: ' + str(current_epoch))

        if score > high_score:
            action_node.leg_agent.network.save('high_score_tmp.pth')
            high_score = score
        if len(low_cost) == 0:
            low_cost.append(cost)
            action_node.leg_agent.network.save('low_cost_tmp.pth')
        elif cost < low_cost[0]:
            low_cost[0] = cost
            action_node.leg_agent.network.save('low_cost_tmp.pth')

        scores.append(score)
        costs.append(cost)
    
    # Clean up resources
    action_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()