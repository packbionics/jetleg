import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from std_srvs.srv import Empty

from jetleg_control.leg_agent import LegAgent

from jetleg_interfaces.action import LegAction

import numpy as np

class StateLegActor(Node):
    """
    Contains state and behavior necessary to control the Jetleg system in Pybullet simulation
    """

    def __init__(self):
        """
        Constructs an instance of StateLegActor and initializes its fields
        """

        super().__init__('state_leg_actor')

        # Construct a learning agent to train for leg stability
        self.leg_agent = LegAgent()

        # Initialize an action client to communicate with action server
        self.action_client = ActionClient(self, LegAction, 'leg_control')

        # Initialize service client to reset simulation after the leg falls down
        self.reset_sim_client = self.create_client(Empty, 'reset_simulation')

        # Store state/action pair
        self.old_state = []
        self.action = []

        # Indicates the end of a training epoch
        self.done = False

        # Indicates if the agent is training
        self.training = True

        # Reward at end of epoch
        self.highest_reward = 0
    
    def execute_action(self, action):
        """
        Performs the given action and configures callbacks for receiving feedback and the response from the action server
        """

        goal_msg = LegAction.Goal()
        goal_msg.action = action.tolist()

        self.action = np.array(action, dtype=np.int32)

        self.action_client.wait_for_server()

        future = self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        rclpy.spin_until_future_complete(self, future)

    def goal_response_callback(self, future):
        """
        Determines if the action was accepted or rejected.
        If accepted, a callback processes the result
        """

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Trains the leg agent based on the results of the associated action
        """

        # Retrieves the result of the associated action
        result = future.result().result

        # self.get_logger().info('Result: state={0} reward={1} done={2}'.format(result.state, result.reward, result.done))

        if len(self.old_state) == 25 and len(result.state) == 25 and self.training:
            # Perform train step after each action
            self.leg_agent.train_short_memory(self.old_state, self.action, result.reward, result.state, result.done)
            self.leg_agent.remember(self.old_state, self.action, result.reward, result.state, result.done)

        self.old_state = result.state

        if result.done:
            self.done = result.done  
            self.highest_reward = result.reward

    def feedback_callback(self, feedback_msg):
        """
        Processes feedback received by action server in response to an action
        """

        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.state))

    def run_epoch(self, epoch):

        # Perform actions until leg falls down
        while not self.done:
            self.test_counter = 0

            # Determine action based on current state
            action = self.leg_agent.get_action(self.old_state)

            # Process action and retrieve results
            self.execute_action(action)
        
        self.get_logger().info('Leg has fallen down!')

        # Process end-of-epoch training and reset simulation
        cost = self.process_end_epoch()

        epoch_reward = self.highest_reward
        self.highest_reward = 0
        
        # Increment for next epoch
        return epoch + 1, epoch_reward, cost


    def process_end_epoch(self):
        cost = 0
        if self.training:
            # Train leg agent after finishing one training epoch
            cost = self.leg_agent.train_long_memory()

        # Reset simulation and prepare for another epoch
        self.get_logger().info('Sending reset request...')
        reset_future = self.reset_sim()

        rclpy.spin_until_future_complete(self, reset_future)
        self.done = False

        return cost

    def reset_sim(self):
        req = Empty.Request()
        
        future = self.reset_sim_client.call_async(req)
        return future