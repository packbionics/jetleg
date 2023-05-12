import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from std_srvs.srv import Empty

import numpy as np

from jetleg_interfaces.action import LegAction
from jetleg_control.machine_learning.agent import Agent


class LegActor(Node):
    """
    Contains state and behavior necessary to control the Jetleg system in Pybullet simulation
    """

    # Used to calculate number of state variables
    NUM_STATE_VAR_PER_LINK = 7
    NUM_LINKS = 1

    NUM_STATE_VAR_PER_JOINT = 2
    NUM_JOINTS = 2

    OUTPUT_VARS_NUM = 5

    # Number of input variables for learning model
    INPUT_VARS_NUM = (NUM_STATE_VAR_PER_JOINT*NUM_JOINTS) + (NUM_STATE_VAR_PER_LINK*NUM_LINKS)

    def __init__(self):
        """
        Constructs an instance of StateLegActor and initializes its fields
        """

        super().__init__('state_leg_actor')

        # Construct a learning agent to train for leg stability
        layers = [LegActor.INPUT_VARS_NUM, 1024, 512, LegActor.OUTPUT_VARS_NUM]
        self.leg_agent = Agent(layers=layers)

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

        joint_indices = (0, LegActor.NUM_STATE_VAR_PER_JOINT * LegActor.NUM_JOINTS)
        link_indices = (joint_indices[1], joint_indices[1] + LegActor.NUM_STATE_VAR_PER_LINK * LegActor.NUM_LINKS)

        num_state_vars = LegActor.NUM_JOINTS * LegActor.NUM_STATE_VAR_PER_JOINT + LegActor.NUM_LINKS * LegActor.NUM_STATE_VAR_PER_LINK

        # Retrieves the result of the associated action
        result = future.result().result
        
        processed_state = result.state[joint_indices[0]:joint_indices[1]]
        processed_state = processed_state + result.state[link_indices[0]:link_indices[1]]

        result.state = processed_state

        if len(self.old_state) == num_state_vars and len(result.state) == num_state_vars and self.training:
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