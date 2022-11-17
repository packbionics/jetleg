import jetleg_control.agent as agent
from jetleg_control.agent import Agent
from jetleg_control.model import Linear_QNet, QTrainer

import numpy as np
import torch
import random

class LegAgent(Agent):
    """
    Serves as the agent perceiving and reacting to its environment and state.
    Child class of the Agent class.
    """

    def __init__(self):
        """
        Constructs an instance of the LegAgent
        """

        # Calls the parent constructor
        super().__init__()

        # Used to calculate number of state variables
        self.NUM_STATE_VAR_PER_LINK = 7
        self.NUM_LINKS = 1

        self.NUM_STATE_VAR_PER_JOINT = 2
        self.NUM_JOINTS = 2

        self.OUTPUT_VARS_NUM = 5

        # Number of input variables for learning model
        self.total_num_state_var = (self.NUM_STATE_VAR_PER_JOINT*self.NUM_JOINTS) + (self.NUM_STATE_VAR_PER_LINK*self.NUM_LINKS)

        self.model = Linear_QNet([self.total_num_state_var, 1024, 512, self.OUTPUT_VARS_NUM])
        self.trainer = QTrainer(self.model, lr=agent.LR, gamma=self.gamma)

    def get_state(self, state_reader_node):
        """
        Reads the current state of the leg from the node which subscribes to /joint_states and /link_states.
        Converts from ROS Msg types to a single numpy array of length n where n is the number of state variables.
        """

        state = np.zeros(self.total_num_state_var)

        # Reads joint position and joint velocity
        for i in range(self.NUM_JOINTS):
            state[i * self.NUM_STATE_VAR_PER_JOINT] = state_reader_node.last_joint_state.message.position[i]
            state[i * self.NUM_STATE_VAR_PER_JOINT + 1] = state_reader_node.last_joint_state.message.velocity[i]

        # Reads the center of mass of each link in the leg
        for i in range(self.NUM_LINKS):

            # Reads the XYZ position of each link
            state[self.NUM_JOINTS * self.NUM_STATE_VAR_PER_JOINT + i * self.NUM_STATE_VAR_PER_LINK] = state_reader_node.last_link_state.pose.point.x
            state[self.NUM_JOINTS * self.NUM_STATE_VAR_PER_JOINT + i * self.NUM_STATE_VAR_PER_LINK + 1] = state_reader_node.last_link_state.pose.point.y
            state[self.NUM_JOINTS * self.NUM_STATE_VAR_PER_JOINT + i * self.NUM_STATE_VAR_PER_LINK + 2] = state_reader_node.last_link_state.pose.point.z

            # Reads the orientation of each link as a quaternion
            state[self.NUM_JOINTS * self.NUM_STATE_VAR_PER_JOINT + i * self.NUM_STATE_VAR_PER_LINK + 3] = state_reader_node.last_link_state.pose.orientation.x
            state[self.NUM_JOINTS * self.NUM_STATE_VAR_PER_JOINT + i * self.NUM_STATE_VAR_PER_LINK + 4] = state_reader_node.last_link_state.pose.orientation.y
            state[self.NUM_JOINTS * self.NUM_STATE_VAR_PER_JOINT + i * self.NUM_STATE_VAR_PER_LINK + 5] = state_reader_node.last_link_state.pose.orientation.z
            state[self.NUM_JOINTS * self.NUM_STATE_VAR_PER_JOINT + i * self.NUM_STATE_VAR_PER_LINK + 6] = state_reader_node.last_link_state.pose.orientation.w

        return state

    def get_action(self,state):
        # random moves: tradeoff between exploration / exploitation
        # self.epsilon = 200 - self.n_games
        final_move = [0] * self.OUTPUT_VARS_NUM
        if random.randint(0, 200) < self.epsilon or len(state) != 25:
            move = random.randint(0, self.OUTPUT_VARS_NUM - 1)
            final_move[move] = 1
        else:
            state0 = torch.tensor(state, dtype=torch.float)
            prediction = self.model.forward(state0)
            move = torch.argmax(prediction).item()
            final_move[move] = 1
        
        return np.array(final_move, dtype=np.float32)
