from jetleg_control.machine_learning.agent import Agent
from jetleg_control.machine_learning.model import Linear_QNet
from jetleg_control.machine_learning.dqn import DQN

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
        self.trainer = DQN(self.model, lr=self.DEFAULT_LR, gamma=self.gamma)

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
