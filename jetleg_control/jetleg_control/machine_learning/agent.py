import random

import torch
import numpy as np

from jetleg_control.machine_learning.dqn import DQN
from jetleg_control.machine_learning.replay_buffer import ReplayBuffer

class Agent:
    
    def __init__(self, layers):
        self.DEFAULT_BATCH_SIZE = 5000
        self.DEFAULT_MAX_MEMORY = 500000
        self.DEFAULT_LR = 0.001

        self.n_games = 0
        self.gamma = 0.90 # discount rate
        self.batch_size = self.DEFAULT_BATCH_SIZE
        self.memory = ReplayBuffer(maxlen=self.DEFAULT_MAX_MEMORY)
        self.network = DQN(layers=layers, lr=self.DEFAULT_LR, gamma=self.gamma)

        # Making the code device-agnostic
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.network.model.to(device)

    def remember(self, state, action, reward, next_state, done):
        self.memory.add((state, action, reward, next_state, done)) # popleft if MAX_MEMORY is reached

    def train_long_memory(self):
        mini_sample = self.memory.get_training_samples(size=self.batch_size)

        # end training prematurely if there are no memory samples
        if len(mini_sample) == 0:
            return 0

        # extract data for training
        states, actions, rewards, next_states, dones = zip(*mini_sample)
        # train the model and retrieve the sum of the loss function
        cost = self.network.train_step(np.array(states), np.array(actions), np.array(rewards), np.array(next_states), np.array(dones))
    
        return cost

    def train_short_memory(self, state, action, reward, next_state, done):
        self.network.train_step(state, action, reward, next_state, done)

    def get_action(self, state):
        return self.network.get_action(state)
