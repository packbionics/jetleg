import sys
import torch
import random
import numpy as np
from collections import deque
from jetleg_control.model import Linear_QNet, QTrainer

BATCH_SIZE = 5000
MAX_MEMORY = 100 * BATCH_SIZE
LR = 0.001

class Agent:
    
    def __init__(self):
        self.n_games = 0
        self.epsilon = 200 # randomness
        self.gamma = 0.90 # discount rate
        self.memory = deque(maxlen=MAX_MEMORY)
        self.model = Linear_QNet([11, 1024, 3])
        self.trainer = QTrainer(self.model, lr=LR, gamma=self.gamma)
    
    def get_state(self, environment):
        return 0

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done)) # popleft if MAX_MEMORY is reached

    def train_long_memory(self):
        if len(self.memory) > BATCH_SIZE:
            mini_sample = random.sample(self.memory, BATCH_SIZE) # list of tuples
        else:
            mini_sample = self.memory

        cost = 0

        states, actions, rewards, next_states, dones = zip(*mini_sample)
        cost += self.trainer.train_step(np.array(states), np.array(actions), np.array(rewards), np.array(next_states), np.array(dones))
        
        return cost

    def train_short_memory(self, state, action, reward, next_state, done):
        self.trainer.train_step(state, action, reward, next_state, done)

    def get_action(self, state):
        # random moves: tradeoff between exploration / exploitation
        # self.epsilon = 200 - self.n_games
        final_move = [0, 0, 0]
        if random.randint(0, 200) < self.epsilon:
            move = random.randint(0, 2)
            final_move[move] = 1
        else:
            state0 = torch.tensor(state, dtype=torch.float)
            prediction = self.model.forward(state0)
            move = torch.argmax(prediction).item()
            final_move[move] = 1
        
        return np.array(final_move, dtype=int)
