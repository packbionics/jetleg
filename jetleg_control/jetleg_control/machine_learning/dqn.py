import random
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

import os


class DQN:

    def __init__(self, layers, lr, gamma):
        super().__init__()

        # Initialize an empty list of layers for the neural network
        self.model = nn.Sequential()
        # Add input layer and hidden layers to the neural network
        for i in range(len(layers) - 2):
            linear = nn.Linear(layers[i], layers[i + 1])

            self.model.append(linear)
            self.model.append(nn.ReLU())
        self.model.append(nn.Linear(layers[-2], layers[-1]))

        self.input_vars = layers[0]
        self.output_vars = layers[-1]
        self.lr = lr
        self.q_lr = 1.0
        self.gamma = gamma
        self.epsilon = 200 # randomness
        self.optimizer = optim.Adam(self.model.parameters(), lr=self.lr)
        self.criterion = nn.MSELoss()

    def train_step(self, state, action, reward, next_state, done):
        state = torch.tensor(state, dtype=torch.float)
        next_state = torch.tensor(next_state, dtype=torch.float)
        action = torch.tensor(action, dtype=torch.long)
        reward = torch.tensor(reward, dtype=torch.float)
        # (n, x)

        if len(state.shape) == 1:
            # (1, x)
            state = torch.unsqueeze(state, 0)
            next_state = torch.unsqueeze(next_state, 0)
            action = torch.unsqueeze(action, 0)
            reward = torch.unsqueeze(reward, 0)
            done = (done, )

        # 1: predicted Q values with the current state
        pred = self.model.forward(state)

        target = pred.clone()
        for idx in range(len(done)):
            try:
                best_action = torch.argmax(action).item()
                Q_new = reward[idx] - pred[idx][best_action]
            except IndexError as ex:
                print(ex)
                print('Invalid index of action')
                print(action)
                exit(1)
            if not done[idx]:
                Q_new = reward[idx] + self.gamma * torch.max(self.model.forward(next_state[idx])) - pred[idx][torch.argmax(action).item()]

            target[idx][torch.argmax(action).item()] = pred[idx][torch.argmax(action).item()] + self.q_lr * Q_new

        # 2: Q_new = r + y * max(next_predicted Q value) -> only do this if not done
        # pred.clone()
        # preds[argmax(action)] = Q_new
        self.optimizer.zero_grad()
        loss = self.criterion(target, pred)
        loss.backward()

        self.optimizer.step()
        return float(loss)

    def get_action(self, state):
        # random moves: tradeoff between exploration / exploitation
        # self.epsilon = 200 - self.n_games
        final_move = [0] * self.output_vars
        if random.randint(0, 200) < self.epsilon:
            move = random.randint(0, self.output_vars - 1)
            final_move[move] = 1
        else:
            state0 = torch.tensor(state, dtype=torch.float)
            prediction = self.model.forward(state0)
            move = torch.argmax(prediction).item()
            final_move[move] = 1
        
        return np.array(final_move, dtype=np.float32)

    def save(self, file_name='model.pth'):
        model_folder_path = './model'
        if not os.path.exists(model_folder_path):
            os.makedirs(model_folder_path)
        
        file_name = os.path.join(model_folder_path, file_name)
        torch.save(self.model.state_dict(), file_name)

    def load(self, file_name='model.pth'):
        model_folder_path = './model'
        if not os.path.exists(model_folder_path):
            raise FileNotFoundError()
        
        file_name = os.path.join(model_folder_path, file_name)
        self.model.load_state_dict(torch.load(file_name))
