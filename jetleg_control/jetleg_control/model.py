import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import os

class Linear_QNet(nn.Module):
    def __init__(self, layers_description):
        super().__init__()

        self.nn_layers = nn.ModuleList()
        for i in range(len(layers_description) - 1):
            linear = nn.Linear(layers_description[i], layers_description[i + 1])
            self.nn_layers.append(linear)

    def forward(self, x):
        for i in range(0, len(self.nn_layers) - 1):
            x = F.leaky_relu(self.nn_layers[i](x), 0.01)

        x = self.nn_layers[-1](x)
        return x

    def save(self, file_name='model.pth'):
        model_folder_path = './model'
        if not os.path.exists(model_folder_path):
            os.makedirs(model_folder_path)
        
        file_name = os.path.join(model_folder_path, file_name)
        torch.save(self.state_dict(), file_name)

    def load(self, file_name='model.pth'):
        model_folder_path = './model'
        if not os.path.exists(model_folder_path):
            raise FileNotFoundError()
        
        file_name = os.path.join(model_folder_path, file_name)
        self.load_state_dict(torch.load(file_name))

class QTrainer:
    def __init__(self, model, lr, gamma):
        self.lr = lr
        self.q_lr = 1.0
        self.gamma = gamma
        self.model = model
        self.optimizer = optim.Adam(model.parameters(), lr=self.lr)
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
        pred = self.model(state)

        target = pred.clone()
        for idx in range(len(done)):
            try:
                Q_new = reward[idx] - pred[idx][torch.argmax(action).item()]
            except IndexError:
                print('Action is empty during training step')
                print(action)
                exit(1)
            if not done[idx]:
                Q_new = reward[idx] + self.gamma * torch.max(self.model(next_state[idx])) - pred[idx][torch.argmax(action).item()]

            target[idx][torch.argmax(action).item()] = pred[idx][torch.argmax(action).item()] + self.q_lr * Q_new

        # 2: Q_new = r + y * max(next_predicted Q value) -> only do this if not done
        # pred.clone()
        # preds[argmax(action)] = Q_new
        self.optimizer.zero_grad()
        loss = self.criterion(target, pred)
        loss.backward()

        self.optimizer.step()
        return float(loss)