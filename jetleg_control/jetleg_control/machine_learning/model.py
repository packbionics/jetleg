import torch
import torch.nn as nn
import torch.nn.functional as F
import os

class Linear_QNet(nn.Module):
    def __init__(self, layers_description):
        super().__init__()

        # Initialize an empty list of layers for the neural network
        self.nn_layers = nn.ModuleList()

        # Add input layer and hidden layers to the neural network
        for i in range(len(layers_description) - 1):
            linear = nn.Linear(layers_description[i], layers_description[i + 1])
            self.nn_layers.append(linear)

    def forward(self, x):

        # Apply the leaky ReLU activation function on the hidden layers
        for i in range(0, len(self.nn_layers) - 1):
            x = F.leaky_relu(self.nn_layers[i](x), 0.01)

        # Run the output layer without an acivation function
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
