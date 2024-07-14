"""
License: Apache-2.0
Copyright (c) 2024, Felipe Mohr

Module: agent_model.py

This module contains the definition of a function to load a trained neural network model.
The neural network is designed for an actor in a reinforcement learning setup.
"""

import torch
from torch import nn

from collections import OrderedDict


def load_model(model_path: str) -> nn.Module:
    """Loads a trained model"""

    class ActorNN(nn.Module):
        def __init__(self, num_obs, num_actions, hidden_dims=[128, 128, 128], activation=nn.ELU()):
            """Initializes the ActorNN class

            Args:
                num_obs (_type_): Number of observations (input features)
                num_actions (_type_): Number of actions (output features)
                hidden_dims (list, optional): List of hidden layer dimensions.
                    Defaults to [128, 128, 128]
                activation (nn.Module, optional): Activation function to be used in the hidden layers.
                    Defaults to nn.ELU()
            """
            super().__init__()

            actor_layers = list()
            actor_layers.append(nn.Linear(num_obs, hidden_dims[0]))
            actor_layers.append(activation)
            for layer_idx, layer_dim in enumerate(hidden_dims):
                if layer_idx == len(hidden_dims) - 1:
                    actor_layers.append(nn.Linear(layer_dim, num_actions))
                else:
                    next_layer_dim = hidden_dims[layer_idx]
                    actor_layers.append(nn.Linear(layer_dim, next_layer_dim))
                    actor_layers.append(activation)
            self.actor = nn.Sequential(*actor_layers)

            print(f"Actor MLP: {self.actor}")

        def forward(self, obs):
            """Defines the forward pass of the network

            Args:
                obs (torch.Tensor): Input tensor containing the observations

            Returns:
                torch.Tensor: Output tensor containing the actions
            """
            return self.actor(obs)

    # Load the saved model
    model_loaded = torch.load(model_path)

    # Initialize the model
    model = ActorNN(num_obs=52, num_actions=12, hidden_dims=[128, 128, 128], activation=nn.ELU())

    # Extract and load the actors's state dictionary
    actor_state_dict = OrderedDict(
        (key, value) for key, value in model_loaded["model_state_dict"].items() if "actor" in key
    )
    model.load_state_dict(actor_state_dict)
    model.eval()

    return model
