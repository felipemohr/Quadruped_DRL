#!/usr/bin/env python3

"""
License: Apache-2.0
Copyright (c) 2024, Felipe Mohr
"""

import rclpy
from rclpy.node import Node

from quadruped_interfaces.msg import FullObservation, JointsAction
from quadruped_agent.agent_model import load_model

import torch
import numpy as np


class Agent(Node):
    def __init__(self):
        super().__init__("agent_node")

        self.action_publisher = self.create_publisher(JointsAction, "agent_action", 10)
        self.observation_subscriber = self.create_subscription(
            FullObservation, "observation_state", self.observationCallback, 10
        )

        # TODO: Read parameter
        self.model_path = ""

        self.model = load_model(model_path=self.model_path, num_obs=48, num_actions=12)

        self.get_logger().info("Agent Node started")

    def observationCallback(self, msg):
        with torch.no_grad():
            self.obs_tensor = torch.Tensor(msg.full_observation)
            self.action_tensor = self.model(self.obs_tensor)

            action_msg = JointsAction()
            action_msg.position = self.action_tensor.numpy().astype(np.float64)

            self.action_publisher.publish(action_msg)

            print("Observation: ", end="")
            print(self.obs_tensor)
            print("Action: ", end="")
            print(self.action_tensor)
            print()


def main(args=None):
    rclpy.init(args=args)
    agent_node = Agent()
    rclpy.spin(agent_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
