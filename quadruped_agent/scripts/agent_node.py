#!/usr/bin/env python3

"""
License: Apache-2.0
Copyright (c) 2024, Felipe Mohr
"""

import rclpy
from rclpy.node import Node

from quadruped_interfaces.msg import FullObservation, JointsAction
from quadruped_agent.agent_model import load_model


class Agent(Node):
    def __init__(self):
        super().__init__("agent_node")

        self.action_publisher = self.create_publisher(JointsAction, "agent_action", 10)
        self.observation_subscriber = self.create_subscription(
            FullObservation, "observation_state", self.observationCallback, 10
        )

        self.get_logger().info("Agent Node started")

    def observationCallback(self, msg):
        print(msg.full_observation)
        print()


def main(args=None):
    rclpy.init(args=args)
    agent_node = Agent()
    rclpy.spin(agent_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
