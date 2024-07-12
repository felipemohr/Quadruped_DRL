from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    observation_collector = Node(
        package="quadruped_agent",
        executable="observation_collector",
        name="observation_collector",
        output="both",
    )

    agent_controller = Node(
        package="quadruped_agent", executable="agent_controller", name="agent_controller", output="both"
    )

    return LaunchDescription([observation_collector, agent_controller])
