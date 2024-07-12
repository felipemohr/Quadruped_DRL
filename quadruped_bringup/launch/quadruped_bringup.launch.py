import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    quadruped_control_pkg_share = FindPackageShare("quadruped_control").find("quadruped_control")
    quadruped_agent_pkg_share = FindPackageShare("quadruped_agent").find("quadruped_agent")

    quadruped_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(quadruped_control_pkg_share, "launch", "quadruped_control.launch.py")
        )
    )

    velocity_estimator = Node(
        package="quadruped_estimator",
        executable="velocity_estimator",
        name="velocity_estimator",
        output="both",
    )

    quadruped_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(quadruped_agent_pkg_share, "launch", "quadruped_agent.launch.py")
        )
    )

    return LaunchDescription(
        [quadruped_control, TimerAction(period=8.0, actions=[velocity_estimator, quadruped_agent])]
    )
