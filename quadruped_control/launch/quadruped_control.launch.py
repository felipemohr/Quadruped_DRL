import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    quadruped_control_pkg_share = FindPackageShare("quadruped_control").find("quadruped_control")

    standing_pose_config = os.path.join(quadruped_control_pkg_share, "config", "go2_standing_pose.yaml")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    standing_pose_server = Node(
        package="quadruped_control",
        executable="standing_pose_server",
        parameters=[standing_pose_config],
    )

    standing_command = ExecuteProcess(
        cmd=["ros2", "service", "call", "standing_pose", "std_srvs/srv/Trigger", "{}"],
        output="screen",
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            trajectory_controller_spawner,
            TimerAction(period=1.0, actions=[standing_pose_server]),
            TimerAction(period=2.0, actions=[standing_command]),
        ]
    )
