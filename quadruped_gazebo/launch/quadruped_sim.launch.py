import os
import xacro
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    ros_gz_sim_pkg_share = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
    go2_description_pkg_share = FindPackageShare("go2_description").find("go2_description")

    robot_description_file = os.path.join(go2_description_pkg_share, "xacro", "robot.xacro")

    robot_description_config = xacro.process_file(robot_description_file)
    robot_description_dict = {"robot_description": robot_description_config.toxml()}

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # Spawn
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "go2", "-topic", "robot_description", "-z", "1.0"],
        output="screen",
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description_dict],
    )

    # Gazebo ~ ROS Bridge
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Clock (IGN -> ROS2)
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    return LaunchDescription([gz_sim, spawn, robot_state_publisher, gz_bridge])
