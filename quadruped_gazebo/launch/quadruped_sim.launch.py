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
    quadruped_gazebo_pkg_share = FindPackageShare("quadruped_gazebo").find("quadruped_gazebo")

    robot_description_file = os.path.join(go2_description_pkg_share, "xacro", "robot.xacro")

    robot_description_config = xacro.process_file(robot_description_file)
    robot_description_dict = {"robot_description": robot_description_config.toxml()}

    gz_bridge_config = os.path.join(quadruped_gazebo_pkg_share, "config", "gz_bridge.yaml")
    gz_bridge_dict = {"config_file": gz_bridge_config}

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_pkg_share, "launch", "gz_sim.launch.py")),
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
        parameters=[gz_bridge_dict],
        output="screen",
    )

    # IMU tf2 Transform
    tf2_imu_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "imu_link", "go2/base/imu_sensor"],
    )

    # Lidar tf2 Transform
    tf2_lidar_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "lidar_link", "go2/base/gpu_lidar"],
    )

    # Camera tf2 Transform
    tf2_camera_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "camera_link", "go2/base/rgbd_camera"],
    )

    return LaunchDescription(
        [
            gz_sim,
            spawn,
            robot_state_publisher,
            gz_bridge,
            tf2_imu_transform,
            tf2_lidar_transform,
            tf2_camera_transform,
        ]
    )
