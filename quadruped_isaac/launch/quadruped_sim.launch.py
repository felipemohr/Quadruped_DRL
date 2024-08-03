import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    quadruped_isaac_pkg_share = FindPackageShare("quadruped_isaac").find("quadruped_isaac")

    standalone_default = os.path.join(quadruped_isaac_pkg_share, "scripts", "standalone", "quadruped_simulation.py")
    install_path_default = ""
    version_default = "4.1.0"

    isaacsim_node = Node(
        package="quadruped_isaac",
        executable="run_isaacsim.py",
        name="isaacsim",
        output="screen",
        parameters=[
            {
                "install_path": LaunchConfiguration("install_path"),
                "version": LaunchConfiguration("version"),
                "standalone": LaunchConfiguration("standalone"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "install_path",
                default_value=install_path_default,
                description="If Isaac Sim is insalled in a non-default location, provide a specific path to Isaac Sim installation root folder. (If defined, 'version' parameter will be ignored)",
            ),
            DeclareLaunchArgument(
                "version",
                default_value=version_default,
                description="Specify the version of Isaac Sim to use. Isaac Sim will be run from default install root folder for the specified version.",
            ),
            DeclareLaunchArgument(
                "standalone",
                default_value=standalone_default,
                description="Provide the path to the python file to open it and start Isaac Sim in standalone workflow. If left empty, Isaac Sim will open an empty stage in standard Gui mode.",
            ),
            isaacsim_node,
        ]
    )
