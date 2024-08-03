"""
License: Apache-2.0
Copyright (c) 2024, Felipe Mohr
"""

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import argparse
import signal
import sys
import os

isaacsim_proc = None


def signal_handler(sig, frame):
    print("Ctrl+C received, shutting down...")
    isaac_sim_shutdown()
    sys.exit()


def isaac_sim_shutdown():
    print(isaacsim_proc)
    if isaacsim_proc is not None:
        if sys.platform == "win32":
            os.kill(isaacsim_proc.pid, signal.SIGTERM)
        else:
            os.killpg(os.getpgid(isaacsim_proc.pid), signal.SIGKILL)
    print("Isaac Sim subprocesses terminated.")


# Register the signal handler for SIGINT
signal.signal(signal.SIGINT, signal_handler)


class IsaacSimLauncherNode(Node):
    def __init__(self):
        super().__init__("isaac_sim_launcher_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("install_path", ""),
                ("version", "4.1.0"),
                ("standalone", ""),
            ],
        )
        self.execute_launch()

    def execute_launch(self):
        global isaacsim_proc
        args = argparse.Namespace()
        args.install_path = self.get_parameter("install_path").get_parameter_value().string_value
        args.version = self.get_parameter("version").get_parameter_value().string_value
        args.standalone = self.get_parameter("standalone").get_parameter_value().string_value

        filepath_root = ""

        if args.install_path != "":
            filepath_root = args.install_path
        else:
            # If custom Isaac Sim Installation folder not given, use the default path using version number provided.
            if sys.platform == "win32":
                home_path = os.getenv("USERPROFILE")
                filepath_root = os.path.join(home_path, "AppData", "Local", "ov", "pkg", f"isaac-sim-{args.version}")
            else:
                home_path = os.getenv("HOME")
                filepath_root = os.path.join(home_path, ".local", "share", "ov", "pkg", f"isaac-sim-{args.version}")

        if not os.path.exists(filepath_root):
            print(f"{filepath_root} filepath does not exist.")
            sys.exit(0)

        if args.standalone != "":
            executable_path = os.path.join(filepath_root, "python.sh" if sys.platform != "win32" else "python.bat")
            isaacsim_proc = subprocess.Popen(f"{executable_path} {args.standalone}", shell=True, start_new_session=True)
        else:
            print("Standalone argument not defined. Launching Isaac Sim.")
            isaacpath = f"{os.path.join(filepath_root, 'isaac-sim.sh' if sys.platform != 'win32' else 'isaac-sim.bat')}"
            executable_command = f"{isaacpath} --/isaac/startup/ros_bridge_extension=omni.isaac.ros2_bridge"
            isaacsim_proc = subprocess.Popen(executable_command, shell=True, start_new_session=True)
            isaacsim_proc


def main(args=None):
    rclpy.init()
    isaac_sim_launcher_node = IsaacSimLauncherNode()
    rclpy.spin(isaac_sim_launcher_node)

    # Ensure all subprocesses are terminated before exiting
    isaac_sim_shutdown()
    isaac_sim_launcher_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
