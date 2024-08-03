"""
License: Apache-2.0
Copyright (c) 2024, Felipe Mohr
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot

from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import add_reference_to_stage

from omni.isaac.sensor import IMUSensor, ContactSensor

import omni.graph.core as og

enable_extension("omni.isaac.ros2_bridge")
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

simulation_app.update()

from typing import Optional, Sequence

import numpy as np


class QuadrupedRobot(Robot):
    def __init__(
        self,
        usd_path: str,
        prim_path: str = "/World/Quadruped",
        name: str = "quadruped",
        position: Optional[Sequence[float]] = np.array([0.0, 0.0, 0.5], dtype=np.float32),
        orientation: Optional[Sequence[float]] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
    ) -> None:

        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

        self._init_position = position
        self._init_orientation = orientation
        self._quadruped_prim_path = prim_path

        Robot.__init__(self, prim_path=prim_path, name=name, position=position, orientation=orientation)

        self.feet = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]

    def initialize_robot(self) -> None:
        self.set_world_pose(position=self._init_position, orientation=self._init_orientation)
        self.set_linear_velocity(np.zeros(3))
        self.set_angular_velocity(np.zeros(3))
        self.initialize_joints()

    def initialize_joints(self) -> None:
        joint_pos = np.array(
            [0, 0, 0, 0, np.pi / 4, np.pi / 4, np.pi / 4, np.pi / 4, -np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2],
            dtype=np.float32,
        )
        self.set_joint_positions(positions=joint_pos)
        self.set_joint_velocities(velocities=np.zeros(12))
        self.set_joint_efforts(efforts=np.zeros(12))

    def add_imu(self, dt: float) -> None:
        self._imu_path = self._quadruped_prim_path + "/imu/imu_sensor"

        self._imu_sensor = IMUSensor(
            prim_path=self._imu_path,
            name="imu",
            dt=dt,
            translation=np.array([0, 0, 0]),
            orientation=np.array([1, 0, 0, 0]),
        )

        self._sensors.append(self._imu_sensor)

    def add_feet_contact(self, dt: float, radius: float) -> None:
        self._feet_contact = dict()
        self._contact_sensors = dict()

        for foot in self.feet:
            self._feet_contact[foot] = False
            self._contact_sensors[foot] = ContactSensor(
                prim_path=self._quadruped_prim_path + "/" + foot + "/sensor",
                name=foot + "_sensor",
                min_threshold=0,
                max_threshold=1000000,
                radius=radius,
                dt=dt,
            )
            self._sensors.append(self._contact_sensors[foot])

    def get_imu_data(self) -> dict:
        frame = self._imu_sensor.get_current_frame()
        return frame

    def get_feet_contact_data(self) -> dict:
        for foot in self.feet:
            frame = self._contact_sensors[foot].get_current_frame()
            if "in_contact" in frame.keys():
                self._feet_contact[foot] = frame["in_contact"]
            else:
                self._feet_contact[foot] = False
        return self._feet_contact

    def get_linear_velocity(self) -> np.ndarray:
        return super().get_linear_velocity()

    def get_angular_velocity(self) -> np.ndarray:
        return super().get_angular_velocity()

    def get_imu_prim_path(self) -> str:
        return self._imu_path


class QuadrupedSimulationNode(Node):
    def __init__(self) -> None:
        Node.__init__(self, "quadruped_simulation_node")

        assets_root_path = get_assets_root_path()
        go2_asset_path = assets_root_path + "/Isaac/Robots/Unitree/Go2/go2.usd"

        self._quadruped = QuadrupedRobot(
            usd_path=go2_asset_path,
            prim_path="/World/Go2",
            name="go2",
            position=np.array([0.0, 0.0, 0.50]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        self._quadruped.add_imu(dt=1.0 / 400.0)
        # TODO: Fix contact sensors
        # self._quadruped.add_feet_contact(dt=1.0 / 400.0, radius=0.03)

        self._world = World(physics_dt=1.0 / 400.0, rendering_dt=10.0 / 400.0, stage_units_in_meters=1.0)
        self._world.scene.add_default_ground_plane()
        self._world.scene.add(self._quadruped)
        self._world.reset()

        self._quadruped.initialize_robot()

        self.create_ros2_graphs()

        self._base_vel_publisher = self.create_publisher(Twist, "base_vel", 10)
        self._publish_base_velocity_timer = self.create_timer(0.01, self.publish_base_velocity)

    def create_ros2_graphs(self) -> None:
        try:
            self._joints_graph = og.Controller.edit(
                {"graph_path": "/QuadrupedJoints", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                        ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                        ("SubscribeJointCmd", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                        ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "SubscribeJointCmd.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                        ("Context.outputs:context", "PublishJointState.inputs:context"),
                        ("Context.outputs:context", "SubscribeJointCmd.inputs:context"),
                        ("SubscribeJointCmd.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                        ("SubscribeJointCmd.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                        ("SubscribeJointCmd.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                        ("SubscribeJointCmd.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("ArticulationController.inputs:robotPath", self._quadruped.prim_path),
                        ("PublishJointState.inputs:targetPrim", self._quadruped.prim_path),
                    ],
                },
            )
            self._tf_graph = og.Controller.edit(
                {"graph_path": "/QuadrupedTF", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                        ("PublishTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                        ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                        ("Context.outputs:context", "PublishTF.inputs:context"),
                        ("Context.outputs:context", "PublishClock.inputs:context"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTF.inputs:targetPrims", self._quadruped.prim_path),
                    ],
                },
            )
            self._imu_graph = og.Controller.edit(
                {"graph_path": "/QuadrupedIMU", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                        ("SimulationGate", "omni.isaac.core_nodes.IsaacSimulationGate"),
                        ("ReadIMU", "omni.isaac.sensor.IsaacReadIMU"),
                        ("PublishIMU", "omni.isaac.ros2_bridge.ROS2PublishImu"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SimulationGate.inputs:execIn"),
                        ("SimulationGate.outputs:execOut", "ReadIMU.inputs:execIn"),
                        ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
                        ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
                        ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
                        ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
                        ("Context.outputs:context", "PublishIMU.inputs:context"),
                        ("ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("SimulationGate.inputs:step", 4),
                        ("ReadIMU.inputs:imuPrim", self._quadruped.get_imu_prim_path()),
                    ],
                },
            )
        except Exception as e:
            print(e)

    def publish_base_velocity(self) -> None:
        lin_vel = self._quadruped.get_linear_velocity()
        ang_vel = self._quadruped.get_angular_velocity()

        vel_msg = Twist()
        vel_msg.linear.x = lin_vel[0].item()
        vel_msg.linear.y = lin_vel[1].item()
        vel_msg.linear.z = lin_vel[2].item()
        vel_msg.angular.x = ang_vel[0].item()
        vel_msg.angular.y = ang_vel[1].item()
        vel_msg.angular.z = ang_vel[2].item()

        self._base_vel_publisher.publish(vel_msg)

    def run_simulation(self) -> None:
        reset_needed = False
        while simulation_app.is_running():
            self._world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self._world.is_stopped() and not reset_needed:
                reset_needed = True
            if self._world.is_playing():
                if reset_needed:
                    self._world.reset()
                    reset_needed = False
        self.destroy_node()
        simulation_app.close()


if __name__ == "__main__":
    rclpy.init()
    node = QuadrupedSimulationNode()
    node.run_simulation()
    rclpy.shutdown()