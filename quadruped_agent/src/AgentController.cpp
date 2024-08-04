/**
 * @file AgentController.cpp
 * @author Felipe Mohr (felipe18mohr@gmail.com)
 * @brief Node to control the Agent with trajectory controller or joint commands
 * @version 1.0
 * @date 2024-07-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "rclcpp/rclcpp.hpp"

#include "quadruped_agent/AgentController.hpp"

#include <memory>

using std::placeholders::_1;

AgentController::AgentController() : Node("agent_controller")
{
  action_subscriber_ = this->create_subscription<quadruped_interfaces::msg::JointsAction>(
      "agent_action", 10, std::bind(&AgentController::jointActionCallback, this, _1));

  // TODO: Create parameter
  if (use_joint_trajectory)
  {
    joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "trajectory_controller/joint_trajectory", 10);
  }
  else
  {
    joint_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);
  }

  // TODO: Create configuration file or read from default_standing_pose
  default_joint_position_map_ = {
      {"FL_hip_joint", 0.1},  {"FL_thigh_joint", 0.8}, {"FL_calf_joint", -1.5},
      {"FR_hip_joint", -0.1}, {"FR_thigh_joint", 0.8}, {"FR_calf_joint", -1.5},
      {"RL_hip_joint", 0.1},  {"RL_thigh_joint", 1.0}, {"RL_calf_joint", -1.5},
      {"RR_hip_joint", -0.1}, {"RR_thigh_joint", 1.0}, {"RR_calf_joint", -1.5}};

  RCLCPP_INFO(this->get_logger(), "Agent Controller started");
}

void AgentController::jointActionCallback(
    const quadruped_interfaces::msg::JointsAction::SharedPtr msg)
{
  use_joint_trajectory ? publishJointTrajectory(msg) : publishJointState(msg);
}

void AgentController::publishJointState(
    const quadruped_interfaces::msg::JointsAction::SharedPtr msg)
{
  auto joint_state_msg = sensor_msgs::msg::JointState();
  joint_state_msg.header.stamp = this->get_clock()->now();

  size_t idx = 0;
  for (auto &[joint, position] : default_joint_position_map_)
  {
    double joint_position = 0.5 * msg->position.at(idx) + position;
    joint_state_msg.name.push_back(joint);
    joint_state_msg.position.push_back(joint_position);

    idx++;
  }

  joint_state_publisher_->publish(joint_state_msg);
}

void AgentController::publishJointTrajectory(
    const quadruped_interfaces::msg::JointsAction::SharedPtr msg)
{
  auto joint_trajectory_msg = trajectory_msgs::msg::JointTrajectory();
  auto joint_trajectory_point = trajectory_msgs::msg::JointTrajectoryPoint();
  joint_trajectory_point.time_from_start.sec = 0;
  joint_trajectory_point.time_from_start.nanosec = 0;

  size_t idx = 0;
  for (auto &[joint, position] : default_joint_position_map_)
  {
    double joint_position = 0.5 * msg->position.at(idx) + position;
    joint_trajectory_point.positions.push_back(joint_position);
    joint_trajectory_msg.joint_names.push_back(joint);

    idx++;
  }

  joint_trajectory_msg.header.stamp = this->get_clock()->now();
  joint_trajectory_msg.points.push_back(joint_trajectory_point);

  joint_trajectory_publisher_->publish(joint_trajectory_msg);
}

AgentController::~AgentController() {}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AgentController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
