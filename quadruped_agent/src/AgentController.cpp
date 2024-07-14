/**
 * @file AgentController.cpp
 * @author Felipe Mohr (felipe18mohr@gmail.com)
 * @brief Node to control the Agent with trajectory controller
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

  trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "trajectory_controller/joint_trajectory", 10);

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
  auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
  auto trajectory_point = trajectory_msgs::msg::JointTrajectoryPoint();
  trajectory_point.time_from_start.sec = 0;
  trajectory_point.time_from_start.nanosec = 0;

  size_t idx = 0;
  for (auto &[joint, position] : default_joint_position_map_)
  {
    double joint_position = 0.5 * msg->position.at(idx) + position;
    trajectory_point.positions.push_back(joint_position);
    trajectory_msg.joint_names.push_back(joint);

    idx++;
  }

  trajectory_msg.points.push_back(trajectory_point);

  trajectory_publisher_->publish(trajectory_msg);
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
