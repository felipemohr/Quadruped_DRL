/**
 * @file AgentController.hpp
 * @author Felipe Mohr (felipe18mohr@gmail.com)
 * @brief Node to control the Agent with trajectory controller
 * @version 1.0
 * @date 2024-07-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef AGENT_CONTROLLER_HPP
#define AGENT_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"

#include "quadruped_interfaces/msg/joints_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <map>
#include <memory>

class AgentController : public rclcpp::Node
{
public:
  /** @brief Construct a new Agent Controller object */
  AgentController();
  /** @brief Destroy the Agent Controller object */
  ~AgentController();

private:
  /**
   * @brief Callback function to receive agent action data and send data to the publisher
   *
   * @param msg A shared pointer to incoming Joints Action message
   */
  void jointActionCallback(const quadruped_interfaces::msg::JointsAction::SharedPtr msg);

  /**
   * @brief Function to publish JointState message with the received agent action data
   *
   * @param msg A shared pointer to incoming Joints Action message
   */
  void publishJointState(const quadruped_interfaces::msg::JointsAction::SharedPtr msg);

  /**
   * @brief Function to publish JointTrajectory message with the received agent action data
   *
   * @param msg A shared pointer to incoming Joints Action message
   */
  void publishJointTrajectory(const quadruped_interfaces::msg::JointsAction::SharedPtr msg);

  /** @brief Subscription to agent action data */
  std::shared_ptr<rclcpp::Subscription<quadruped_interfaces::msg::JointsAction>> action_subscriber_;
  /** @brief Publisher for the trajectory controller */
  std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>>
      joint_trajectory_publisher_;
  /** @brief Publisher for the joint commands */
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_state_publisher_;

  /** @brief Default joint positions to use as offset */
  std::map<std::string, double> default_joint_position_map_;

  /** @brief Either to publish JointTrajectory message or JointState message */
  bool use_joint_trajectory = false;
};

#endif // AGENT_CONTROLLER_HPP
