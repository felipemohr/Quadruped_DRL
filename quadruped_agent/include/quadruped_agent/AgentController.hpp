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
   * @brief Callback function to receive agent action data and send data to trajectory controller
   *
   * @param msg A shared pointer to incoming Joints Action message
   */
  void jointActionCallback(const quadruped_interfaces::msg::JointsAction::SharedPtr msg);

  /** @brief Subscription to agent action data */
  std::shared_ptr<rclcpp::Subscription<quadruped_interfaces::msg::JointsAction>> action_subscriber_;
  /** @brief Publisher for the trajectory controller */
  std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> trajectory_publisher_;

  /** @brief Default joint positions to use as offset */
  std::map<std::string, double> default_joint_position_map_;
};

#endif // AGENT_CONTROLLER_HPP
