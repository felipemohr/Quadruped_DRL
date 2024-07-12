/**
 * @file standing_pose_server.cpp
 * @author Felipe Mohr (felipe18mohr@gmail.com)
 * @brief Implements a Service for triggering a standing pose for the robot
 * @version 1.0
 * @date 2024-06-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create a ROS2 Node for the standing pose server
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("standing_pose_server");

  // Default standing joint positions for all legs
  std::map<std::string, double> default_standing_joint_positions{
      {"FL_hip_joint", 0.0}, {"FL_thigh_joint", 0.78}, {"FL_calf_joint", -1.57},
      {"FR_hip_joint", 0.0}, {"FR_thigh_joint", 0.78}, {"FR_calf_joint", -1.57},
      {"RL_hip_joint", 0.0}, {"RL_thigh_joint", 0.78}, {"RL_calf_joint", -1.57},
      {"RR_hip_joint", 0.0}, {"RR_thigh_joint", 0.78}, {"RR_calf_joint", -1.57}};

  // Time in seconds to reach the standing pose
  int32_t seconds_to_stand;
  std::map<std::string, double> standing_joint_positions;

  // Declare and retrieve parameters
  node->declare_parameter("seconds_to_stand", 3);
  node->get_parameter("seconds_to_stand", seconds_to_stand);
  node->declare_parameters("standing_joint_positions", default_standing_joint_positions);
  node->get_parameters("standing_joint_positions", standing_joint_positions);

  // Prepare the JointTrajectory message
  std::vector<std::string> joint_names;
  std::vector<double> joint_positions;

  for (auto &[name, position] : standing_joint_positions)
  {
    joint_names.push_back(name);
    joint_positions.push_back(position);
  }

  auto joint_point = trajectory_msgs::msg::JointTrajectoryPoint();
  joint_point.positions = joint_positions;
  joint_point.time_from_start.sec = seconds_to_stand;

  auto standing_pose_msg = trajectory_msgs::msg::JointTrajectory();
  standing_pose_msg.joint_names = joint_names;
  standing_pose_msg.points.push_back(joint_point);

  // Publisher for the joint trajectory
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher =
      node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          "trajectory_controller/joint_trajectory", 10);

  // Service to trigger the standing pose
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service =
      node->create_service<std_srvs::srv::Trigger>(
          "standing_pose",
          [node, standing_pose_msg, joint_trajectory_publisher](
              const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
          {
            (void)request;

            joint_trajectory_publisher->publish(standing_pose_msg);

            response->success = true;
            response->message = "Quadruped reseting to standing pose";

            RCLCPP_INFO(node->get_logger(), response->message.c_str());
          });

  RCLCPP_INFO(node->get_logger(), "Standing Pose Server ready!");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
