/**
 * @file ObservationCollector.cpp
 * @author Felipe Mohr (felipe18mohr@gmail.com)
 * @brief Node to collect all the observations for the Agent
 * @version 1.0
 * @date 2024-07-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "rclcpp/rclcpp.hpp"

#include "quadruped_agent/ObservationCollector.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

ObservationCollector::ObservationCollector() : Node("observation_collector")
{
  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&ObservationCollector::IMUCallback, this, _1));

  cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&ObservationCollector::cmdVelCallback, this, _1));

  // TODO: Create parameter
  if (use_odom)
  {
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "ground_thruth", 10, std::bind(&ObservationCollector::odometryCallback, this, _1));
  }
  else
  {
    base_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "base_vel", 10, std::bind(&ObservationCollector::baseVelCallback, this, _1));
  }

  joint_states_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&ObservationCollector::jointStatesCallbadk, this, _1));

  // TODO: Create parameter
  if (use_foot_contacts)
  {
    feet_fl_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "fl_foot/contact", 10, std::bind(&ObservationCollector::feetFLContactCallback, this, _1));
    feet_fr_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "fr_foot/contact", 10, std::bind(&ObservationCollector::feetFRContactCallback, this, _1));
    feet_rl_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "rl_foot/contact", 10, std::bind(&ObservationCollector::feetRLContactCallback, this, _1));
    feet_rr_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "rr_foot/contact", 10, std::bind(&ObservationCollector::feetRRContactCallback, this, _1));
  }

  action_subscriber_ = this->create_subscription<quadruped_interfaces::msg::JointsAction>(
      "agent_action", 10, std::bind(&ObservationCollector::lastActionCallback, this, _1));

  obs_publisher_ =
      this->create_publisher<quadruped_interfaces::msg::FullObservation>("observation_state", 10);

  observation_timer_ =
      this->create_wall_timer(30ms, std::bind(&ObservationCollector::publishObservation, this));

  joint_order_map_ = {{"FL_hip_joint", 0}, {"FL_thigh_joint", 1},  {"FL_calf_joint", 2},
                      {"FR_hip_joint", 3}, {"FR_thigh_joint", 4},  {"FR_calf_joint", 5},
                      {"RL_hip_joint", 6}, {"RL_thigh_joint", 7},  {"RL_calf_joint", 8},
                      {"RR_hip_joint", 9}, {"RR_thigh_joint", 10}, {"RR_calf_joint", 11}};

  RCLCPP_INFO(this->get_logger(), "Observation Collector started");
}

ObservationCollector::~ObservationCollector() {}

void ObservationCollector::IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tf2::Quaternion quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z,
                             msg->orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

  full_obs_msg_.base_obs.projected_gravity.x = roll;
  full_obs_msg_.base_obs.projected_gravity.y = pitch;
  full_obs_msg_.base_obs.projected_gravity.z = yaw;
}

void ObservationCollector::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  full_obs_msg_.cmd_vel.linear_velocity_x = msg->linear.x;
  full_obs_msg_.cmd_vel.linear_velocity_y = msg->linear.y;
  full_obs_msg_.cmd_vel.angular_velocity_z = msg->angular.z;
}

void ObservationCollector::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  full_obs_msg_.base_obs.linear_velocity = msg->twist.twist.linear;
  full_obs_msg_.base_obs.angular_velocity = msg->twist.twist.angular;
}

void ObservationCollector::baseVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  full_obs_msg_.base_obs.linear_velocity = msg->linear;
  full_obs_msg_.base_obs.angular_velocity = msg->angular;
}

void ObservationCollector::jointStatesCallbadk(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t msg_idx = 0; msg_idx < msg->name.size(); ++msg_idx)
  {
    std::string joint_name = msg->name.at(msg_idx);
    if (joint_order_map_.find(joint_name) != joint_order_map_.end())
    {
      size_t index = joint_order_map_[joint_name];
      full_obs_msg_.joints_obs.position.at(index) = msg->position.at(msg_idx);
      full_obs_msg_.joints_obs.velocity.at(index) = msg->velocity.at(msg_idx);
    }
  }
}

void ObservationCollector::feetFLContactCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  full_obs_msg_.feet_obs.contact.at(0) = msg->data;
}

void ObservationCollector::feetFRContactCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  full_obs_msg_.feet_obs.contact.at(1) = msg->data;
}

void ObservationCollector::feetRLContactCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  full_obs_msg_.feet_obs.contact.at(2) = msg->data;
}

void ObservationCollector::feetRRContactCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  full_obs_msg_.feet_obs.contact.at(3) = msg->data;
}

void ObservationCollector::lastActionCallback(
    const quadruped_interfaces::msg::JointsAction::SharedPtr msg)
{
  full_obs_msg_.last_action = *msg;
}

void ObservationCollector::publishObservation()
{
  auto cmd_vel = full_obs_msg_.cmd_vel;
  auto base_obs = full_obs_msg_.base_obs;
  auto joints_obs = full_obs_msg_.joints_obs;
  auto feet_obs = full_obs_msg_.feet_obs;
  auto last_action = full_obs_msg_.last_action;

  std::vector<double> observation_vector;
  observation_vector.push_back(cmd_vel.linear_velocity_x);
  observation_vector.push_back(cmd_vel.linear_velocity_y);
  observation_vector.push_back(cmd_vel.angular_velocity_z);

  observation_vector.push_back(base_obs.linear_velocity.x);
  observation_vector.push_back(base_obs.linear_velocity.y);
  observation_vector.push_back(base_obs.linear_velocity.z);

  observation_vector.push_back(base_obs.angular_velocity.x);
  observation_vector.push_back(base_obs.angular_velocity.y);
  observation_vector.push_back(base_obs.angular_velocity.z);

  observation_vector.push_back(base_obs.projected_gravity.x);
  observation_vector.push_back(base_obs.projected_gravity.y);
  observation_vector.push_back(base_obs.projected_gravity.z);

  observation_vector.insert(observation_vector.end(), joints_obs.position.begin(),
                            joints_obs.position.end());
  observation_vector.insert(observation_vector.end(), joints_obs.velocity.begin(),
                            joints_obs.velocity.end());

  if (use_foot_contacts)
  {
    observation_vector.insert(observation_vector.end(), feet_obs.contact.begin(),
                              feet_obs.contact.end());
  }

  observation_vector.insert(observation_vector.end(), last_action.position.begin(),
                            last_action.position.end());

  full_obs_msg_.full_observation = observation_vector;

  obs_publisher_->publish(full_obs_msg_);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObservationCollector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
