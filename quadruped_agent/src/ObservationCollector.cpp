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

  base_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "base_vel", 10, std::bind(&ObservationCollector::baseVelCallback, this, _1));

  joint_states_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&ObservationCollector::jointStatesCallbadk, this, _1));

  feet_fl_subscriber = this->create_subscription<std_msgs::msg::Bool>(
      "fl_foot/contact", 10, std::bind(&ObservationCollector::feetFLContactCallback, this, _1));
  feet_fr_subscriber = this->create_subscription<std_msgs::msg::Bool>(
      "fr_foot/contact", 10, std::bind(&ObservationCollector::feetFRContactCallback, this, _1));
  feet_rl_subscriber = this->create_subscription<std_msgs::msg::Bool>(
      "rl_foot/contact", 10, std::bind(&ObservationCollector::feetRLContactCallback, this, _1));
  feet_rr_subscriber = this->create_subscription<std_msgs::msg::Bool>(
      "rr_foot/contact", 10, std::bind(&ObservationCollector::feetRRContactCallback, this, _1));

  obs_publisher_ =
      this->create_publisher<quadruped_interfaces::msg::FullObservation>("observation_state", 10);

  observation_timer_ =
      this->create_wall_timer(30ms, std::bind(&ObservationCollector::publishObservation, this));

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

void ObservationCollector::baseVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  full_obs_msg_.base_obs.linear_velocity = msg->linear;
  full_obs_msg_.base_obs.angular_velocity = msg->angular;
}

void ObservationCollector::jointStatesCallbadk(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // TODO
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

void ObservationCollector::publishObservation() { obs_publisher_->publish(full_obs_msg_); }

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObservationCollector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
