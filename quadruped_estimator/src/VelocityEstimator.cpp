/**
 * @file VelocityEstimator.cpp
 * @author Felipe Mohr (felipe18mohr@gmail.com)
 * @brief Node the estimate the base velocity of the Agent
 * @version 1.0
 * @date 2024-07-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "quadruped_estimator/VelocityEstimator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

#include <Eigen/Dense>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

VelocityEstimator::VelocityEstimator() : Node("velocity_estimator")
{
  RCLCPP_INFO(this->get_logger(), "Initializing Velocity Estimator");

  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&VelocityEstimator::IMUCallback, this, _1));

  vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("base_vel", 10);

  vel_timer_ = this->create_wall_timer(20ms, std::bind(&VelocityEstimator::publishVel, this));

  vel_state_ = Eigen::Vector3d::Zero();
  last_time_ = this->now();
}

VelocityEstimator::~VelocityEstimator() {}

void VelocityEstimator::IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // TODO: Publish base_observation (?)
  Eigen::Vector3d accel;
  tf2::fromMsg(msg->linear_acceleration, accel);

  Eigen::Quaterniond quaternion;
  tf2::fromMsg(msg->orientation, quaternion);

  Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
  Eigen::Vector3d gravity_vector(0.0, 0.0, 9.80);
  Eigen::Vector3d accel_transformed = rotation_matrix * accel - gravity_vector;

  double dt = (this->now() - last_time_).seconds();

  vel_state_ += dt * accel_transformed;
  last_time_ = this->now();

  tf2::toMsg(vel_state_, vel_msg_.linear);
  vel_msg_.angular = msg->angular_velocity;
}

void VelocityEstimator::publishVel() { vel_publisher_->publish(vel_msg_); }

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VelocityEstimator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
