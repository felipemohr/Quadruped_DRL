#include "quadruped_estimator/VelocityEstimator.hpp"
#include "rclcpp/rclcpp.hpp"

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
  Eigen::Vector3d accel(msg->linear_acceleration.x, msg->linear_acceleration.y,
                        msg->linear_acceleration.z);

  Eigen::Quaterniond quaternion(msg->orientation.w, msg->orientation.x, msg->orientation.y,
                                msg->orientation.z);
  Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();

  Eigen::Vector3d gravity_vector(0.0, 0.0, 9.80);
  Eigen::Vector3d accel_transformed = rotation_matrix * accel - gravity_vector;

  double dt = (this->now() - last_time_).seconds();

  vel_state_ += dt * accel_transformed;
  last_time_ = this->now();

  vel_msg_.linear.x = vel_state_.x();
  vel_msg_.linear.y = vel_state_.y();
  vel_msg_.linear.z = vel_state_.z();
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
