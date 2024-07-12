#ifndef VELOCITY_ESTIMATOR_HPP
#define VELOCITY_ESTIMATOR_HPP

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <Eigen/Dense>

#include <memory>

// TODO: Apply noise to IMU simulation and use Kalman Filter
class VelocityEstimator : public rclcpp::Node
{
public:
  VelocityEstimator();
  ~VelocityEstimator();

private:
  void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void publishVel();

  rclcpp::TimerBase::SharedPtr vel_timer_;
  rclcpp::Time last_time_;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subscriber_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> vel_publisher_;

  geometry_msgs::msg::Twist vel_msg_;

  Eigen::Vector3d vel_state_;
};

#endif // VELOCITY_ESTIMATOR_HPP
