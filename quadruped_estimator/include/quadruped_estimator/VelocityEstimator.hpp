/**
 * @file VelocityEstimator.hpp
 * @author Felipe Mohr (felipe18mohr@gmail.com)
 * @brief Node the estimate the base velocity of the Agent
 * @version 1.0
 * @date 2024-07-11
 *
 * @copyright Copyright (c) 2024
 *
 */

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
  /** @brief Construct a new Velocity Estimator object */
  VelocityEstimator();
  /** @brief Destroy the Velocity Estimator object */
  ~VelocityEstimator();

private:
  /**
   * @brief Callback function for processing incoming IMU data
   *
   * @param msg A shared pointer to the incoming IMU message
   */
  void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /** @brief Publishes the estimated base velocity */
  void publishVel();

  /** @brief Timer for periodically publishing the estimated velocity */
  rclcpp::TimerBase::SharedPtr vel_timer_;
  /** Timestamp of the last received IMU message */
  rclcpp::Time last_time_;

  /** @brief Subscription to IMU data */
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subscriber_;
  /** @brief Publisher for the estimated velocity */
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> vel_publisher_;

  /** @brief Message containing the estimated velocity */
  geometry_msgs::msg::Twist vel_msg_;

  /** @brief State vector for the velocity estimate */
  Eigen::Vector3d vel_state_;
};

#endif // VELOCITY_ESTIMATOR_HPP
