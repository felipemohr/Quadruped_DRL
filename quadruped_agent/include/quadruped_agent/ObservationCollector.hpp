/**
 * @file ObservationCollector.hpp
 * @author Felipe Mohr (felipe18mohr@gmail.com)
 * @brief Node to collect all the observations for the Agent
 * @version 1.0
 * @date 2024-07-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef OBSERVATION_COLLECTOR_HPP
#define OBSERVATION_COLLECTOR_HPP

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "quadruped_interfaces/msg/full_observation.hpp"
#include "quadruped_interfaces/msg/joints_action.hpp"

#include <map>
#include <memory>

class ObservationCollector : public rclcpp::Node
{
public:
  /** @brief Construct a new Observation Collector object */
  ObservationCollector();
  /** @brief Destroy the Observation Collector object */
  ~ObservationCollector();

private:
  /**
   * @brief Callback function to collect incoming IMU data
   *
   * @param msg A shared pointer to the incoming IMU message
   */
  void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  /**
   * @brief Callback function to collect incoming velocity command data
   *
   * @param msg A shared pointer to the incoming Twist message
   */
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  /**
   * @brief Callback function to collect incoming estimated base velocity data
   *
   * @param msg A shared pointer to the incoming Twist message
   */
  void baseVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  /**
   * @brief Callback function to collect incoming joint states data
   *
   * @param msg A shared pointer to the incoming Joint State message
   */
  void jointStatesCallbadk(const sensor_msgs::msg::JointState::SharedPtr msg);
  /**
   * @brief Callback function to collect incoming front left foot contact data
   *
   * @param msg A shared pointer to the incoming Bool message
   */
  void feetFLContactCallback(const std_msgs::msg::Bool::SharedPtr msg);
  /**
   * @brief Callback function to collect incoming front right foot contact data
   *
   * @param msg A shared pointer to the incoming Bool message
   */
  void feetFRContactCallback(const std_msgs::msg::Bool::SharedPtr msg);
  /**
   * @brief Callback function to collect incoming rear left foot contact data
   *
   * @param msg A shared pointer to the incoming Bool message
   */
  void feetRLContactCallback(const std_msgs::msg::Bool::SharedPtr msg);
  /**
   * @brief Callback function to collect incoming rear right foot contact data
   *
   * @param msg A shared pointer to the incoming Bool message
   */
  void feetRRContactCallback(const std_msgs::msg::Bool::SharedPtr msg);
  /**
   * @brief Callback function to collect the last action data
   *
   * @param msg A shared pointer to the Joints Action message
   */
  void lastActionCallback(const quadruped_interfaces::msg::JointsAction::SharedPtr msg);

  /** @brief Publishes the collected observation */
  void publishObservation();

  /** @brief Timer for periodically publishing the observation */
  rclcpp::TimerBase::SharedPtr observation_timer_;

  /** @brief Message containing the full observation */
  quadruped_interfaces::msg::FullObservation full_obs_msg_;

  /** @brief Subscription to IMU data */
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subscriber_;
  /** @brief Subscription to velocity command data */
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_subscriber_;
  /** @brief Subscription to base estimated velocity data */
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> base_vel_subscriber_;
  /** @brief Subscription to joint states data */
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> joint_states_subscriber_;
  /** @brief Subscription to front left contact data */
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> feet_fl_subscriber_;
  /** @brief Subscription to front right contact data */
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> feet_fr_subscriber_;
  /** @brief Subscription to rear left contact data */
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> feet_rl_subscriber_;
  /** @brief Subscription to rear right contact data */
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> feet_rr_subscriber_;
  /** @brief Subscription to last action data */
  std::shared_ptr<rclcpp::Subscription<quadruped_interfaces::msg::JointsAction>> action_subscriber_;

  /** @brief Publisher for the full observation */
  std::shared_ptr<rclcpp::Publisher<quadruped_interfaces::msg::FullObservation>> obs_publisher_;

  /** @brief Mapping from joint_name to joint_index to define joint observation order */
  std::map<std::string, int> joint_order_map_;
};

#endif // OBSERVATION_COLLECTOR_HPP
