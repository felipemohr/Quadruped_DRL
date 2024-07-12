#ifndef OBSERVATION_COLLECTOR_HPP
#define OBSERVATION_COLLECTOR_HPP

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "quadruped_interfaces/msg/full_observation.hpp"

#include <memory>

class ObservationCollector : public rclcpp::Node
{
public:
  ObservationCollector();
  ~ObservationCollector();

private:
  void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void baseVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void jointStatesCallbadk(const sensor_msgs::msg::JointState::SharedPtr msg);
  void feetFLContactCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void feetFRContactCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void feetRLContactCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void feetRRContactCallback(const std_msgs::msg::Bool::SharedPtr msg);

  void publishObservation();

  rclcpp::TimerBase::SharedPtr observation_timer_;

  quadruped_interfaces::msg::FullObservation full_obs_msg_;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subscriber_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_subscriber_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> base_vel_subscriber_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> joint_states_subscriber_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> feet_fl_subscriber;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> feet_fr_subscriber;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> feet_rl_subscriber;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> feet_rr_subscriber;

  std::shared_ptr<rclcpp::Publisher<quadruped_interfaces::msg::FullObservation>> obs_publisher_;
};

#endif // OBSERVATION_COLLECTOR_HPP
