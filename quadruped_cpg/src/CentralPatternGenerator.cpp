/**
 * @file CentralPatternGenerator.cpp
 * @author Felipe Mohr (felipe18mohr@gmail.com)
 * @brief Node control a quadruped robot using Central Pattern Generator algorithm
 * @version 1.0
 * @date 2024-08-14
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "rclcpp/rclcpp.hpp"

#include "quadruped_cpg/CentralPatternGenerator.hpp"

#include <cmath>
#include <memory>

using std::placeholders::_1;
using namespace std::chrono_literals;

CentralPatternGenerator::CentralPatternGenerator() : Node("central_pattern_generator")
{
  cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&CentralPatternGenerator::cmdVelCallback, this, _1));

  cmd_ik_publisher_ =
      this->create_publisher<quadruped_interfaces::msg::QuadrupedKinematics>("cmd_ik", 10);

  publish_ik_timer_ =
      this->create_wall_timer(20ms, std::bind(&CentralPatternGenerator::cmdIKCallback, this));

  // TODO: Create ros2 parameters
  coupling_matrix_ << 0, M_PI, M_PI, 0, -M_PI, 0, 0, -M_PI, -M_PI, 0, 0, -M_PI, 0, M_PI, M_PI, 0;

  convergence_factor_a_ = 50.0;
  coupling_weight_ = 1.0;
  amplitude_mu_ = 1.0;
  swing_frequency_ = 2.0;
  stance_frequency_ = 2.0;
  ground_clearance_ = 0.05;
  ground_penetration_ = 0.005;

  amplitude_r_ = Eigen::Vector4d::Random();
  phase_theta_ = Eigen::Vector4d::Random();

  last_time_ = this->get_clock()->now();

  RCLCPP_INFO(this->get_logger(), "Central Pattern Generator started");
}

CentralPatternGenerator::~CentralPatternGenerator() {}

void CentralPatternGenerator::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (abs(msg->linear.x) < 0.1 && abs(msg->linear.y) < 0.1 && abs(msg->angular.z) < M_PI / 8.0)
  {
    d_step_x_ = Eigen::Vector4d::Zero();
    d_step_y_ = Eigen::Vector4d::Zero();
    return;
  }

  // TODO: Read parameter
  double body_length = 0.3868;

  // TODO: Fix calculation to achieve desired velocity
  d_step_x_ = Eigen::Vector4d::Constant(msg->linear.x / 10.0);
  d_step_y_ = Eigen::Vector4d::Constant(msg->linear.y / 10.0);
  d_step_y_(0) += msg->angular.z * body_length / 2 / (2 * M_PI);
  d_step_y_(1) -= msg->angular.z * body_length / 2 / (2 * M_PI);
  d_step_y_(2) += msg->angular.z * body_length / 2 / (2 * M_PI);
  d_step_y_(3) -= msg->angular.z * body_length / 2 / (2 * M_PI);
}

void CentralPatternGenerator::cmdIKCallback()
{
  float dt = this->get_clock()->now().seconds() - last_time_.seconds();

  auto [feet_x, feet_y, feet_z] = this->updateCPG(dt);

  quadruped_interfaces::msg::QuadrupedKinematics cmd_ik_msg_;
  cmd_ik_msg_.use_foot_transforms = true;
  cmd_ik_msg_.front_left_foot.x = feet_x(0);
  cmd_ik_msg_.front_left_foot.y = feet_y(0);
  cmd_ik_msg_.front_left_foot.z = feet_z(0);

  cmd_ik_msg_.front_right_foot.x = feet_x(1);
  cmd_ik_msg_.front_right_foot.y = feet_y(1);
  cmd_ik_msg_.front_right_foot.z = feet_z(1);

  cmd_ik_msg_.rear_left_foot.x = feet_x(2);
  cmd_ik_msg_.rear_left_foot.y = feet_y(2);
  cmd_ik_msg_.rear_left_foot.z = feet_z(2);

  cmd_ik_msg_.rear_right_foot.x = feet_x(3);
  cmd_ik_msg_.rear_right_foot.y = feet_y(3);
  cmd_ik_msg_.rear_right_foot.z = feet_z(3);

  cmd_ik_publisher_->publish(cmd_ik_msg_);
}

FeetPositionTuple CentralPatternGenerator::updateCPG(double dt)
{
  amplitude_d2r_ = convergence_factor_a_ *
                   ((convergence_factor_a_ / 4.0) *
                        (Eigen::Vector4d::Constant(amplitude_mu_).array() - amplitude_r_.array()) -
                    amplitude_dr_.array());
  amplitude_dr_ += amplitude_d2r_ * dt;

  for (int i = 0; i < 4; i++)
  {
    frequency_omega_(i) =
        (2 * M_PI) * (phase_theta_(i) < M_PI ? swing_frequency_ : stance_frequency_);
    phase_dtheta_(i) = frequency_omega_(i);
    for (int j = 0; j < 4; j++)
      phase_dtheta_(i) += amplitude_dr_(j) * coupling_weight_ *
                          sin(phase_theta_(j) - phase_theta_(i) - coupling_matrix_(i, j));
  }

  amplitude_r_ += amplitude_dr_ * dt;
  phase_theta_ += phase_dtheta_ * dt;

  phase_theta_ = phase_theta_.unaryExpr([](double x) { return std::fmod(x, 2 * M_PI); });
  ground_multiplier_ = phase_theta_.unaryExpr(
      [this](double x) { return sin(x) > 0 ? ground_clearance_ : ground_penetration_; });

  Eigen::Vector4d feet_x = -d_step_x_.array() * amplitude_r_.array() * (phase_theta_.array().cos());
  Eigen::Vector4d feet_y = -d_step_y_.array() * amplitude_r_.array() * phase_theta_.array().cos();
  Eigen::Vector4d feet_z = ground_multiplier_.array() * phase_theta_.array().sin();

  if ((d_step_x_ == Eigen::Vector4d::Zero()) && (d_step_y_ == Eigen::Vector4d::Zero()))
    feet_z = Eigen::Vector4d::Zero();

  last_time_ = this->get_clock()->now();

  return std::make_tuple(feet_x, feet_y, feet_z);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CentralPatternGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
