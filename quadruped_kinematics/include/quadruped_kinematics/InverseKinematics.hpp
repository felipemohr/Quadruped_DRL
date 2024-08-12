/**
 * @file InverseKinematics.hpp
 * @author Felipe Mohr (felipe18mohr@gmail.com)
 * @brief Node compute inverse kinematics and publish joint trajectories or joint commands
 * @version 1.0
 * @date 2024-08-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef INVERSE_KINEMATICS_HPP
#define INVERSE_KINEMATICS_HPP

#include "geometry_msgs/msg/vector3.hpp"
#include "quadruped_interfaces/msg/joints_action.hpp"
#include "quadruped_interfaces/msg/quadruped_kinematics.hpp"
#include "rclcpp/rclcpp.hpp"

#include <Eigen/Geometry>
#include <memory>

class InverseKinematics : public rclcpp::Node
{
public:
  /** @brief Enum class for the Leg */
  enum class Leg
  {
    FRONT_LEFT,
    FRONT_RIGHT,
    REAR_LEFT,
    REAR_RIGHT
  };

  /** @brief Class containing the kinematics parameters of a quadruped robot */
  class KinematicsParameters
  {
  public:
    /** @brief Length of the robot (measured between the front and rear hip joints) */
    double body_length;
    /** @brief Width of the robot (measured between the left and right hip joints) */
    double body_width;
    /** @brief Height of the robot (measured from the hip joints to the feet) */
    double body_height;

    /** @brief Distance between hip joints and thigh joints */
    double leg_l1;
    /** @brief Distance between thigh joints and calf joints */
    double leg_l2;
    /** @brief Distance between calf joints and the feet */
    double leg_l3;
  };

  /** @brief Construct a new Inverse Kinematics object */
  InverseKinematics();
  /** @brief Destroy the Inverse Kinematics object */
  ~InverseKinematics();

private:
  /**
   * @brief Callback function to receive inverse kinematics commands and send data to the publisher
   *
   * @param msg A shared pointer to incoming Joints Action message
   */
  void CmdIKCallback(const quadruped_interfaces::msg::QuadrupedKinematics::SharedPtr msg);

  /**
   * @brief Get the translation matrix from x, y, z values
   *
   * @param x The x translation
   * @param y The y translation
   * @param z The z translation
   * @return Eigen::Matrix4d The translation matrix
   */
  Eigen::Matrix4d getTranslationMatrix(const float x, const float y, const float z);

  /**
   * @brief Get the transformation matrix from translation and rotation
   *
   * @param translation The Vector3 translation
   * @param rotation The Vector3 rotation
   * @return Eigen::Matrix4d The transformation matrix
   */
  Eigen::Matrix4d getTransformationMatrix(const geometry_msgs::msg::Vector3 translation,
                                          const geometry_msgs::msg::Vector3 rotation);

  /**
   * @brief Get transformation matrix from a specific leg to the center of the base
   *
   * @param bodyTm The transformation matrix of the body
   * @param leg The leg identification
   * @return Eigen::Matrix4d The transformation of the leg hip joint to the center of the base
   */
  Eigen::Matrix4d getBodyLegTransform(Eigen::Matrix4d bodyTm, Leg leg);

  /**
   * @brief Compute the leg hip, thigh and calf joints to achieve the desired foot position
   *
   * @param foot_pos The desired foot position w.r.t. the hip joint
   * @param leg The leg identification
   * @return geometry_msgs::msg::Vector3 The hip, thigh and calf joints, respectively
   */
  geometry_msgs::msg::Vector3 computeLegJoints(const geometry_msgs::msg::Vector3 foot_pos, Leg leg);

  /**
   * @brief Compute all the quadruped joints from the desired foot positions and body pose
   *
   * @param cmd_ik The desired foot positions and body translation and rotation
   * @return quadruped_interfaces::msg::JointsAction Message containing all the joint positions
   */
  quadruped_interfaces::msg::JointsAction
  computeQuadrupedJoints(const quadruped_interfaces::msg::QuadrupedKinematics cmd_ik);

  /** @brief The kinematics parameters of the quadruped robot */
  KinematicsParameters parameters;
};

#endif // INVERSE_KINEMATICS_HPP
