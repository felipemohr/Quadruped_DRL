/**
 * @file InverseKinematics.cpp
 * @author Felipe Mohr (felipe18mohr@gmail.com)
 * @brief Node compute inverse kinematics and publish joint trajectories or joint commands
 * @version 1.0
 * @date 2024-08-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "quadruped_kinematics/InverseKinematics.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <memory>

InverseKinematics::InverseKinematics() : Node("inverse_kinematics")
{
  // TODO: Create ros2 parameters

  RCLCPP_INFO(this->get_logger(), "Inverse Kinematics started");
}

InverseKinematics::~InverseKinematics() {}

void InverseKinematics::CmdIKCallback(
    const quadruped_interfaces::msg::QuadrupedKinematics::SharedPtr msg)
{
}

Eigen::Matrix4d InverseKinematics::getTranslationMatrix(const float x, const float y, const float z)
{
  Eigen::Matrix4d translation_matrix = Eigen::Matrix4d::Identity();
  translation_matrix.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
  return translation_matrix;
}

Eigen::Matrix4d
InverseKinematics::getTransformationMatrix(const geometry_msgs::msg::Vector3 translation,
                                           const geometry_msgs::msg::Vector3 rotation)
{
  Eigen::Vector3d translation_eigen;
  tf2::fromMsg(translation, translation_eigen);

  Eigen::AngleAxisd rollRotation(rotation.x, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchRotation(rotation.y, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawRotation(rotation.z, Eigen::Vector3d::UnitZ());

  Eigen::Quaternion q = rollRotation * pitchRotation * yawRotation;
  Eigen::Matrix3d rotation_eigen = q.toRotationMatrix();

  Eigen::Matrix4d Tm = Eigen::Matrix4d::Identity();
  Tm.block<3, 3>(0, 0) = rotation_eigen;
  Tm.block<3, 1>(0, 3) = translation_eigen;

  return Tm;
}

Eigen::Matrix4d InverseKinematics::getBodyLegTransform(Eigen::Matrix4d bodyTm, Leg leg)
{
  Eigen::Matrix4d bodyLegMatrix;
  switch (leg)
  {
  case Leg::FRONT_LEFT:
    bodyLegMatrix = bodyTm * this->getTranslationMatrix(parameters.body_length / 2,
                                                        parameters.body_width / 2, 0);
    break;
  case Leg::FRONT_RIGHT:
    bodyLegMatrix = bodyTm * this->getTranslationMatrix(parameters.body_length / 2,
                                                        -parameters.body_width / 2, 0);
    break;
  case Leg::REAR_LEFT:
    bodyLegMatrix = bodyTm * this->getTranslationMatrix(-parameters.body_length / 2,
                                                        parameters.body_width / 2, 0);
    break;
  case Leg::REAR_RIGHT:
    bodyLegMatrix = bodyTm * this->getTranslationMatrix(-parameters.body_length / 2,
                                                        -parameters.body_width / 2, 0);
    break;
  }

  return bodyLegMatrix;
}

geometry_msgs::msg::Vector3
InverseKinematics::computeLegJoints(const geometry_msgs::msg::Vector3 foot_pos, Leg leg)
{
  int reflect = (leg == Leg::FRONT_LEFT || leg == Leg::REAR_LEFT) ? 1 : -1;

  float a = sqrt(pow(foot_pos.y, 2) + pow(foot_pos.z, 2) - pow(parameters.leg_l1, 2));
  float A =
      (pow(a, 2) + pow(foot_pos.x, 2) + pow(parameters.leg_l2, 2) - pow(parameters.leg_l3, 2)) /
      (2 * parameters.leg_l2 * sqrt(pow(a, 2) + pow(foot_pos.x, 2)));
  float B =
      (pow(a, 2) + pow(foot_pos.x, 2) - pow(parameters.leg_l2, 2) - pow(parameters.leg_l3, 2)) /
      (2 * parameters.leg_l2 * parameters.leg_l3);

  float theta1 = atan2(foot_pos.y, -foot_pos.z) - atan2(reflect * parameters.leg_l1, a);
  float theta2 = M_PI_2 - atan2(a, foot_pos.x) - atan2(sqrt(1 - pow(A, 2)), A);
  float theta3 = atan2(sqrt(1 - pow(B, 2)), B);

  geometry_msgs::msg::Vector3 leg_joints;
  leg_joints.x = theta1;
  leg_joints.y = -theta2;
  leg_joints.z = -theta3;

  return leg_joints;
}

quadruped_interfaces::msg::JointsAction InverseKinematics::computeQuadrupedJoints(
    const quadruped_interfaces::msg::QuadrupedKinematics cmd_ik)
{
  Eigen::Vector3d fl_foot_pos;
  Eigen::Vector3d fr_foot_pos;
  Eigen::Vector3d rl_foot_pos;
  Eigen::Vector3d rr_foot_pos;

  tf2::fromMsg(cmd_ik.front_left_foot, fl_foot_pos);
  tf2::fromMsg(cmd_ik.front_right_foot, fr_foot_pos);
  tf2::fromMsg(cmd_ik.rear_left_foot, rl_foot_pos);
  tf2::fromMsg(cmd_ik.rear_right_foot, rr_foot_pos);

  if (cmd_ik.use_foot_transforms)
  {
    Eigen::Matrix4d fl_translation = this->getTranslationMatrix(
        parameters.body_length / 2, (parameters.body_width / 2 + parameters.leg_l1),
        -parameters.body_height);
    Eigen::Matrix4d fr_translation = this->getTranslationMatrix(
        parameters.body_length / 2, -(parameters.body_width / 2 + parameters.leg_l1),
        -parameters.body_height);
    Eigen::Matrix4d rl_translation = this->getTranslationMatrix(
        -parameters.body_length / 2, (parameters.body_width / 2 + parameters.leg_l1),
        -parameters.body_height);
    Eigen::Matrix4d rr_translation = this->getTranslationMatrix(
        -parameters.body_length / 2, -(parameters.body_width / 2 + parameters.leg_l1),
        -parameters.body_height);

    fl_foot_pos = (fl_translation * fl_foot_pos.homogeneous()).head<3>();
    fr_foot_pos = (fr_translation * fr_foot_pos.homogeneous()).head<3>();
    rl_foot_pos = (rl_translation * rl_foot_pos.homogeneous()).head<3>();
    rr_foot_pos = (rr_translation * rr_foot_pos.homogeneous()).head<3>();
  }

  Eigen::Matrix4d Tm = this->getTransformationMatrix(cmd_ik.body_translation, cmd_ik.body_rotation);

  Eigen::Matrix4d body_fl_transform = this->getBodyLegTransform(Tm, Leg::FRONT_LEFT);
  Eigen::Matrix4d body_fr_transform = this->getBodyLegTransform(Tm, Leg::FRONT_RIGHT);
  Eigen::Matrix4d body_rl_transform = this->getBodyLegTransform(Tm, Leg::REAR_LEFT);
  Eigen::Matrix4d body_rr_transform = this->getBodyLegTransform(Tm, Leg::REAR_RIGHT);

  Eigen::Vector4d fl_ik_pos = body_fl_transform.inverse() * fl_foot_pos.homogeneous();
  Eigen::Vector4d fr_ik_pos = body_fr_transform.inverse() * fr_foot_pos.homogeneous();
  Eigen::Vector4d rl_ik_pos = body_rl_transform.inverse() * rl_foot_pos.homogeneous();
  Eigen::Vector4d rr_ik_pos = body_rr_transform.inverse() * rr_foot_pos.homogeneous();

  geometry_msgs::msg::Vector3 fl_pos_vector3;
  geometry_msgs::msg::Vector3 fr_pos_vector3;
  geometry_msgs::msg::Vector3 rl_pos_vector3;
  geometry_msgs::msg::Vector3 rr_pos_vector3;
  tf2::toMsg(fl_ik_pos.head(3), fl_pos_vector3);
  tf2::toMsg(fr_ik_pos.head(3), fr_pos_vector3);
  tf2::toMsg(rl_ik_pos.head(3), rl_pos_vector3);
  tf2::toMsg(rr_ik_pos.head(3), rr_pos_vector3);

  geometry_msgs::msg::Vector3 fl_joints = this->computeLegJoints(fl_pos_vector3, Leg::FRONT_LEFT);
  geometry_msgs::msg::Vector3 fr_joints = this->computeLegJoints(fl_pos_vector3, Leg::FRONT_RIGHT);
  geometry_msgs::msg::Vector3 rl_joints = this->computeLegJoints(fl_pos_vector3, Leg::REAR_LEFT);
  geometry_msgs::msg::Vector3 rr_joints = this->computeLegJoints(fl_pos_vector3, Leg::REAR_RIGHT);

  quadruped_interfaces::msg::JointsAction joint_positions;
  joint_positions.position.at(0) = fl_joints.x;
  joint_positions.position.at(1) = fl_joints.y;
  joint_positions.position.at(2) = fl_joints.z;
  joint_positions.position.at(3) = fr_joints.x;
  joint_positions.position.at(4) = fr_joints.y;
  joint_positions.position.at(5) = fr_joints.z;
  joint_positions.position.at(6) = rl_joints.x;
  joint_positions.position.at(7) = rl_joints.y;
  joint_positions.position.at(8) = rl_joints.z;
  joint_positions.position.at(9) = rr_joints.x;
  joint_positions.position.at(10) = rr_joints.y;
  joint_positions.position.at(11) = rr_joints.z;

  return joint_positions;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InverseKinematics>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
