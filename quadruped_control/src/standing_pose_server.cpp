#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp" 
#include "trajectory_msgs/msg/joint_trajectory.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("standing_pose_server");

  std::vector<std::string> joint_names{"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
                                       "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
                                       "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", 
                                       "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
  std::vector<double> joint_positions{0.0, 0.78, -1.57, 
                                      0.0, 0.78, -1.57, 
                                      0.0, 0.78, -1.57, 
                                      0.0, 0.78, -1.57};

  trajectory_msgs::msg::JointTrajectoryPoint joint_point;
  joint_point.positions = joint_positions;
  joint_point.time_from_start.sec = 3.0;

  auto standing_pose_msg = trajectory_msgs::msg::JointTrajectory();
  standing_pose_msg.joint_names = joint_names;
  standing_pose_msg.points.push_back(joint_point);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher =
    node->create_publisher<trajectory_msgs::msg::JointTrajectory>("trajectory_controller/joint_trajectory", 10);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service = 
    node->create_service<std_srvs::srv::Trigger>(
      "standing_pose", 
      [node, standing_pose_msg, joint_trajectory_publisher]
      (const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        (void) request;

        joint_trajectory_publisher->publish(standing_pose_msg);

        response->success = true;
        response->message = "Quadruped reseting to standing pose";

        RCLCPP_INFO(node->get_logger(), response->message.c_str());
      }
    );

  RCLCPP_INFO(node->get_logger(), "Standing Pose Server ready!");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
