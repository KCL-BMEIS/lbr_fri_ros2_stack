#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/rclcpp.hpp"

#include "friClientIf.h"
#include "friLBRState.h"
#include "lbr_fri_idl/msg/lbr_state.hpp"

#include "lbr_base_position_command_node.hpp"

namespace lbr_demos {
class PoseControlNode : public LBRBasePositionCommandNode {
protected:
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;

  lbr_fri_idl::msg::LBRState current_lbr_state_; // robot state, including joint positions
  geometry_msgs::msg::Pose current_pose_;        // current pose of robot

  KDL::Chain chain_; // robot kinematics chain exetracted from robot URDF file

public:
  PoseControlNode(const rclcpp::NodeOptions &options)
      : LBRBasePositionCommandNode("pose_control", options) {
    KDL::Tree robot_tree;
    if (!kdl_parser::treeFromString(this->robot_description_, robot_tree)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to construct kdl tree.");
      return;
    }

    this->declare_parameter<std::string>("base_link", "lbr_link_0");
    this->declare_parameter<std::string>("end_effector_link", "lbr_link_ee");

    std::string root_link = this->get_parameter("base_link").as_string();
    std::string tip_link = this->get_parameter("end_effector_link").as_string();

    if (!robot_tree.getChain(root_link, tip_link, chain_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get chain from tree.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Get chain from tree successfully.");
    }

    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("state/pose", 1);
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "command/pose", 1, std::bind(&PoseControlNode::on_pose_, this, std::placeholders::_1));
  }

protected:
  void on_lbr_state_(const lbr_fri_idl::msg::LBRState::SharedPtr lbr_state) override {
    current_lbr_state_ = *lbr_state;

    double joint_position[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
    for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i) {
      joint_position[i] = current_lbr_state_.measured_joint_position[i];
    }
    current_pose_ = compute_fk_(joint_position);
    pose_pub_->publish(current_pose_);
  }

  void on_pose_(const geometry_msgs::msg::Pose &msg) {
    if (current_lbr_state_.session_state == KUKA::FRI::COMMANDING_ACTIVE) {
      KDL::JntArray current_joint_positions(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

      for (unsigned int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i) {
        current_joint_positions(i) = current_lbr_state_.measured_joint_position[i];
      }

      lbr_fri_idl::msg::LBRJointPositionCommand joint_position_command =
          compute_ik_(msg, current_joint_positions);
      lbr_joint_position_command_pub_->publish(joint_position_command);
    }
  }

  geometry_msgs::msg::Pose compute_fk_(double *position_array_ptr) {
    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(chain_);
    KDL::JntArray joint_positions = KDL::JntArray(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

    for (unsigned int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i) {
      joint_positions(i) = position_array_ptr[i];
    }

    KDL::Frame pose_tmp;           // pose described in data type KDL::Frame
    geometry_msgs::msg::Pose pose; // described in geometry_msgs::msg::Pose

    if (fk_solver.JntToCart(joint_positions, pose_tmp) < 0) {
      RCLCPP_ERROR(this->get_logger(), "FK Solver to calculate JointToCartesian failed.");
    } else {
      // Position
      pose.position.x = pose_tmp.p.x();
      pose.position.y = pose_tmp.p.y();
      pose.position.z = pose_tmp.p.z();

      // Orientation
      double x, y, z, w;
      pose_tmp.M.GetQuaternion(x, y, z, w); // get quaternion
      pose.orientation.x = x;
      pose.orientation.y = y;
      pose.orientation.z = z;
      pose.orientation.w = w;
    }

    return pose;
  }

  lbr_fri_idl::msg::LBRJointPositionCommand
  compute_ik_(const geometry_msgs::msg::Pose &desired_pose,
              KDL::JntArray &current_joint_positions) {
    KDL::ChainIkSolverPos_LMA ik_solver(chain_);
    KDL::JntArray result_joint_positions = KDL::JntArray(chain_.getNrOfJoints());
    lbr_fri_idl::msg::LBRJointPositionCommand joint_position_command;

    // transfer data type 'geometry::msg::Pose' to be 'KDL::Frame'
    KDL::Vector position(desired_pose.position.x, desired_pose.position.y, desired_pose.position.z);
    KDL::Rotation rotation =
        KDL::Rotation::Quaternion(desired_pose.orientation.x, desired_pose.orientation.y,
                                  desired_pose.orientation.z, desired_pose.orientation.w);
    KDL::Frame desired_pose_tmp(rotation, position);

    auto start = std::chrono::high_resolution_clock::now();
    int ik_result =
        ik_solver.CartToJnt(current_joint_positions, desired_pose_tmp, result_joint_positions);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> execution_time = end - start;
    RCLCPP_DEBUG(this->get_logger(), "IK solver execution time: %f ms", execution_time.count());

    if (ik_result < 0) // if solving failed, 'ik_result' would be less than 0
    {
      RCLCPP_ERROR(this->get_logger(), "Inverse kinematics failed.");
    } else {
      for (unsigned int i = 0; i < result_joint_positions.data.size(); ++i) {
        joint_position_command.joint_position[i] = result_joint_positions(i);
      }
    }

    return joint_position_command;
  }
};
} // namespace lbr_demos

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lbr_demos::PoseControlNode)
