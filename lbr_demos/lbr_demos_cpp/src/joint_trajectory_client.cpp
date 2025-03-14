#include <chrono>
#include <memory>
#include <string>

// include fri for number of joints
#include "friLBRState.h"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class JointTrajectoryClient : public rclcpp::Node {
public:
  JointTrajectoryClient(const std::string &node_name) : Node(node_name) {

    joint_trajectory_action_client_ =
        rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            this, "joint_trajectory_controller/follow_joint_trajectory");

    while (!joint_trajectory_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the action server. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for action server to become available...");
    }
    RCLCPP_INFO(this->get_logger(), "Action server available.");
  };

  void execute(const std::vector<double> &positions, const int32_t &sec_from_start = 15) {
    if (positions.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
      RCLCPP_ERROR(this->get_logger(), "Invalid number of joint positions.");
      return;
    }

    control_msgs::action::FollowJointTrajectory::Goal joint_trajectory_goal;
    int32_t goal_sec_tolerance = 1;
    joint_trajectory_goal.goal_time_tolerance.sec = goal_sec_tolerance;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.velocities.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0);
    point.time_from_start.sec = sec_from_start;

    for (std::size_t i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i) {
      joint_trajectory_goal.trajectory.joint_names.push_back("A" + std::to_string(i + 1));
    }

    joint_trajectory_goal.trajectory.points.push_back(point);

    // send goal
    auto goal_future = joint_trajectory_action_client_->async_send_goal(joint_trajectory_goal);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_future);
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Goal was accepted by server.");

    // wait for result
    auto result_future = joint_trajectory_action_client_->async_get_result(goal_handle);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future,
                                       std::chrono::seconds(sec_from_start + goal_sec_tolerance));
    if (result_future.get().result->error_code !=
        control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL) {
      RCLCPP_ERROR(this->get_logger(), "Failed to execute joint trajectory.");
      return;
    }
  }

protected:
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
      joint_trajectory_action_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto joint_trajectory_client = std::make_shared<JointTrajectoryClient>("joint_trajectory_client");

  // rotate odd joints
  RCLCPP_INFO(joint_trajectory_client->get_logger(), "Rotating odd joints.");
  joint_trajectory_client->execute({
      1.0,
      0.0,
      1.0,
      0.0,
      1.0,
      0.0,
      1.0,
  });

  // move to zero position
  RCLCPP_INFO(joint_trajectory_client->get_logger(), "Moving to zero position.");
  joint_trajectory_client->execute({
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
  });

  rclcpp::shutdown();
  return 0;
}
