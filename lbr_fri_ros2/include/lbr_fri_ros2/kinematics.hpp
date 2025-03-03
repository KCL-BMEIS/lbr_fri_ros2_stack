#ifndef LBR_FRI_ROS2__KINEMATICS_HPP_
#define LBR_FRI_ROS2__KINEMATICS_HPP_

#include <string>

#include "eigen3/Eigen/Core"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/types.hpp"

namespace lbr_fri_ros2 {
class Kinematics {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::Kinematics";

public:
  Kinematics(const std::string &robot_description, const std::string &chain_root = "lbr_link_0",
             const std::string &chain_tip = "lbr_link_ee");

  // internally computes the jacobian and return a reference to it
  const KDL::Jacobian &compute_jacobian(const_jnt_array_t_ref q);
  const KDL::Jacobian &compute_jacobian(const std::vector<double> &q);

  // forward kinematics
  const KDL::Frame &compute_fk(const_jnt_array_t_ref q);
  const KDL::Frame &compute_fk(const std::vector<double> &q);

protected:
  KDL::Tree tree_;
  KDL::Chain chain_;

  // solvers
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  // robot state
  KDL::JntArray q_;

  // forward kinematics
  KDL::Frame chain_tip_frame_;

  // jacobian
  KDL::Jacobian jacobian_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__KINEMATICS_HPP_
