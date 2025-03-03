#include "lbr_fri_ros2/kinematics.hpp"

namespace lbr_fri_ros2 {
Kinematics::Kinematics(const std::string &robot_description, const std::string &chain_root,
                       const std::string &chain_tip) {
  if (!kdl_parser::treeFromString(robot_description, tree_)) {
    std::string err = "Failed to construct kdl tree from robot description.";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), err.c_str());
    throw std::runtime_error(err);
  }
  if (!tree_.getChain(chain_root, chain_tip, chain_)) {
    std::string err = "Failed to construct kdl chain from robot description.";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), err.c_str());
    throw std::runtime_error(err);
  }
  if (chain_.getNrOfJoints() != N_JNTS) {
    std::string err = "Invalid number of joints in chain.";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), err.c_str());
    throw std::runtime_error(err);
  }
  jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jacobian_.resize(N_JNTS);
  q_.resize(N_JNTS);
  q_.data.setZero();
}

const KDL::Jacobian &Kinematics::compute_jacobian(const_jnt_array_t_ref q) {
  q_.data = Eigen::Map<const Eigen::Matrix<double, N_JNTS, 1>>(q.data());
  jacobian_solver_->JntToJac(q_, jacobian_);
  return jacobian_;
}

const KDL::Jacobian &Kinematics::compute_jacobian(const std::vector<double> &q) {
  if (q.size() != N_JNTS) {
    std::string err = "Invalid number of joint positions.";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), err.c_str());
    throw std::runtime_error(err);
  }
  q_.data = Eigen::Map<const Eigen::Matrix<double, N_JNTS, 1>>(q.data());
  jacobian_solver_->JntToJac(q_, jacobian_);
  return jacobian_;
}

const KDL::Frame &Kinematics::compute_fk(const_jnt_array_t_ref q) {
  q_.data = Eigen::Map<const Eigen::Matrix<double, N_JNTS, 1>>(q.data());
  fk_solver_->JntToCart(q_, chain_tip_frame_);
  return chain_tip_frame_;
}

const KDL::Frame &Kinematics::compute_fk(const std::vector<double> &q) {
  if (q.size() != N_JNTS) {
    std::string err = "Invalid number of joint positions.";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), err.c_str());
    throw std::runtime_error(err);
  }
  q_.data = Eigen::Map<const Eigen::Matrix<double, N_JNTS, 1>>(q.data());
  fk_solver_->JntToCart(q_, chain_tip_frame_);
  return chain_tip_frame_;
}
} // namespace lbr_fri_ros2
