#include "lbr_fri_ros2/control.hpp"

namespace lbr_fri_ros2 {
InvJacCtrlImpl::InvJacCtrlImpl(const std::string &robot_description,
                               const InvJacCtrlParameters &parameters)
    : parameters_(parameters) {
  kinematics_ptr_ = std::make_unique<Kinematics>(robot_description, parameters_.chain_root,
                                                 parameters_.chain_tip);
  set_all_zero_();
  q_gains_ = Eigen::Map<Eigen::Matrix<double, N_JNTS, 1>>(parameters_.joint_gains.data());
  x_gains_ =
      Eigen::Map<Eigen::Matrix<double, CARTESIAN_DOF, 1>>(parameters_.cartesian_gains.data());
}

void InvJacCtrlImpl::compute(const geometry_msgs::msg::Twist::SharedPtr &twist_target,
                             const_jnt_array_t_ref q, jnt_array_t_ref dq) {
  // twist to Eigen
  twist_target_[0] = twist_target->linear.x;
  twist_target_[1] = twist_target->linear.y;
  twist_target_[2] = twist_target->linear.z;
  twist_target_[3] = twist_target->angular.x;
  twist_target_[4] = twist_target->angular.y;
  twist_target_[5] = twist_target->angular.z;

  // compute
  compute_impl_(q, dq);
}

void InvJacCtrlImpl::compute(const_cart_array_t_ref twist_target, const_jnt_array_t_ref q,
                             jnt_array_t_ref dq) {
  // twist to Eigen
  twist_target_[0] = twist_target[0];
  twist_target_[1] = twist_target[1];
  twist_target_[2] = twist_target[2];
  twist_target_[3] = twist_target[3];
  twist_target_[4] = twist_target[4];
  twist_target_[5] = twist_target[5];

  // compute
  compute_impl_(q, dq);
}

void InvJacCtrlImpl::compute(const Eigen::Matrix<double, CARTESIAN_DOF, 1> &twist_target,
                             const_jnt_array_t_ref q, jnt_array_t_ref dq) {
  twist_target_ = twist_target;

  // compute
  compute_impl_(q, dq);
}

void InvJacCtrlImpl::log_info() const {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*** Parameters:");
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   Chain root: %s",
              parameters_.chain_root.c_str());
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   Chain tip: %s", parameters_.chain_tip.c_str());
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   Twist in tip frame: %s",
              parameters_.twist_in_tip_frame ? "true" : "false");
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   Damping: %.3f", parameters_.damping);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   Max linear velocity: %.3f",
              parameters_.max_linear_velocity);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   Max angular velocity: %.3f",
              parameters_.max_angular_velocity);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
              "*   Joint gains: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              parameters_.joint_gains[0], parameters_.joint_gains[1], parameters_.joint_gains[2],
              parameters_.joint_gains[3], parameters_.joint_gains[4], parameters_.joint_gains[5],
              parameters_.joint_gains[6]);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
              "*   Cartesian gains: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              parameters_.cartesian_gains[0], parameters_.cartesian_gains[1],
              parameters_.cartesian_gains[2], parameters_.cartesian_gains[3],
              parameters_.cartesian_gains[4], parameters_.cartesian_gains[5]);
}

void AdmittanceImpl::compute(const Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> &f_ext,
                             const Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> &x,
                             const Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> &dx,
                             Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> &ddx) {
  if ((m_.array() < 0.).any()) {
    throw std::runtime_error("Mass must be positive and greater zero.");
  }
  ddx = (f_ext - b_.asDiagonal() * dx - k_.asDiagonal() * x).array() / m_.array();
}

void AdmittanceImpl::log_info() const {
  {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*** Parameters:");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   Mass: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                parameters_.m[0], parameters_.m[1], parameters_.m[2], parameters_.m[3],
                parameters_.m[4], parameters_.m[5]);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                "*   Damping: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", parameters_.b[0],
                parameters_.b[1], parameters_.b[2], parameters_.b[3], parameters_.b[4],
                parameters_.b[5]);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                "*   Stiffness: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", parameters_.k[0],
                parameters_.k[1], parameters_.k[2], parameters_.k[3], parameters_.k[4],
                parameters_.k[5]);
  }
}

void InvJacCtrlImpl::compute_impl_(const_jnt_array_t_ref q, jnt_array_t_ref dq) {
  // clip velocity
  twist_target_.head(3).unaryExpr([&](double v) {
    return std::clamp(v, -parameters_.max_linear_velocity, parameters_.max_linear_velocity);
  });
  twist_target_.tail(3).unaryExpr([&](double v) {
    return std::clamp(v, -parameters_.max_angular_velocity, parameters_.max_angular_velocity);
  });

  // if desired, transform to tip frame
  if (parameters_.twist_in_tip_frame) {
    auto chain_tip_frame = kinematics_ptr_->compute_fk(q);
    twist_target_.topRows(3) =
        Eigen::Matrix3d::Map(chain_tip_frame.M.data).transpose() * twist_target_.topRows(3);
    twist_target_.bottomRows(3) =
        Eigen::Matrix3d::Map(chain_tip_frame.M.data).transpose() * twist_target_.bottomRows(3);
  }

  twist_target_ = x_gains_.asDiagonal() * twist_target_;

  // compute jacobian
  auto jacobian = kinematics_ptr_->compute_jacobian(q);
  jacobian_inv_ = pinv(jacobian.data, parameters_.damping);

  // compute target joint veloctiy and map it to dq
  Eigen::Map<Eigen::Matrix<double, N_JNTS, 1>>(dq.data()) =
      q_gains_.asDiagonal() * jacobian_inv_ * twist_target_;
}

void InvJacCtrlImpl::set_all_zero_() {
  std::fill(q_.begin(), q_.end(), 0.0);
  jacobian_inv_.setZero();
  twist_target_.setZero();
  q_gains_.setZero();
  x_gains_.setZero();
}
} // namespace lbr_fri_ros2
