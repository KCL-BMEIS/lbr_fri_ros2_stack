#include "lbr_fri_ros2/ft_estimator.hpp"

namespace lbr_fri_ros2 {
FTEstimatorImpl::FTEstimatorImpl(const std::string &robot_description,
                                 const std::string &chain_root, const std::string &chain_tip,
                                 const_cart_array_t_ref f_ext_th, const double &damping)
    : f_ext_th_(f_ext_th), damping_(damping) {
  kinematics_ptr_ = std::make_unique<Kinematics>(robot_description, chain_root, chain_tip);
  reset();
}

void FTEstimatorImpl::compute() {
  auto jacobian = kinematics_ptr_->compute_jacobian(q_);
  jacobian_inv_ = pinv(jacobian.data, damping_);
  f_ext_raw_ = jacobian_inv_.transpose() * tau_ext_;
  int i = -1;
  f_ext_ = f_ext_raw_.unaryExpr([&](double v) {
    ++i;
    if (std::abs(v) < f_ext_th_[i]) {
      return 0.;
    } else {
      return std::copysign(1., v) * (std::abs(v) - f_ext_th_[i]);
    }
  });

  // rotate into chain tip frame
  auto chain_tip_frame = kinematics_ptr_->compute_fk(q_);
  f_ext_tf_.topRows(3) = Eigen::Matrix3d::Map(chain_tip_frame.M.data) * f_ext_.topRows(3);
  f_ext_tf_.bottomRows(3) = Eigen::Matrix3d::Map(chain_tip_frame.M.data) * f_ext_.bottomRows(3);
}

void FTEstimatorImpl::reset() {
  std::for_each(q_.begin(), q_.end(), [](double &q_i) { q_i = 0.; });
  tau_ext_.setZero();
  f_ext_raw_.setZero();
  f_ext_.setZero();
  f_ext_tf_.setZero();
  jacobian_inv_.setZero();
}

FTEstimator::FTEstimator(const std::shared_ptr<FTEstimatorImpl> ft_estimator_impl_ptr,
                         const std::uint16_t &update_rate)
    : ft_estimator_impl_ptr_(ft_estimator_impl_ptr), update_rate_(update_rate) {}

void FTEstimator::perform_work_() {
  running_ = true;
  while (rclcpp::ok() && !should_stop_) {
    auto start = std::chrono::high_resolution_clock::now();
    ft_estimator_impl_ptr_->compute();
    std::this_thread::sleep_until(start + std::chrono::nanoseconds(static_cast<int>(
                                              1.e9 / static_cast<double>(update_rate_))));
  }
};
} // namespace lbr_fri_ros2
