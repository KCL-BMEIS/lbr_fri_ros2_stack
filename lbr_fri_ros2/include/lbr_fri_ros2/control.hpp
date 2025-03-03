#ifndef LBR_FRI_ROS2__CONTROL_HPP_
#define LBR_FRI_ROS2__CONTROL_HPP_

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

#include "eigen3/Eigen/Core"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "lbr_fri_ros2/kinematics.hpp"
#include "lbr_fri_ros2/pinv.hpp"
#include "lbr_fri_ros2/types.hpp"

namespace lbr_fri_ros2 {
struct InvJacCtrlParameters {
  std::string chain_root = "lbr_link_0";
  std::string chain_tip = "lbr_link_ee";
  bool twist_in_tip_frame = true;
  double damping = 0.2;
  double max_linear_velocity = 0.1;
  double max_angular_velocity = 0.1;
  jnt_array_t joint_gains = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  cart_array_t cartesian_gains = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

class InvJacCtrlImpl {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::InvJacCtrlImpl";

public:
  InvJacCtrlImpl(const std::string &robot_description, const InvJacCtrlParameters &parameters);

  void compute(const geometry_msgs::msg::Twist::SharedPtr &twist_target, const_jnt_array_t_ref q,
               jnt_array_t_ref dq);
  void compute(const_cart_array_t_ref twist_target, const_jnt_array_t_ref q, jnt_array_t_ref dq);
  void compute(const Eigen::Matrix<double, CARTESIAN_DOF, 1> &twist_target, const_jnt_array_t_ref q,
               jnt_array_t_ref dq);

  inline const std::unique_ptr<Kinematics> &get_kinematics_ptr() const { return kinematics_ptr_; };

  void log_info() const;

protected:
  void compute_impl_(const_jnt_array_t_ref q, jnt_array_t_ref dq);
  void set_all_zero_();

  InvJacCtrlParameters parameters_;

  jnt_array_t q_;
  std::unique_ptr<Kinematics> kinematics_ptr_;
  Eigen::Matrix<double, N_JNTS, CARTESIAN_DOF> jacobian_inv_;
  Eigen::Matrix<double, CARTESIAN_DOF, 1> twist_target_;
  Eigen::Matrix<double, N_JNTS, 1> q_gains_;
  Eigen::Matrix<double, CARTESIAN_DOF, 1> x_gains_;
};

struct AdmittanceParameters {
  AdmittanceParameters() = delete;
  AdmittanceParameters(const_cart_array_t_ref m = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
                       const_cart_array_t_ref b = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
                       const_cart_array_t_ref k = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
      : m(m), b(b), k(k) {
    if (std::any_of(m.cbegin(), m.cend(), [](const double &m_i) { return m_i <= 0.0; })) {
      throw std::runtime_error("Mass must be positive and greater zero.");
    }
    if (std::any_of(b.cbegin(), b.cend(), [](const double &b_i) { return b_i < 0.0; })) {
      throw std::runtime_error("Damping must be positive.");
    }
    if (std::any_of(k.cbegin(), k.cend(), [](const double &k_i) { return k_i < 0.0; })) {
      throw std::runtime_error("Stiffness must be positive.");
    }
  }

  cart_array_t m = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  cart_array_t b = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  cart_array_t k = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

class AdmittanceImpl {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::AdmittanceImpl";

public:
  AdmittanceImpl() = delete;
  AdmittanceImpl(const AdmittanceParameters &parameters) : parameters_(parameters) {
    m_ = Eigen::Map<Eigen::Matrix<double, CARTESIAN_DOF, 1>>(parameters_.m.data());
    b_ = Eigen::Map<Eigen::Matrix<double, CARTESIAN_DOF, 1>>(parameters_.b.data());
    k_ = Eigen::Map<Eigen::Matrix<double, CARTESIAN_DOF, 1>>(parameters_.k.data());
  }

  void compute(const Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> &f_ext,
               const Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> &x,
               const Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> &dx,
               Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> &ddx);

  void log_info() const;

protected:
  AdmittanceParameters parameters_;

  Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> m_, b_, k_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__CONTROL_HPP_
