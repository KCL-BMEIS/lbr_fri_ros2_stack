#ifndef LBR_FRI_ROS2__FT_ESTIMATOR_HPP_
#define LBR_FRI_ROS2__FT_ESTIMATOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "eigen3/Eigen/Core"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/kinematics.hpp"
#include "lbr_fri_ros2/pinv.hpp"
#include "lbr_fri_ros2/types.hpp"
#include "lbr_fri_ros2/worker.hpp"

namespace lbr_fri_ros2 {
class FTEstimatorImpl {
  /**
   * @brief A class to estimate force-torques from external joint torque readings. Note that only
   * forces beyond a specified threshold are returned. The specified threshold is removed from the
   * estimated force-torque.
   *
   */
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::FTEstimatorImpl";

public:
  FTEstimatorImpl(const std::string &robot_description,
                  const std::string &chain_root = "lbr_link_0",
                  const std::string &chain_tip = "lbr_link_ee",
                  const_cart_array_t_ref f_ext_th = {2., 2., 2., 0.5, 0.5, 0.5},
                  const double &damping = 0.2);
  void compute();
  void reset();

  inline void get_f_ext(cart_array_t_ref f_ext) const {
    Eigen::Map<Eigen::Matrix<double, CARTESIAN_DOF, 1>>(f_ext.data()) = f_ext_;
  }
  inline void get_f_ext_tf(cart_array_t_ref f_ext) const {
    Eigen::Map<Eigen::Matrix<double, CARTESIAN_DOF, 1>>(f_ext.data()) = f_ext_tf_;
  }
  inline void set_tau_ext(const_jnt_array_t_ref tau_ext) {
    tau_ext_ = Eigen::Map<const Eigen::Matrix<double, N_JNTS, 1>>(tau_ext.data());
  }
  inline void set_q(const_jnt_array_t_ref q) { q_ = q; }

protected:
  // force threshold
  cart_array_t f_ext_th_;

  // damping for pseudo-inverse of Jacobian
  double damping_;

  // joint positions and external joint torques
  jnt_array_t q_;

  // kinematics
  std::unique_ptr<Kinematics> kinematics_ptr_;

  // force estimation
  Eigen::Matrix<double, N_JNTS, CARTESIAN_DOF> jacobian_inv_;
  Eigen::Matrix<double, N_JNTS, 1> tau_ext_;
  Eigen::Matrix<double, CARTESIAN_DOF, 1> f_ext_raw_, f_ext_, f_ext_tf_;
};

class FTEstimator : public Worker {
  /**
   * @brief A simple class to run the FTEstimatorImpl asynchronously at a specified update rate.
   *
   */
public:
  FTEstimator(const std::shared_ptr<FTEstimatorImpl> ft_estimator,
              const std::uint16_t &update_rate = 100);

  inline std::string LOGGER_NAME() const override { return "lbr_fri_ros2::FTEstimator"; };

protected:
  void perform_work_() override;

  std::shared_ptr<FTEstimatorImpl> ft_estimator_impl_ptr_;
  std::uint16_t update_rate_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__FT_ESTIMATOR_HPP_
