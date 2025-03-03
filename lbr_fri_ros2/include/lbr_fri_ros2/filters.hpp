#ifndef LBR_FRI_ROS2__FILTERS_HPP_
#define LBR_FRI_ROS2__FILTERS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "lbr_fri_ros2/types.hpp"

namespace lbr_fri_ros2 {
class ExponentialFilter {
public:
  /**
   * @brief Construct a new ExponentialFilter object.
   *
   */
  ExponentialFilter();

  /**
   * @brief Construct a new ExponentialFilter object.
   *
   * @param[in] tau Time constant in seconds.
   */
  ExponentialFilter(const double &tau);

  /**
   * @brief Initialize the filter from members. Computes the new #alpha_ following
   * alpha = 1.0 - exp(-sample_time / tau).
   *
   * If tau << sample_time => alpha -> 1 (very fast response, no smoothing)
   * If tau >> sample_time => alpha -> 0 (very slow response, heavy smoothing)
   *
   * @param[in] sample_time Sample time in seconds.
   */
  void initialize(const double &sample_time);

  /**
   * @brief Initialize the filter. Computes the new #alpha_ following
   * alpha = 1.0 - exp(-sample_time / tau).
   *
   * If tau << sample_time => alpha -> 1 (very fast response, no smoothing)
   * If tau >> sample_time => alpha -> 0 (very slow response, heavy smoothing)
   *
   * @param[in] tau Time constant in seconds.
   * @param[in] sample_time Sample time in seconds.
   */
  void initialize(const double &tau, const double &sample_time);

  /**
   * @brief Compute the exponential smoothing. Internally computes the new smoothed value following
   * smoothed = alpha * current + (1 - alpha) * previous.
   *
   * @param[in] current The current value.
   * @param[in] previous The previous smoothed value.
   * @return double The returned smoothed value.
   */
  inline double compute(const double &current, const double &previous) {
    return alpha_ * current + (1.0 - alpha_) * previous;
  };

  /**
   * @brief Get the time constant #tau_.
   *
   * @return const double&
   */
  inline const double &get_tau() const { return tau_; };

  /**
   * @brief Get #sample_time_.
   *
   * @return const double&
   */
  inline const double &get_sample_time() const { return sample_time_; };

  /**
   * @brief Get #alpha_.
   *
   * @return const double&
   */
  inline const double &get_alpha() const { return alpha_; };

protected:
  /**
   * @brief Validate alpha in [0, 1].
   *
   * @param[in] alpha Alpha parameter for smoothing.
   * @return true if in [0, 1].
   * @return false if outside [0, 1].
   */
  bool validate_alpha_(const double &alpha);

  double tau_;         /**< Time constant in seconds.*/
  double sample_time_; /**< Sample time in seconds.*/
  double alpha_;       /**< Smoothing parameter in [0, 1].*/
};

class JointExponentialFilterArray {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::JointExponentialFilterArray";

public:
  JointExponentialFilterArray() = default;
  JointExponentialFilterArray(const double &tau);

  void compute(const double *const current, jnt_array_t_ref previous);
  void compute(const_jnt_array_t_ref current, jnt_array_t_ref previous);
  void initialize(const double &sample_time);
  void initialize(const double &tau, const double &sample_time);
  inline const bool &is_initialized() const { return initialized_; };

  void log_info() const;

protected:
  bool initialized_{false};              /**< True if initialized.*/
  ExponentialFilter exponential_filter_; /**< Exponential filter applied to all joints.*/
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__FILTERS_HPP_
