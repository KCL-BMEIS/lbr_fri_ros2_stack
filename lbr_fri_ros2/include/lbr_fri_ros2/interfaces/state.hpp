#ifndef LBR_FRI_ROS2__INTERFACES__STATE_HPP_
#define LBR_FRI_ROS2__INTERFACES__STATE_HPP_

#include <atomic>
#include <cstring>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friClientVersion.h"

#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/filters.hpp"
#include "lbr_fri_ros2/types.hpp"

namespace lbr_fri_ros2 {
struct StateInterfaceParameters {
  double external_torque_tau; /*seconds*/
  double measured_torque_tau; /*seconds*/
};

class StateInterface {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::StateInterface";

public:
  StateInterface() = delete;
  StateInterface(const StateInterfaceParameters &state_interface_parameters = {0.04, 0.04});

  inline const_idl_state_t_ref get_state() const { return state_; };

  void set_state(const_fri_state_t_ref state);
  void set_state_open_loop(const_fri_state_t_ref state, const_jnt_array_t_ref joint_position);

  inline void uninitialize() { state_initialized_ = false; }
  inline bool is_initialized() const { return state_initialized_; };

  void log_info() const;

protected:
  void init_filters_();

  std::atomic_bool state_initialized_;
  idl_state_t state_;
  StateInterfaceParameters parameters_;
  JointExponentialFilterArray external_torque_filter_, measured_torque_filter_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__INTERFACES__STATE_HPP_
