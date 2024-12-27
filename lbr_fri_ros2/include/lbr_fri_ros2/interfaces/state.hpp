#ifndef LBR_FRI_ROS2__INTERFACES__STATE_HPP_
#define LBR_FRI_ROS2__INTERFACES__STATE_HPP_
#include <atomic>
#include <string>
#include <array>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friClientVersion.h"
#include "friLBRClient.h"

#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/filters.hpp"

namespace lbr_fri_ros2 {
struct StateInterfaceParameters {
  double external_torque_cutoff_frequency; /*Hz*/
  double measured_torque_cutoff_frequency; /*Hz*/
};

class StateInterface {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::StateInterface";

  // ROS IDL types
  using idl_state_t = lbr_fri_idl::msg::LBRState;
  using const_idl_state_t_ref = const idl_state_t &;
  using idl_joint_pos_t = idl_state_t::_measured_joint_position_type;
  using const_idl_joint_pos_t_ref = const idl_joint_pos_t &;

  // FRI types
  using fri_state_t = KUKA::FRI::LBRState;
  using const_fri_state_t_ref = const fri_state_t &;
  using fri_session_state_t = KUKA::FRI::ESessionState;

public:
  StateInterface() = delete;
  StateInterface(const StateInterfaceParameters &state_interface_parameters = {10.0, 10.0});

  inline const_idl_state_t_ref get_state() const { return state_; };

  void set_state(const_fri_state_t_ref state);
  void set_state_open_loop(const_fri_state_t_ref state, const_idl_joint_pos_t_ref joint_position);

  inline void uninitialize() { state_initialized_ = false; }
  inline bool is_initialized() const { return state_initialized_; };

  void log_info() const;

protected:
  void init_filters_();

  std::atomic_bool state_initialized_;
  idl_state_t state_;
  StateInterfaceParameters parameters_;
  JointExponentialFilterArray external_torque_filter_, measured_torque_filter_;

  // I/O 状态字段
  std::array<double, 2> measured_analog_io_;
  std::array<double, 2> commanded_analog_io_;
  std::array<bool, 2> measured_boolean_io_;
  std::array<bool, 2> commanded_boolean_io_;
  std::array<uint64_t, 2> measured_digital_io_;
  std::array<uint64_t, 2> commanded_digital_io_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__INTERFACES__STATE_HPP_
