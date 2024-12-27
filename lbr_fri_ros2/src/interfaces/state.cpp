#include "lbr_fri_ros2/interfaces/state.hpp"

namespace lbr_fri_ros2 {
StateInterface::StateInterface(const StateInterfaceParameters &state_interface_parameters)
    : state_initialized_(false), parameters_(state_interface_parameters) {}

void StateInterface::set_state(const_fri_state_t_ref state) {
  state_.client_command_mode = state.getClientCommandMode();
#if FRI_CLIENT_VERSION_MAJOR == 1
  std::memcpy(state_.commanded_joint_position.data(), state.getCommandedJointPosition(),
              sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
#endif
  std::memcpy(state_.commanded_torque.data(), state.getCommandedTorque(),
              sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
  state_.connection_quality = state.getConnectionQuality();
  state_.control_mode = state.getControlMode();
  state_.drive_state = state.getDriveState();
  external_torque_filter_.compute(state.getExternalTorque(), state_.external_torque);

  if (state.getSessionState() == fri_session_state_t::COMMANDING_WAIT ||
      state.getSessionState() == fri_session_state_t::COMMANDING_ACTIVE) {
    std::memcpy(state_.ipo_joint_position.data(), state.getIpoJointPosition(),
                sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
  }

  std::memcpy(state_.measured_joint_position.data(), state.getMeasuredJointPosition(),
              sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
  measured_torque_filter_.compute(state.getMeasuredTorque(), state_.measured_torque);

  state_.operation_mode = state.getOperationMode();
  state_.overlay_type = state.getOverlayType();
  state_.safety_state = state.getSafetyState();
  state_.sample_time = state.getSampleTime();
  state_.session_state = state.getSessionState();
  state_.time_stamp_nano_sec = state.getTimestampNanoSec();
  state_.time_stamp_sec = state.getTimestampSec();
  state_.tracking_performance = state.getTrackingPerformance();

  // 获取布尔 I/O 数据
  measured_boolean_io_[0] = state.getBooleanIOValue("boolean_io_0");
  measured_boolean_io_[1] = state.getBooleanIOValue("boolean_io_1");

  // 获取数字 I/O 数据
  measured_digital_io_[0] = state.getDigitalIOValue("digital_io_0");
  measured_digital_io_[1] = state.getDigitalIOValue("digital_io_1");

  // 获取模拟 I/O 数据
  measured_analog_io_[0] = state.getAnalogIOValue("analog_io_0");
  measured_analog_io_[1] = state.getAnalogIOValue("analog_io_1");

  // 这里可以根据需求填充 `commanded_xxx_io_`，例如从其他来源读取或默认设置为测量值
  commanded_boolean_io_ = measured_boolean_io_;
  commanded_digital_io_ = measured_digital_io_;
  commanded_analog_io_ = measured_analog_io_;

  if (!external_torque_filter_.is_initialized() || !measured_torque_filter_.is_initialized()) {
    // initialize once state_ is available
    init_filters_();
  }

  state_initialized_ = true;
}


void StateInterface::set_state_open_loop(const_fri_state_t_ref state,
                                         const_idl_joint_pos_t_ref joint_position) {
  set_state(state); // 重用已有逻辑
  std::memcpy(state_.measured_joint_position.data(), joint_position.data(),
              sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
}

void StateInterface::init_filters_() {
  external_torque_filter_.initialize(parameters_.external_torque_cutoff_frequency,
                                     state_.sample_time);
  measured_torque_filter_.initialize(parameters_.measured_torque_cutoff_frequency,
                                     state_.sample_time);
}

void StateInterface::log_info() const {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*** Parameters:");
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   external_torque_cutoff_frequency: %.1f Hz",
              parameters_.external_torque_cutoff_frequency);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   measured_torque_cutoff_frequency: %.1f Hz",
              parameters_.measured_torque_cutoff_frequency);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   Analog IO: Measured [%f, %f], Commanded [%f, %f]",
              measured_analog_io_[0], measured_analog_io_[1],
              commanded_analog_io_[0], commanded_analog_io_[1]);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   Boolean IO: Measured [%d, %d], Commanded [%d, %d]",
              measured_boolean_io_[0], measured_boolean_io_[1],
              commanded_boolean_io_[0], commanded_boolean_io_[1]);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   Digital IO: Measured [%lu, %lu], Commanded [%lu, %lu]",
              measured_digital_io_[0], measured_digital_io_[1],
              commanded_digital_io_[0], commanded_digital_io_[1]);
}
} // namespace lbr_fri_ros2
