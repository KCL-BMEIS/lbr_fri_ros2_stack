#include "lbr_fri_ros2/interfaces/state.hpp"

#include <cstring> // for std::memcpy
#include <stdexcept>
#include <vector>
#include <cmath> // for std::isnan

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

  // IO functionalities
  // Validate analog IO values
  for (size_t i = 0; i < NUM_ANALOG_IO; ++i) {
    double value = state.getAnalogIOValue(("analog_io_" + std::to_string(i + 1)).c_str());
    if (value < 0.0 || value > 10.0) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger(LOGGER_NAME),
                         "Analog IO value out of range [0, 10]: " << value);
      value = 0.0; // Default to 0.0 if out of range
    }
    state_.measured_analog_value[i] = value;
  }

  // Validate digital IO values
  for (size_t i = 0; i < NUM_DIGITAL_IO; ++i) {
    uint64_t value = state.getDigitalIOValue(("digital_io_" + std::to_string(i + 1)).c_str());
    if (value != 0 && value != 1) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger(LOGGER_NAME),
                         "Digital IO value not boolean (0 or 1): " << value);
      value = 0; // Default to 0 if invalid
    }
    state_.measured_digital_value[i] = value;
  }

  if (!external_torque_filter_.is_initialized() || !measured_torque_filter_.is_initialized()) {
    // initialize once state_ is available
    init_filters_();
  }
  state_initialized_ = true;
};

void StateInterface::set_state_open_loop(const_fri_state_t_ref state,
                                         const_idl_joint_pos_t_ref joint_position) {
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
  std::memcpy(state_.measured_joint_position.data(), joint_position.data(),
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

  // IO functionalities
  // Validate analog IO values
  for (size_t i = 0; i < NUM_ANALOG_IO; ++i) {
    double value = state.getAnalogIOValue(("analog_io_" + std::to_string(i + 1)).c_str());
    if (value < 0.0 || value > 10.0) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger(LOGGER_NAME),
                         "Analog IO value out of range [0, 10]: " << value);
      value = 0.0; // Default to 0.0 if out of range
    }
    state_.measured_analog_value[i] = state.getAnalogIOValue(("analog_io_" + std::to_string(i + 1)).c_str());
  }

  // Validate digital IO values
  for (size_t i = 0; i < NUM_DIGITAL_IO; ++i) {
    uint64_t value = state.getDigitalIOValue(("digital_io_" + std::to_string(i + 1)).c_str());
    if (value != 0 && value != 1) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger(LOGGER_NAME),
                         "Digital IO value not boolean (0 or 1): " << value);
      value = 0; // Default to 0 if invalid
    }
    state_.measured_digital_value[i] = state.getDigitalIOValue(("digital_io_" + std::to_string(i + 1)).c_str());
  }

  if (!external_torque_filter_.is_initialized() || !measured_torque_filter_.is_initialized()) {
    // initialize once state_ is available
    init_filters_();
  }
  state_initialized_ = true;
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
}
} // namespace lbr_fri_ros2
