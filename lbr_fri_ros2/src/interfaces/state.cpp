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
  if (state.getSessionState() == KUKA::FRI::ESessionState::COMMANDING_WAIT ||
      state.getSessionState() == KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
    std::memcpy(state_.ipo_joint_position.data(), state.getIpoJointPosition(),
                sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
  }
  std::memcpy(state_.measured_joint_position.data(), state.getMeasuredJointPosition(),
              sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
  state_.operation_mode = state.getOperationMode();
  state_.overlay_type = state.getOverlayType();
  state_.safety_state = state.getSafetyState();
  state_.sample_time = state.getSampleTime();
  state_.session_state = state.getSessionState();
  state_.time_stamp_nano_sec = state.getTimestampNanoSec();
  state_.time_stamp_sec = state.getTimestampSec();
  state_.tracking_performance = state.getTrackingPerformance();

  if (!external_torque_filter_.is_initialized() || !measured_torque_filter_.is_initialized()) {
    // initialize state_.sample_time is available
    init_filters_();
  }

  // only compute after state_.sample_time is available
  external_torque_filter_.compute(state.getExternalTorque(), state_.external_torque);
  measured_torque_filter_.compute(state.getMeasuredTorque(), state_.measured_torque);

  state_initialized_ = true;
};

void StateInterface::set_state_open_loop(const_fri_state_t_ref state,
                                         const_jnt_array_t_ref joint_position) {
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
  if (state.getSessionState() == KUKA::FRI::ESessionState::COMMANDING_WAIT ||
      state.getSessionState() == KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
    std::memcpy(state_.ipo_joint_position.data(), state.getIpoJointPosition(),
                sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
  }
  std::memcpy(state_.measured_joint_position.data(), joint_position.data(),
              sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
  state_.operation_mode = state.getOperationMode();
  state_.overlay_type = state.getOverlayType();
  state_.safety_state = state.getSafetyState();
  state_.sample_time = state.getSampleTime();
  state_.session_state = state.getSessionState();
  state_.time_stamp_nano_sec = state.getTimestampNanoSec();
  state_.time_stamp_sec = state.getTimestampSec();
  state_.tracking_performance = state.getTrackingPerformance();

  if (!external_torque_filter_.is_initialized() || !measured_torque_filter_.is_initialized()) {
    // initialize state_.sample_time is available
    init_filters_();
  }

  // only compute after state_.sample_time is available
  external_torque_filter_.compute(state.getExternalTorque(), state_.external_torque);
  measured_torque_filter_.compute(state.getMeasuredTorque(), state_.measured_torque);

  state_initialized_ = true;
}

void StateInterface::init_filters_() {
  external_torque_filter_.initialize(parameters_.external_torque_tau, state_.sample_time);
  measured_torque_filter_.initialize(parameters_.measured_torque_tau, state_.sample_time);
}

void StateInterface::log_info() const {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*** Parameters:");
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   external_torque_tau: %.5f s",
              parameters_.external_torque_tau);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   measured_torque_tau: %.5f s",
              parameters_.measured_torque_tau);
}
} // namespace lbr_fri_ros2
