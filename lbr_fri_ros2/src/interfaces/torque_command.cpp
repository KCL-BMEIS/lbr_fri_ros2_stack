#include "lbr_fri_ros2/interfaces/torque_command.hpp"

namespace lbr_fri_ros2 {
TorqueCommandInterface::TorqueCommandInterface(
    const double &joint_position_tau, const CommandGuardParameters &command_guard_parameters,
    const std::string &command_guard_variant)
    : BaseCommandInterface(joint_position_tau, command_guard_parameters, command_guard_variant) {}

void TorqueCommandInterface::buffered_command_to_fri(fri_command_t_ref command,
                                                     const_idl_state_t_ref state) {
  if (state.client_command_mode != KUKA::FRI::EClientCommandMode::TORQUE) {
    std::string err = "Expected robot in '" +
                      EnumMaps::client_command_mode_map(KUKA::FRI::EClientCommandMode::TORQUE) +
                      "' command mode got '" +
                      EnumMaps::client_command_mode_map(state.client_command_mode) + "'";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }

  if (!joint_position_filter_.is_initialized()) {
    joint_position_filter_.initialize(state.sample_time);
  }

  if (!command_initialized_) {
    std::string err = "Uninitialized command.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }

  if (!std::any_of(command_target_.joint_position.cbegin(), command_target_.joint_position.cend(),
                   [](const double &v) { return std::isnan(v); }) &&
      !std::any_of(command_target_.torque.cbegin(), command_target_.torque.cend(),
                   [](const double &v) { return std::isnan(v); })) {
    // write command_target_ to command_ (with exponential smooth on joint positions), else use
    // internal command_
    joint_position_filter_.compute(command_target_.joint_position, command_.joint_position);
    command_.torque = command_target_.torque;
  }

  if (!command_guard_) {
    std::string err = "Uninitialized command guard.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }

  // validate
  if (!command_guard_->is_valid_command(command_, state)) {
    std::string err = "Invalid command.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }
  // write joint position and torque to output
  command.setJointPosition(command_.joint_position.data());
  command.setTorque(command_.torque.data());
}
} // namespace lbr_fri_ros2
