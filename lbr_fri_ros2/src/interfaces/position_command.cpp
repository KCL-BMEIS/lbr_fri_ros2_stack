#include "lbr_fri_ros2/interfaces/position_command.hpp"

namespace lbr_fri_ros2 {
PositionCommandInterface::PositionCommandInterface(
    const double &joint_position_tau, const CommandGuardParameters &command_guard_parameters,
    const std::string &command_guard_variant)
    : BaseCommandInterface(joint_position_tau, command_guard_parameters, command_guard_variant) {}

void PositionCommandInterface::buffered_command_to_fri(fri_command_t_ref command,
                                                       const_idl_state_t_ref state) {
#if FRI_CLIENT_VERSION_MAJOR == 1
  if (state.client_command_mode != KUKA::FRI::EClientCommandMode::POSITION) {
    std::string err = "Expected robot in '" +
                      EnumMaps::client_command_mode_map(KUKA::FRI::EClientCommandMode::POSITION) +
                      "' command mode got '" +
                      EnumMaps::client_command_mode_map(state.client_command_mode) + "'";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }
#endif
#if FRI_CLIENT_VERSION_MAJOR >= 2
  if (state.client_command_mode != KUKA::FRI::EClientCommandMode::JOINT_POSITION) {
    std::string err =
        "Expected robot in " +
        EnumMaps::client_command_mode_map(KUKA::FRI::EClientCommandMode::JOINT_POSITION) +
        " command mode.";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME()), err.c_str());
    throw std::runtime_error(err);
  }
#endif
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
                   [](const double &v) { return std::isnan(v); })) {
    // write command_target_ to command_ (with exponential smooth on joint positions), else use
    // internal command_
    joint_position_filter_.compute(command_target_.joint_position, command_.joint_position);
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

  // write joint position to output
  command.setJointPosition(command_.joint_position.data());
}
} // namespace lbr_fri_ros2
