#include "lbr_fri_ros2/command_guard.hpp"

namespace lbr_fri_ros2 {

CommandGuard::CommandGuard(const CommandGuardParameters &command_guard_parameters)
    : parameters_(command_guard_parameters), prev_measured_joint_position_init_(false) {}

bool CommandGuard::is_valid_command(const_idl_command_t_ref lbr_command,
                                    const_idl_state_t_ref lbr_state) {
  if (!command_in_position_limits_(lbr_command, lbr_state)) {
    return false;
  }
  if (!command_in_velocity_limits_(lbr_state)) {
    return false;
  }
  if (!command_in_torque_limits_(lbr_command, lbr_state)) {
    return false;
  }
  if (!command_in_analog_limits_(lbr_command.analog_io, lbr_command.analog_value)) {
    return false;
  }
  if (!command_in_digital_limits_(lbr_command.digital_io, lbr_command.digital_value)) {
    return false;
  }
  return true;
}

void CommandGuard::log_info() const {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*** Command Guard Parameters ***");
}

bool CommandGuard::command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                               const_idl_state_t_ref /*lbr_state*/) const {
  for (std::size_t i = 0; i < lbr_command.joint_position.size(); ++i) {
    if (lbr_command.joint_position[i] < parameters_.min_positions[i] ||
        lbr_command.joint_position[i] > parameters_.max_positions[i]) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          "Position out of limits for joint: " << parameters_.joint_names[i]);
      return false;
    }
  }
  return true;
}

bool CommandGuard::command_in_velocity_limits_(const_idl_state_t_ref lbr_state) {
  const double &dt = lbr_state.sample_time;
  if (!prev_measured_joint_position_init_) {
    prev_measured_joint_position_init_ = true;
    prev_measured_joint_position_ = lbr_state.measured_joint_position;
    return true;
  }
  for (std::size_t i = 0; i < lbr_state.measured_joint_position.size(); ++i) {
    if (std::abs(prev_measured_joint_position_[i] - lbr_state.measured_joint_position[i]) / dt >
        parameters_.max_velocities[i]) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          "Velocity out of limits for joint: " << parameters_.joint_names[i]);
      return false;
    }
  }
  prev_measured_joint_position_ = lbr_state.measured_joint_position;
  return true;
}

bool CommandGuard::command_in_torque_limits_(const_idl_command_t_ref lbr_command,
                                             const_idl_state_t_ref lbr_state) const {
  for (std::size_t i = 0; i < lbr_command.torque.size(); ++i) {
    if (std::abs(lbr_command.torque[i] + lbr_state.external_torque[i]) > parameters_.max_torques[i]) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          "Torque out of limits for joint: " << parameters_.joint_names[i]);
      return false;
    }
  }
  return true;
}

bool CommandGuard::command_in_analog_limits_(const std::array<std::string, 2> &analog_io,
                                             const std::array<double, 2> &analog_values) const {
  for (std::size_t i = 0; i < analog_io.size(); ++i) {
    if (analog_values[i] < parameters_.min_analog_values[i] ||
        analog_values[i] > parameters_.max_analog_values[i]) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          "Analog IO '" << analog_io[i] << "' value out of range.");
      return false;
    }
  }
  return true;
}

bool CommandGuard::command_in_digital_limits_(const std::array<std::string, 8> &digital_io,
                                              const std::array<uint64_t, 8> &digital_values) const {
  for (std::size_t i = 0; i < digital_io.size(); ++i) {
    if (digital_values[i] != parameters_.digital_values[i]) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          "Digital IO '" << digital_io[i] << "' value mismatch.");
      return false;
    }
  }
  return true;
}

std::unique_ptr<CommandGuard>
command_guard_factory(const CommandGuardParameters &command_guard_parameters,
                      const std::string &variant) {
  if (variant == "default") {
    return std::make_unique<CommandGuard>(command_guard_parameters);
  }
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("lbr_fri_ros2::command_guard_factory"),
                      "Invalid CommandGuard variant provided.");
  throw std::runtime_error("Invalid CommandGuard variant provided.");
}

} // namespace lbr_fri_ros2
