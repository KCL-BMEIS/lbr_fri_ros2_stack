#include "lbr_fri_ros2/command_guard.hpp"

namespace lbr_fri_ros2 {
CommandGuard::CommandGuard(const CommandGuardParameters &command_guard_parameters)
    : parameters_(command_guard_parameters), prev_measured_joint_position_init_(false) {};

bool CommandGuard::is_valid_command(const_idl_command_t_ref lbr_command,
                                    const_idl_state_t_ref lbr_state) {
  if (!command_in_position_limits_(lbr_command, lbr_state)) {
    return false;
  }
  if (!command_in_velocity_limits_(lbr_state)) {
    return false;
  }
  return true;
}

void CommandGuard::log_info() const {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*** Parameters:");
  for (std::size_t i = 0; i < parameters_.joint_names.size(); ++i) {
    RCLCPP_INFO(
        rclcpp::get_logger(LOGGER_NAME),
        "*   Joint %s limits: Position: [%.1f, %.1f] deg, velocity: %.1f deg/s, torque: %.1f Nm",
        parameters_.joint_names[i].c_str(), parameters_.min_positions[i] * (180. / M_PI),
        parameters_.max_positions[i] * (180. / M_PI), parameters_.max_velocities[i] * (180. / M_PI),
        parameters_.max_torques[i]);
  }
}

bool CommandGuard::command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                               const_idl_state_t_ref /*lbr_state*/) const {
  for (std::size_t i = 0; i < lbr_command.joint_position.size(); ++i) {
    if (lbr_command.joint_position[i] < parameters_.min_positions[i] ||
        lbr_command.joint_position[i] > parameters_.max_positions[i]) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          ColorScheme::ERROR << "Position not in limits for joint '"
                                             << parameters_.joint_names[i].c_str() << "'"
                                             << ColorScheme::ENDC);
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
                          ColorScheme::ERROR << "Velocity not in limits for joint '"
                                             << parameters_.joint_names[i].c_str() << "'"
                                             << ColorScheme::ENDC);
      return false;
    }
  }
  prev_measured_joint_position_ = lbr_state.measured_joint_position;
  return true;
}

bool CommandGuard::command_in_torque_limits_(const_idl_command_t_ref lbr_command,
                                             const_idl_state_t_ref lbr_state) const {
  for (std::size_t i = 0; i < lbr_command.torque.size(); ++i) {
    if (std::abs(lbr_command.torque[i] + lbr_state.external_torque[i]) >
        parameters_.max_torques[i]) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME), ColorScheme::ERROR
                                                               << "Torque not in limits for joint '"
                                                               << parameters_.joint_names[i].c_str()
                                                               << "'" << ColorScheme::ENDC);
      return false;
    }
  }
  return true;
}

bool SafeStopCommandGuard::command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                                       const_idl_state_t_ref lbr_state) const {
  for (std::size_t i = 0; i < lbr_command.joint_position.size(); ++i) {
    if (lbr_command.joint_position[i] <
            parameters_.min_positions[i] + parameters_.max_velocities[i] * lbr_state.sample_time ||
        lbr_command.joint_position[i] >
            parameters_.max_positions[i] - parameters_.max_velocities[i] * lbr_state.sample_time) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          ColorScheme::ERROR << "Position not in limits for joint '"
                                             << parameters_.joint_names[i].c_str() << "'"
                                             << ColorScheme::ENDC);
      return false;
    }
  }
  return true;
}

std::unique_ptr<CommandGuard>
command_guard_factory(const CommandGuardParameters &command_guard_parameters,
                      const std::string &variant) {
  constexpr char LOGGER_NAME[] = "lbr_fri_ros2::command_guard_factory";
  if (variant == "default") {
    return std::make_unique<CommandGuard>(command_guard_parameters);
  }
  if (variant == "safe_stop") {
    return std::make_unique<SafeStopCommandGuard>(command_guard_parameters);
  }
  std::string err = "Invalid CommandGuard variant provided.";
  RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                      ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
  throw std::runtime_error(err);
}
} // namespace lbr_fri_ros2
