#include "lbr_fri_ros2/async_client.hpp"

namespace lbr_fri_ros2 {
AsyncClient::AsyncClient(const KUKA::FRI::EClientCommandMode &client_command_mode,
                         const double &joint_position_tau,
                         const CommandGuardParameters &command_guard_parameters,
                         const std::string &command_guard_variant,
                         const StateInterfaceParameters &state_interface_parameters,
                         const bool &open_loop)
    : open_loop_(open_loop) {
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME),
                     ColorScheme::OKBLUE << "Configuring client" << ColorScheme::ENDC);

  // create command interface
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME),
                     "Client command mode: '"
                         << EnumMaps::client_command_mode_map(client_command_mode).c_str() << "'");
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME),
                     "Command guard variant '" << command_guard_variant.c_str() << "'");
  switch (client_command_mode) {
#if FRI_CLIENT_VERSION_MAJOR == 1
  case KUKA::FRI::EClientCommandMode::POSITION:
#endif
#if FRI_CLIENT_VERSION_MAJOR >= 2
  case KUKA::FRI::EClientCommandMode::JOINT_POSITION:
#endif
  {
    command_interface_ptr_ = std::make_shared<PositionCommandInterface>(
        joint_position_tau, command_guard_parameters, command_guard_variant);
    break;
  }
  case KUKA::FRI::EClientCommandMode::TORQUE:
    command_interface_ptr_ = std::make_shared<TorqueCommandInterface>(
        joint_position_tau, command_guard_parameters, command_guard_variant);
    break;
  case KUKA::FRI::EClientCommandMode::WRENCH:
    command_interface_ptr_ = std::make_shared<WrenchCommandInterface>(
        joint_position_tau, command_guard_parameters, command_guard_variant);
    break;
  default:
    std::string err = "Unsupported client command mode.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }
  command_interface_ptr_->log_info();

  // create state interface
  state_interface_ptr_ = std::make_shared<StateInterface>(state_interface_parameters);
  state_interface_ptr_->log_info();
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME),
                     "Open loop '" << (open_loop_ ? "true" : "false") << "'");
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME),
                     ColorScheme::OKGREEN << "Client configured" << ColorScheme::ENDC);
}

void AsyncClient::onStateChange(KUKA::FRI::ESessionState old_state,
                                KUKA::FRI::ESessionState new_state) {
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME),
                     "LBR switched from '"
                         << ColorScheme::OKBLUE << ColorScheme::BOLD
                         << EnumMaps::session_state_map(old_state).c_str() << ColorScheme::ENDC
                         << "' to '" << ColorScheme::OKGREEN << ColorScheme::BOLD
                         << EnumMaps::session_state_map(new_state).c_str() << ColorScheme::ENDC
                         << "'");

  // initialize command
  state_interface_ptr_->set_state(robotState());
  command_interface_ptr_->init_command(state_interface_ptr_->get_state());
}

void AsyncClient::monitor() { state_interface_ptr_->set_state(robotState()); };

void AsyncClient::waitForCommand() {
  KUKA::FRI::LBRClient::waitForCommand();
  state_interface_ptr_->set_state(robotState());
  command_interface_ptr_->init_command(state_interface_ptr_->get_state());
  command_interface_ptr_->buffered_command_to_fri(robotCommand(),
                                                  state_interface_ptr_->get_state());
}

void AsyncClient::command() {
  // if robot is in impedance or Cartesian impedance control mode, override open_loop_ to false
  // also refer to https://github.com/lbr-stack/lbr_fri_ros2_stack/issues/226
  auto control_mode = robotState().getControlMode();
  if (control_mode == KUKA::FRI::EControlMode::JOINT_IMP_CONTROL_MODE ||
      control_mode == KUKA::FRI::EControlMode::CART_IMP_CONTROL_MODE) {
    if (open_loop_) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME),
                         "Overriding open loop from '"
                             << (open_loop_ ? "true" : "false")
                             << "' to 'false' since LBR is in control mode '"
                             << EnumMaps::control_mode_map(control_mode)
                             << "'. Please refer to "
                                "[https://github.com/lbr-stack/lbr_fri_ros2_stack/issues/226].");
      open_loop_ = false;
    }
  }

  if (open_loop_) {
    state_interface_ptr_->set_state_open_loop(robotState(),
                                              command_interface_ptr_->get_command().joint_position);
  } else {
    state_interface_ptr_->set_state(robotState());
  }
  command_interface_ptr_->buffered_command_to_fri(
      robotCommand(),
      state_interface_ptr_->get_state()); // current state accessed via state interface (allows for
                                          // open loop and is statically sized)
}
} // namespace lbr_fri_ros2
