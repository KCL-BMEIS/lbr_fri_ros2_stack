#include "lbr_fri_ros2/async_client.hpp"
#include "lbr_fri_ros2/interfaces/digital_command.hpp"
#include "lbr_fri_ros2/interfaces/analog_command.hpp"
#include "lbr_fri_ros2/interfaces/position_command.hpp"
#include "lbr_fri_ros2/interfaces/torque_command.hpp"
#include "lbr_fri_ros2/interfaces/wrench_command.hpp"

namespace lbr_fri_ros2 {

AsyncClient::AsyncClient(const KUKA::FRI::EClientCommandMode &client_command_mode,
                         const PIDParameters &pid_parameters,
                         const CommandGuardParameters &command_guard_parameters,
                         const std::string &command_guard_variant,
                         const StateInterfaceParameters &state_interface_parameters,
                         const bool &open_loop)
    : open_loop_(open_loop) {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Configuring client...");

  // Create command interface based on command mode
  switch (client_command_mode) {
  case KUKA::FRI::EClientCommandMode::POSITION:
    command_interface_ptr_ = std::make_shared<PositionCommandInterface>(
        pid_parameters, command_guard_parameters, command_guard_variant);
    break;

  case KUKA::FRI::EClientCommandMode::TORQUE:
    command_interface_ptr_ = std::make_shared<TorqueCommandInterface>(
        pid_parameters, command_guard_parameters, command_guard_variant);
    break;

  case KUKA::FRI::EClientCommandMode::WRENCH:
    command_interface_ptr_ = std::make_shared<WrenchCommandInterface>(
        pid_parameters, command_guard_parameters, command_guard_variant);
    break;

  default:
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Unsupported client command mode.");
    throw std::runtime_error("Unsupported client command mode.");
  }

  // Log created command interface
  command_interface_ptr_->log_info();

  // Create state interface
  state_interface_ptr_ = std::make_shared<StateInterface>(state_interface_parameters);
  state_interface_ptr_->log_info();

  // Create IO command interfaces
  digital_io_command_interface_ptr_ = std::make_shared<DigitalIOCommandInterface>(
      command_guard_parameters, command_guard_variant);
  analog_io_command_interface_ptr_ = std::make_shared<AnalogIOCommandInterface>(
      command_guard_parameters, command_guard_variant);

  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
              (std::string("Open loop: ") + (open_loop_ ? "true" : "false")).c_str());

  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Client configured successfully.");
}

void AsyncClient::onStateChange(KUKA::FRI::ESessionState old_state,
                                KUKA::FRI::ESessionState new_state) {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
              "LBR switched state from %d to %d", old_state, new_state);

  // Initialize state and commands
  state_interface_ptr_->set_state(robotState());
  command_interface_ptr_->init_command(state_interface_ptr_->get_state());
  digital_io_command_interface_ptr_->init_command();
  analog_io_command_interface_ptr_->init_command();
}

void AsyncClient::monitor() {
  state_interface_ptr_->set_state(robotState());
}

void AsyncClient::waitForCommand() {
  state_interface_ptr_->set_state(robotState());
  command_interface_ptr_->init_command(state_interface_ptr_->get_state());
  command_interface_ptr_->buffered_command_to_fri(robotCommand(),
                                                  state_interface_ptr_->get_state());
  digital_io_command_interface_ptr_->buffered_command_to_fri(robotCommand(),
                                                             state_interface_ptr_->get_state());
  analog_io_command_interface_ptr_->buffered_command_to_fri(robotCommand(),
                                                            state_interface_ptr_->get_state());
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Wait for command successfully.");
}

void AsyncClient::command() {
  if (open_loop_) {
    state_interface_ptr_->set_state_open_loop(robotState(),
                                              command_interface_ptr_->get_command().joint_position);
  } else {
    state_interface_ptr_->set_state(robotState());
  }
  command_interface_ptr_->buffered_command_to_fri(robotCommand(),
                                                  state_interface_ptr_->get_state());
  digital_io_command_interface_ptr_->buffered_command_to_fri(robotCommand(),
                                                             state_interface_ptr_->get_state());
  analog_io_command_interface_ptr_->buffered_command_to_fri(robotCommand(),
                                                            state_interface_ptr_->get_state());
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Command successfully.");
}

} // namespace lbr_fri_ros2
