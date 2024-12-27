#include "lbr_fri_ros2/async_client.hpp"

namespace lbr_fri_ros2 {

// I/O 相关定义
static constexpr size_t NUM_BOOLEAN_IO = 2;  // 假设有 8 个 Boolean I/O
static constexpr size_t NUM_DIGITAL_IO = 2;  // 假设有 8 个 Digital I/O
static constexpr size_t NUM_ANALOG_IO = 2;   // 假设有 2 个 Analog I/O

AsyncClient::AsyncClient(const KUKA::FRI::EClientCommandMode &client_command_mode,
                         const PIDParameters &pid_parameters,
                         const CommandGuardParameters &command_guard_parameters,
                         const std::string &command_guard_variant,
                         const StateInterfaceParameters &state_interface_parameters,
                         const bool &open_loop)
    : open_loop_(open_loop),
      boolean_io_states_(NUM_BOOLEAN_IO, false),           // 初始化 Boolean I/O 状态
      digital_io_states_(NUM_DIGITAL_IO, 0),               // 初始化 Digital I/O 状态
      analog_io_states_(NUM_ANALOG_IO, 0.0) {              // 初始化 Analog I/O 状态
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME),
                     ColorScheme::OKBLUE << "Configuring client" << ColorScheme::ENDC);

  // 创建 command interface
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
        pid_parameters, command_guard_parameters, command_guard_variant);
    break;
  }
  case KUKA::FRI::EClientCommandMode::TORQUE:
    command_interface_ptr_ = std::make_shared<TorqueCommandInterface>(
        pid_parameters, command_guard_parameters, command_guard_variant);
    break;
  case KUKA::FRI::EClientCommandMode::WRENCH:
    command_interface_ptr_ = std::make_shared<WrenchCommandInterface>(
        pid_parameters, command_guard_parameters, command_guard_variant);
    break;
  default:
    std::string err = "Unsupported client command mode.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }
  command_interface_ptr_->log_info();

  // 创建 state interface
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

  // 初始化 command
  state_interface_ptr_->set_state(robotState());
  command_interface_ptr_->init_command(state_interface_ptr_->get_state());
}

void AsyncClient::monitor() {
  state_interface_ptr_->set_state(robotState());
}

void AsyncClient::waitForCommand() {
  KUKA::FRI::LBRClient::waitForCommand();
  state_interface_ptr_->set_state(robotState());
  command_interface_ptr_->init_command(state_interface_ptr_->get_state());
  command_interface_ptr_->buffered_command_to_fri(robotCommand(),
                                                  state_interface_ptr_->get_state());
}

void AsyncClient::command() {
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

// 新增 I/O 读取方法
bool AsyncClient::read_boolean_io(size_t index) {
  if (index >= boolean_io_states_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Invalid Boolean I/O index: %zu", index);
    return false;
  }
  return boolean_io_states_[index];
}

unsigned long long AsyncClient::read_digital_io(size_t index) {
  if (index >= digital_io_states_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Invalid Digital I/O index: %zu", index);
    return 0;
  }
  return digital_io_states_[index];
}

double AsyncClient::read_analog_io(size_t index) {
  if (index >= analog_io_states_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Invalid Analog I/O index: %zu", index);
    return 0.0;
  }
  return analog_io_states_[index];
}

// 新增 I/O 写入方法
void AsyncClient::write_boolean_io(size_t index, bool value) {
  if (index >= boolean_io_states_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Invalid Boolean I/O index: %zu", index);
    return;
  }
  boolean_io_states_[index] = value;
}

void AsyncClient::write_digital_io(size_t index, unsigned long long value) {
  if (index >= digital_io_states_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Invalid Digital I/O index: %zu", index);
    return;
  }
  digital_io_states_[index] = value;
}

void AsyncClient::write_analog_io(size_t index, double value) {
  if (index >= analog_io_states_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Invalid Analog I/O index: %zu", index);
    return;
  }
  analog_io_states_[index] = value;
}

} // namespace lbr_fri_ros2
