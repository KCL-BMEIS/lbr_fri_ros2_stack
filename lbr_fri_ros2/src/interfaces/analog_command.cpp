#include "lbr_fri_ros2/interfaces/analog_command.hpp"

namespace lbr_fri_ros2 {

AnalogCommandInterface::AnalogCommandInterface(
    const double &joint_position_tau, 
    const CommandGuardParameters &command_guard_parameters,
    const std::string &command_guard_variant)
    : BaseCommandInterface(joint_position_tau, command_guard_parameters, command_guard_variant) {}

void AnalogCommandInterface::buffered_command_to_fri(fri_command_t_ref command,
                                                     const_idl_state_t_ref state) {
  // 检查命令是否初始化
  if (!command_initialized_) {
    std::string err = "Uninitialized command.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()), err);
    throw std::runtime_error(err);
  }

  // 检查 command guard
  if (!command_guard_) {
    std::string err = "Uninitialized command guard.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()), err);
    throw std::runtime_error(err);
  }

  // 验证 command 是否有效
  if (!command_guard_->is_valid_command(command_, state)) {
    std::string err = "Invalid command.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()), err);
    throw std::runtime_error(err);
  }

  // 将每个模拟 IO 的值写入到命令中
  // 假设 analog_io_names.size() == analog_io_values.size()
  for (size_t i = 0; i < command_target_.analog_io_names.size(); ++i) {
    const char *io_name = command_target_.analog_io_names[i].c_str();  // IO 名称
    double io_value     = command_target_.analog_io_values[i];        // IO 值

    // 调用 FRI 接口设置 IO 值
    command.setAnalogIOValue(io_name, io_value);
  }
}

} // namespace lbr_fri_ros2
