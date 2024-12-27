#include "lbr_fri_ros2/interfaces/position_command.hpp"

namespace lbr_fri_ros2 {

PositionCommandInterface::PositionCommandInterface(
    const PIDParameters &pid_parameters, const CommandGuardParameters &command_guard_parameters,
    const std::string &command_guard_variant)
    : BaseCommandInterface(pid_parameters, command_guard_parameters, command_guard_variant) {}

void PositionCommandInterface::buffered_command_to_fri(fri_command_t_ref command,
                                                       const_idl_state_t_ref state) {
#if FRI_CLIENT_VERSION_MAJOR == 1
    if (state.client_command_mode != KUKA::FRI::EClientCommandMode::POSITION) {
        std::string err = "Expected robot in '" +
                          EnumMaps::client_command_mode_map(KUKA::FRI::EClientCommandMode::POSITION) +
                          "' command mode, but got '" +
                          EnumMaps::client_command_mode_map(state.client_command_mode) + "'";
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                            ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
        throw std::runtime_error(err);
    }
#endif

#if FRI_CLIENT_VERSION_MAJOR >= 2
    if (state.client_command_mode != KUKA::FRI::EClientCommandMode::JOINT_POSITION) {
        std::string err =
            "Expected robot in " + EnumMaps::client_command_mode_map(KUKA::FRI::EClientCommandMode::JOINT_POSITION) +
            " command mode.";
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME()), err.c_str());
        throw std::runtime_error(err);
    }
#endif

    // 检查关节位置是否有非法值（如 NaN）
    if (std::any_of(command_target_.joint_position.cbegin(), command_target_.joint_position.cend(),
                    [](const double &v) { return std::isnan(v); })) {
        this->init_command(state);  // 初始化命令
    }

    // 检查命令保护是否已初始化
    if (!command_guard_) {
        std::string err = "Uninitialized command guard.";
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()), ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
        throw std::runtime_error(err);
    }

    // 使用 PID 控制器计算关节位置目标
    joint_position_pid_.compute(command_target_.joint_position, state.measured_joint_position,
                                std::chrono::nanoseconds(static_cast<int64_t>(state.sample_time * 1.e9)),
                                command_.joint_position);

    // 验证命令的有效性
    if (!command_guard_->is_valid_command(command_, state)) {
        std::string err = "Invalid command.";
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()), ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
        throw std::runtime_error(err);
    }

    // 写入关节位置
    command.setJointPosition(command_.joint_position.data());
    
    // 写入布尔量 I/O
    command.setBooleanIOValue("boolean_io_0", command_.boolean_io[0]);
    command.setBooleanIOValue("boolean_io_1", command_.boolean_io[1]);

    // 写入数字量 I/O
    command.setDigitalIOValue("digital_io_0", command_.digital_io[0]);
    command.setDigitalIOValue("digital_io_1", command_.digital_io[1]);

    // 写入模拟量 I/O
    command.setAnalogIOValue("analog_io_0", command_.analog_io[0]);
    command.setAnalogIOValue("analog_io_1", command_.analog_io[1]);
}

} // namespace lbr_fri_ros2
