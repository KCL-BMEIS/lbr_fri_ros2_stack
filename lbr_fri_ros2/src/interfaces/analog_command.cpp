#include "lbr_fri_ros2/interfaces/analog_command.hpp"
#include <stdexcept>
#include <cmath> // For std::isnan

namespace lbr_fri_ros2 {

// 构造函数实现
AnalogIOCommandInterface::AnalogIOCommandInterface(const CommandGuardParameters &command_guard_parameters,
                                                    const std::string &command_guard_variant)
    : IOCommandInterface(command_guard_parameters, command_guard_variant) {}

// `buffered_command_to_fri` 函数实现
void AnalogIOCommandInterface::buffered_command_to_fri(fri_command_t_ref command, const_idl_state_t_ref state) {
    // 检查模拟 IO 值是否无效 (NaN) 或超出范围 [0, 10]，如果有问题则恢复默认命令
    if (std::any_of(command_target_.analog_value.cbegin(), command_target_.analog_value.cend(),
                    [](const double &v) { return std::isnan(v) || v < 0.0 || v > 10.0; })) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                           "Analog IO values contain invalid entries. Restoring default command.");
        this->init_command(); // 恢复默认值
    }

    // 检查 IO 名称和值的长度是否匹配
    if (command_target_.analog_io.size() != command_target_.analog_value.size()) {
        std::string err = "Mismatch between analog IO names and values.";
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                            ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
        throw std::runtime_error(err);
    }

    // 将每个模拟 IO 的值设置到命令中
    for (size_t i = 0; i < command_target_.analog_io.size(); ++i) {
        const char *io_name = command_target_.analog_io[i].c_str(); // 获取 IO 名称
        double io_value = command_target_.analog_value[i];          // 获取 IO 值

        // 调用 FRI 接口设置 IO 值
        command.setAnalogIOValue(io_name, io_value);
    }
}

} // namespace lbr_fri_ros2
