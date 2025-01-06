#include "lbr_fri_ros2/interfaces/digital_command.hpp"
#include <stdexcept>
#include <cmath>

namespace lbr_fri_ros2 {
DigitalIOCommandInterface::DigitalIOCommandInterface(const CommandGuardParameters &command_guard_parameters,
                                                    const std::string &command_guard_variant)
    : IOCommandInterface(command_guard_parameters, command_guard_variant){}

void DigitalIOCommandInterface::buffered_command_to_fri(fri_command_t_ref command, const_idl_state_t_ref state){
    // 检查是否有无效值（比如 NaN），如果有则恢复默认命令
    if (std::any_of(command_target_.digital_value.cbegin(), command_target_.digital_value.cend(),
                    [](const uint64_t &v) { return v != 0 && v != 1; })) { // 数字 IO 值应该是 0 或 1
        this->init_command();
    }

    // 检查 IO 名称和值的长度是否匹配
    if (command_target_.digital_io.size() != command_target_.digital_value.size()) {
        std::string err = "Mismatch between digital IO names and values.";
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                            ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
        throw std::runtime_error(err);
    }

    // 将每个数字 IO 的名称和值设置到命令中
    for (size_t i = 0; i < command_target_.digital_io.size(); ++i) {
        const char *io_name = command_target_.digital_io[i].c_str(); // 获取 IO 名称
        bool io_value = static_cast<bool>(command_target_.digital_value[i]); // 转换值为布尔类型

        // 调用 FRI 接口设置 IO 值
        command.setDigitalIOValue(io_name, io_value);
    }
}
} // namespace lbr_fri_ros2