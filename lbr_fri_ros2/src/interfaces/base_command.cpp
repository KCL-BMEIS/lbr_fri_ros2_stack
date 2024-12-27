#include "lbr_fri_ros2/interfaces/base_command.hpp"

namespace lbr_fri_ros2 {

// 构造函数
BaseCommandInterface::BaseCommandInterface(
    const PIDParameters &pid_parameters,
    const CommandGuardParameters &command_guard_parameters,
    const std::string &command_guard_variant)
    : command_guard_(command_guard_factory(command_guard_parameters, command_guard_variant)),
      joint_position_pid_(pid_parameters),
      boolean_io_0_(false), boolean_io_1_(false),
      digital_io_0_(0), digital_io_1_(0),
      analog_io_0_(0.0), analog_io_1_(0.0) {}

// 初始化命令
void BaseCommandInterface::init_command(const_idl_state_t_ref state) {
    command_target_.joint_position = state.measured_joint_position;
    command_target_.torque.fill(0.0);
    command_target_.wrench.fill(0.0);

    // 初始化 I/O 命令
    boolean_io_0_ = false;
    boolean_io_1_ = false;
    digital_io_0_ = 0;
    digital_io_1_ = 0;
    analog_io_0_ = 0.0;
    analog_io_1_ = 0.0;

    command_target_.boolean_io[0] = boolean_io_0_;
    command_target_.boolean_io[1] = boolean_io_1_;
    command_target_.digital_io[0] = digital_io_0_;
    command_target_.digital_io[1] = digital_io_1_;
    command_target_.analog_io[0] = analog_io_0_;
    command_target_.analog_io[1] = analog_io_1_;

    command_ = command_target_;
}

// 记录日志信息
void BaseCommandInterface::log_info() const {
    command_guard_->log_info();
    joint_position_pid_.log_info();

    // Log each I/O data
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()),
                "Boolean IO: boolean_io_0=%d, boolean_io_1=%d",
                boolean_io_0_, boolean_io_1_);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()),
                "Digital IO: digital_io_0=%lu, digital_io_1=%lu",
                digital_io_0_, digital_io_1_);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()),
                "Analog IO: analog_io_0=%.2f, analog_io_1=%.2f",
                analog_io_0_, analog_io_1_);
}

} // namespace lbr_fri_ros2
