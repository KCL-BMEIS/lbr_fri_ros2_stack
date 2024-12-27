#ifndef LBR_FRI_ROS2__INTERFACES__BASE_COMMAND_HPP_
#define LBR_FRI_ROS2__INTERFACES__BASE_COMMAND_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friClientVersion.h"
#include "friLBRClient.h"

#include "lbr_fri_idl/msg/lbr_command.hpp"
#include "lbr_fri_ros2/command_guard.hpp"
#include "lbr_fri_ros2/filters.hpp"

namespace lbr_fri_ros2 {

class BaseCommandInterface {
protected:
    virtual std::string LOGGER_NAME() const = 0;

    using idl_command_t = lbr_fri_idl::msg::LBRCommand;
    using const_idl_command_t_ref = const idl_command_t &;
    using idl_state_t = lbr_fri_idl::msg::LBRState;
    using const_idl_state_t_ref = const idl_state_t &;

    using fri_command_t = KUKA::FRI::LBRCommand;
    using fri_command_t_ref = fri_command_t &;

public:
    BaseCommandInterface() = delete;
    BaseCommandInterface(const PIDParameters &pid_parameters,
                         const CommandGuardParameters &command_guard_parameters,
                         const std::string &command_guard_variant = "default");

    virtual void buffered_command_to_fri(fri_command_t_ref command, const_idl_state_t_ref state) = 0;

    inline void buffer_command_target(const_idl_command_t_ref command) {
        command_target_ = command;

        // Assign I/O data
        boolean_io_0_ = command.boolean_io[0];
        boolean_io_1_ = command.boolean_io[1];
        digital_io_0_ = command.digital_io[0];
        digital_io_1_ = command.digital_io[1];
        analog_io_0_ = command.analog_io[0];
        analog_io_1_ = command.analog_io[1];
    }

    void init_command(const_idl_state_t_ref state);

    inline const_idl_command_t_ref get_command() const { return command_; }
    inline const_idl_command_t_ref get_command_target() const { return command_target_; }

    void log_info() const;

protected:
    std::unique_ptr<CommandGuard> command_guard_;
    JointPIDArray joint_position_pid_;
    idl_command_t command_, command_target_;

    // Separate variables for each specific I/O
    bool boolean_io_0_;
    bool boolean_io_1_;
    uint64_t digital_io_0_;
    uint64_t digital_io_1_;
    double analog_io_0_;
    double analog_io_1_;
};

} // namespace lbr_fri_ros2

#endif // LBR_FRI_ROS2__INTERFACES__BASE_COMMAND_HPP_
