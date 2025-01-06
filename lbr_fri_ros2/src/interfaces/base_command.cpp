#include "lbr_fri_ros2/interfaces/base_command.hpp"

namespace lbr_fri_ros2 {

BaseCommandInterface::BaseCommandInterface(const PIDParameters &pid_parameters,
                                           const CommandGuardParameters &command_guard_parameters,
                                           const std::string &command_guard_variant)
    : joint_position_pid_(pid_parameters) {
  command_guard_ = command_guard_factory(command_guard_parameters, command_guard_variant);
};

void BaseCommandInterface::init_command(const_idl_state_t_ref state) {
  command_target_.joint_position = state.measured_joint_position;
  command_target_.torque.fill(0.);
  command_target_.wrench.fill(0.);
  command_ = command_target_;
}

void BaseCommandInterface::log_info() const {
  command_guard_->log_info();
  joint_position_pid_.log_info();
}
} // namespace lbr_fri_ros2
