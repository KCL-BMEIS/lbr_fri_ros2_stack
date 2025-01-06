#ifndef LBR_FRI_ROS2__INTERACES__COMMAND_HPP_
#define LBR_FRI_ROS2__INTERACES__COMMAND_HPP_

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friClientVersion.h"
#include "friLBRClient.h"

#include "lbr_fri_idl/msg/lbr_command.hpp"
#include "lbr_fri_ros2/command_guard.hpp"
#include "lbr_fri_ros2/filters.hpp"
#include "lbr_fri_ros2/formatting.hpp"

namespace lbr_fri_ros2 {
class BaseCommandInterface {
protected:
  virtual std::string LOGGER_NAME() const = 0;

  // ROS IDL types
  using idl_command_t = lbr_fri_idl::msg::LBRCommand;
  using const_idl_command_t_ref = const idl_command_t &;
  using idl_state_t = lbr_fri_idl::msg::LBRState;
  using const_idl_state_t_ref = const idl_state_t &;

  // FRI types
  using fri_command_t = KUKA::FRI::LBRCommand;
  using fri_command_t_ref = fri_command_t &;

public:
  BaseCommandInterface() = delete;
  BaseCommandInterface(const PIDParameters &pid_parameters,
                       const CommandGuardParameters &command_guard_parameters,
                       const std::string &command_guard_variant = "default");

  virtual void buffered_command_to_fri(fri_command_t_ref command, const_idl_state_t_ref state) = 0;
  inline void buffer_command_target(const_idl_command_t_ref command) { command_target_ = command; }
  void init_command(const_idl_state_t_ref state);

  inline const_idl_command_t_ref get_command() const { return command_; }
  inline const_idl_command_t_ref get_command_target() const { return command_target_; }

  void log_info() const;

protected:
  std::unique_ptr<CommandGuard> command_guard_;
  JointPIDArray joint_position_pid_;
  idl_command_t command_, command_target_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__INTERACES__COMMAND_HPP_
