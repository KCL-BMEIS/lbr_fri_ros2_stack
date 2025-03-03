#ifndef LBR_FRI_ROS2__COMMAND_GUARD_HPP_
#define LBR_FRI_ROS2__COMMAND_GUARD_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friClientVersion.h"
#include "friLBRClient.h"
#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_command.hpp"
#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/formatting.hpp"
#include "lbr_fri_ros2/types.hpp"

namespace lbr_fri_ros2 {
struct CommandGuardParameters {
  jnt_name_array_t joint_names;                           /**< Joint names.*/
  jnt_array_t min_positions{0., 0., 0., 0., 0., 0., 0.};  /**< Minimum joint positions [rad].*/
  jnt_array_t max_positions{0., 0., 0., 0., 0., 0., 0.};  /**< Maximum joint positions [rad].*/
  jnt_array_t max_velocities{0., 0., 0., 0., 0., 0., 0.}; /**< Maximum joint velocities [rad/s].*/
  jnt_array_t max_torques{0., 0., 0., 0., 0., 0., 0.};    /**< Maximum joint torque [Nm].*/
};

class CommandGuard {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::CommandGuard";

public:
  CommandGuard() = default;
  CommandGuard(const CommandGuardParameters &command_guard_parameters);
  virtual bool is_valid_command(const_idl_command_t_ref lbr_command,
                                const_idl_state_t_ref lbr_state);

  void log_info() const;

protected:
  virtual bool command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                           const_idl_state_t_ref /*lbr_state*/) const;
  virtual bool command_in_velocity_limits_(const_idl_state_t_ref lbr_state);
  virtual bool command_in_torque_limits_(const_idl_command_t_ref lbr_command,
                                         const_idl_state_t_ref lbr_state) const;

  CommandGuardParameters parameters_;
  bool prev_measured_joint_position_init_;
  jnt_array_t prev_measured_joint_position_;
};

class SafeStopCommandGuard : public CommandGuard {
public:
  SafeStopCommandGuard(const CommandGuardParameters &command_guard_parameters)
      : CommandGuard(command_guard_parameters) {};

protected:
  virtual bool command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                           const_idl_state_t_ref lbr_state) const override;
};

std::unique_ptr<CommandGuard>
command_guard_factory(const CommandGuardParameters &command_guard_parameters,
                      const std::string &variant = "default");
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__COMMAND_GUARD_HPP_
