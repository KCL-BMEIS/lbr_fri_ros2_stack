#ifndef LBR_FRI_ROS2__COMMAND_GUARD_HPP_
#define LBR_FRI_ROS2__COMMAND_GUARD_HPP_

#include <array>
#include <string>
#include <vector>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friClientVersion.h"
#include "friLBRClient.h"
#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_command.hpp"
#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/formatting.hpp"

namespace lbr_fri_ros2 {

struct CommandGuardParameters {
  using jnt_array_t = std::array<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;
  using jnt_name_array_t = std::array<std::string, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;

  jnt_name_array_t joint_names;
  jnt_array_t min_positions{0., 0., 0., 0., 0., 0., 0.};
  jnt_array_t max_positions{0., 0., 0., 0., 0., 0., 0.};
  jnt_array_t max_velocities{0., 0., 0., 0., 0., 0., 0.};
  jnt_array_t max_torques{0., 0., 0., 0., 0., 0., 0.};

  // Analog IO configuration
  std::vector<std::string> analog_io_names;
  std::vector<double> min_analog_values;
  std::vector<double> max_analog_values;

  // Digital IO configuration
  std::vector<std::string> digital_io_names;
  std::vector<uint64_t> digital_values;
};

class CommandGuard {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::CommandGuard";

  using idl_command_t = lbr_fri_idl::msg::LBRCommand;
  using const_idl_command_t_ref = const idl_command_t &;
  using idl_state_t = lbr_fri_idl::msg::LBRState;
  using const_idl_state_t_ref = const idl_state_t &;

public:
  CommandGuard() = default;
  CommandGuard(const CommandGuardParameters &command_guard_parameters);

  virtual bool is_valid_command(const_idl_command_t_ref lbr_command,
                                const_idl_state_t_ref lbr_state);

  void log_info() const;

protected:
  virtual bool command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                           const_idl_state_t_ref lbr_state) const;
  virtual bool command_in_velocity_limits_(const_idl_state_t_ref lbr_state);
  virtual bool command_in_torque_limits_(const_idl_command_t_ref lbr_command,
                                         const_idl_state_t_ref lbr_state) const;
  virtual bool command_in_analog_limits_(const std::array<std::string, 2> &analog_io,
                                         const std::array<double, 2> &analog_values) const;
  virtual bool command_in_digital_limits_(const std::array<std::string, 8> &digital_io,
                                          const std::array<uint64_t, 8> &digital_values) const;

  CommandGuardParameters parameters_;
  bool prev_measured_joint_position_init_;
  std::array<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS> prev_measured_joint_position_;
};

std::unique_ptr<CommandGuard>
command_guard_factory(const CommandGuardParameters &command_guard_parameters,
                      const std::string &variant = "default");

} // namespace lbr_fri_ros2

#endif // LBR_FRI_ROS2__COMMAND_GUARD_HPP_
