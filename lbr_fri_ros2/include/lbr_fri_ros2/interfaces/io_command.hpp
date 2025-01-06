#ifndef LBR_FRI_ROS2__INTERFACES__IO_COMMAND_HPP_
#define LBR_FRI_ROS2__INTERFACES__IO_COMMAND_HPP_

#include <vector>
#include <string>
#include "lbr_fri_idl/msg/lbr_command.hpp"
#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/command_guard.hpp"

namespace lbr_fri_ros2 {

class IOCommandInterface {
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
  // 构造函数
  IOCommandInterface(const CommandGuardParameters &command_guard_parameters,
                    const std::string &command_guard_variant = "default");

  // Pure virtual function
  virtual void buffered_command_to_fri(fri_command_t_ref command,
                                       const_idl_state_t_ref state) = 0;

  // Buffer target command
  inline void buffer_command_target(const_idl_command_t_ref command) { command_target_ = command; }

  // Initialize the command
  void init_command();

  // Accessors for the current and target commands
  inline const_idl_command_t_ref get_command() const { return command_; }
  inline const_idl_command_t_ref get_command_target() const { return command_target_; }

protected:
  idl_command_t command_, command_target_;

  // Constants
  static constexpr size_t NUM_DIGITAL_IO = 8;
  static constexpr size_t NUM_ANALOG_IO = 2;

  // IO Values and Names
  std::vector<uint64_t> digital_value;
  std::vector<std::string> digital_io;
  std::vector<double> analog_value;
  std::vector<std::string> analog_io;

  const std::vector<std::string> digital_io_names_{
      "digital_io_1", "digital_io_2", "digital_io_3", "digital_io_4",
      "digital_io_5", "digital_io_6", "digital_io_7", "digital_io_8"};
  const std::vector<std::string> analog_io_names_{
      "analog_io_1", "analog_io_2"};
};

} // namespace lbr_fri_ros2

#endif // LBR_FRI_ROS2__INTERFACES__IO_COMMAND_HPP_
