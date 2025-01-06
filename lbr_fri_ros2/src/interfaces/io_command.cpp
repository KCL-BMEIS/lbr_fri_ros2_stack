#include "lbr_fri_ros2/interfaces/io_command.hpp"
#include <algorithm>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace lbr_fri_ros2 {

IOCommandInterface::IOCommandInterface(const CommandGuardParameters &command_guard_parameters,
                                           const std::string &command_guard_variant) {
    // 初始化数字 IO 和模拟 IO 的大小
    digital_value.resize(NUM_DIGITAL_IO, 0);
    digital_io = digital_io_names_;

    analog_value.resize(NUM_ANALOG_IO, 0.0);
    analog_io = analog_io_names_;
}

void IOCommandInterface::init_command() {
  // Reset digital IO values to 0
  if (digital_value.size() != NUM_DIGITAL_IO) {
    digital_value.resize(NUM_DIGITAL_IO, 0);
  }
  std::fill(digital_value.begin(), digital_value.end(), 0);

  // Reset analog IO values to 0.0
  if (analog_value.size() != NUM_ANALOG_IO) {
    analog_value.resize(NUM_ANALOG_IO, 0.0);
  }
  std::fill(analog_value.begin(), analog_value.end(), 0.0);

  // Reset digital and analog IO names to defaults
  if (digital_io.size() != NUM_DIGITAL_IO) {
    digital_io = digital_io_names_;
  }
  if (analog_io.size() != NUM_ANALOG_IO) {
    analog_io = analog_io_names_;
  }

  // Optional: Log the initialization
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()),
              "Command initialized: all digital IO set to 0, all analog IO set to 0.0.");
}

} // namespace lbr_fri_ros2
