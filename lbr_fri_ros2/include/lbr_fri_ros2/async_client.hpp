#ifndef LBR_FRI_ROS2__ASYNC_CLIENT_HPP_
#define LBR_FRI_ROS2__ASYNC_CLIENT_HPP_

#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friClientVersion.h"
#include "friLBRClient.h"

#include "lbr_fri_ros2/filters.hpp"
#include "lbr_fri_ros2/formatting.hpp"
#include "lbr_fri_ros2/interfaces/base_command.hpp"
#include "lbr_fri_ros2/interfaces/position_command.hpp"
#include "lbr_fri_ros2/interfaces/state.hpp"
#include "lbr_fri_ros2/interfaces/torque_command.hpp"
#include "lbr_fri_ros2/interfaces/wrench_command.hpp"
#include "lbr_fri_ros2/interfaces/digital_command.hpp"
#include "lbr_fri_ros2/interfaces/analog_command.hpp"

namespace lbr_fri_ros2 {
class AsyncClient : public KUKA::FRI::LBRClient {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::AsyncClient";

public:
  AsyncClient() = delete;
  AsyncClient(const KUKA::FRI::EClientCommandMode &client_command_mode,
              const PIDParameters &pid_parameters,
              const CommandGuardParameters &command_guard_parameters,
              const std::string &command_guard_variant,
              const StateInterfaceParameters &state_interface_parameters = {10.0, 10.0},
              const bool &open_loop = true);

  inline std::shared_ptr<BaseCommandInterface> get_command_interface() {
    return command_interface_ptr_;
  }
  inline std::shared_ptr<StateInterface> get_state_interface() { return state_interface_ptr_; }

  inline std::shared_ptr<DigitalIOCommandInterface> get_digital_io_command_interface() {
    return digital_io_command_interface_ptr_;
  }

  inline std::shared_ptr<AnalogIOCommandInterface> get_analog_io_command_interface() {
    return analog_io_command_interface_ptr_;
  }

  void onStateChange(KUKA::FRI::ESessionState old_state,
                     KUKA::FRI::ESessionState new_state) override;
  void monitor() override;
  void waitForCommand() override;
  void command() override;

protected:
  std::shared_ptr<BaseCommandInterface> command_interface_ptr_;
  std::shared_ptr<StateInterface> state_interface_ptr_;
  std::shared_ptr<DigitalIOCommandInterface> digital_io_command_interface_ptr_;
  std::shared_ptr<AnalogIOCommandInterface> analog_io_command_interface_ptr_;

  bool open_loop_;
};
} // namespace lbr_fri_ros2

#endif // LBR_FRI_ROS2__ASYNC_CLIENT_HPP_
