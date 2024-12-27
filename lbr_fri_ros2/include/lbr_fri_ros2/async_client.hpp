#ifndef LBR_FRI_ROS2__ASYNC_CLIENT_HPP_
#define LBR_FRI_ROS2__ASYNC_CLIENT_HPP_

#include <cstring>
#include <memory>
#include <string>
#include <vector> // 新增支持向量
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

  void onStateChange(KUKA::FRI::ESessionState old_state,
                     KUKA::FRI::ESessionState new_state) override;
  void monitor() override;
  void waitForCommand() override;
  void command() override;

  // 新增 I/O 方法
  bool read_boolean_io(size_t index);                     // 读取单个 Boolean I/O
  unsigned long long read_digital_io(size_t index);       // 读取单个 Digital I/O
  double read_analog_io(size_t index);                    // 读取单个 Analog I/O

  void write_boolean_io(size_t index, bool value);        // 写入单个 Boolean I/O
  void write_digital_io(size_t index, unsigned long long value); // 写入单个 Digital I/O
  void write_analog_io(size_t index, double value);       // 写入单个 Analog I/O

protected:
  std::shared_ptr<BaseCommandInterface> command_interface_ptr_;
  std::shared_ptr<StateInterface> state_interface_ptr_;

  bool open_loop_;

  // 新增 I/O 状态存储（模拟硬件层的 I/O 状态）
  std::vector<bool> boolean_io_states_;                   // 存储 Boolean I/O 的状态
  std::vector<unsigned long long> digital_io_states_;     // 存储 Digital I/O 的状态
  std::vector<double> analog_io_states_;                  // 存储 Analog I/O 的状态
};
} // namespace lbr_fri_ros2

#endif // LBR_FRI_ROS2__ASYNC_CLIENT_HPP_
