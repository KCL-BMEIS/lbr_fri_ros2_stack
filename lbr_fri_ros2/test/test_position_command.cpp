#include <cmath>
#include <memory>
#include <array>
#include <vector>
#include <cstdint>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "friClientIf.h"

#include "lbr_fri_ros2/app.hpp"
#include "lbr_fri_ros2/async_client.hpp"
#include "lbr_fri_ros2/command_guard.hpp"
#include "lbr_fri_ros2/filters.hpp"
#include "lbr_fri_ros2/interfaces/position_command.hpp"

// 辅助函数：将 std::vector 转换为 std::array
template <typename T, size_t N>
std::array<T, N> vector_to_array(const std::vector<T>& vec) {
  if (vec.size() != N) {
    throw std::runtime_error("Input vector size does not match array size.");
  }
  std::array<T, N> arr;
  std::copy_n(vec.begin(), N, arr.begin());
  return arr;
}

int main() {
  rclcpp::init(0, nullptr);

  lbr_fri_ros2::PIDParameters pid_params;
  lbr_fri_ros2::CommandGuardParameters cmd_guard_params;
  lbr_fri_ros2::StateInterfaceParameters state_interface_params;

  pid_params.p = 1.0;

  cmd_guard_params.joint_names = {"A1", "A2", "A3", "A4", "A5", "A6", "A7"};
  cmd_guard_params.max_positions = {170. * M_PI / 180., 120. * M_PI / 180., 170. * M_PI / 180.,
                                     120. * M_PI / 180., 170. * M_PI / 180., 120. * M_PI / 180.,
                                     175. * M_PI / 180.};
  cmd_guard_params.min_positions = {-170. * M_PI / 180., -120. * M_PI / 180., -170. * M_PI / 180.,
                                     -120. * M_PI / 180., -170. * M_PI / 180., -120. * M_PI / 180.,
                                     -175. * M_PI / 180.};
  cmd_guard_params.max_velocities = {98. * M_PI / 180.,  98. * M_PI / 180.,  100. * M_PI / 180.,
                                      130. * M_PI / 180., 140. * M_PI / 180., 180. * M_PI / 180.,
                                      180. * M_PI / 180.};
  cmd_guard_params.max_torques = {200., 200., 200., 200., 200., 200., 200.};

  auto client = std::make_shared<lbr_fri_ros2::AsyncClient>(KUKA::FRI::EClientCommandMode::POSITION,
                                                            pid_params, cmd_guard_params, "default",
                                                            state_interface_params, true);
  lbr_fri_ros2::App app(client);

  app.open_udp_socket();
  app.run_async();

  auto node = std::make_shared<rclcpp::Node>("test_async_client");

  while (!client->get_state_interface()->is_initialized()) {
    if (!rclcpp::ok()) {
      app.request_stop();
      app.close_udp_socket();
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for state interface initialization");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  lbr_fri_idl::msg::LBRCommand command;
  lbr_fri_idl::msg::LBRState state;
  bool state_initialized = false;

  // 使用 std::array 定义 I/O 数据
  std::array<bool, 2> boolean_io_data = {true, false};
  std::array<uint64_t, 2> digital_io_data = {1, 0};
  std::array<double, 2> analog_io_data = {0.5, 0.25};

  while (rclcpp::ok()) {
    if (!state_initialized) {
      state = client->get_state_interface()->get_state();
      state_initialized = true;
    }

    // 填充命令的 I/O 数据
    command.boolean_io = boolean_io_data;
    command.digital_io = digital_io_data;
    command.analog_io = analog_io_data;

    command.joint_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0};

    // 调试信息
    RCLCPP_INFO(node->get_logger(), "Sending command with I/O data:");

    client->get_command_interface()->buffer_command_target(command);

    KUKA::FRI::LBRCommand fri_command;

    client->get_command_interface()->buffered_command_to_fri(fri_command, state);

    auto processed_command = client->get_command_interface()->get_command();

    // 验证 I/O 数据
    if (processed_command.boolean_io != boolean_io_data ||
        processed_command.digital_io != digital_io_data ||
        processed_command.analog_io != analog_io_data) {
      RCLCPP_ERROR(node->get_logger(), "I/O data processing failed!");
      return -1;
    } else {
      RCLCPP_INFO(node->get_logger(), "I/O data processing successful!");
    }

    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  app.request_stop();
  app.close_udp_socket();

  return 0;
}
