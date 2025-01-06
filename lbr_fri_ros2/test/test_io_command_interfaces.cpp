#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "friClientIf.h"

#include "lbr_fri_ros2/app.hpp"
#include "lbr_fri_ros2/async_client.hpp"
#include "lbr_fri_ros2/command_guard.hpp"
#include "lbr_fri_ros2/filters.hpp"

int main() {
  rclcpp::init(0, nullptr);

  lbr_fri_ros2::PIDParameters pid_params;
  lbr_fri_ros2::CommandGuardParameters cmd_guard_params;
  lbr_fri_ros2::StateInterfaceParameters state_interface_params;

  // #include "hardware_interface/hardware_info.hpp"
  // hardware_interface::HardwareInfo hardware_info;

  // 1. read this info!!!! from robot description

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
  cmd_guard_params.max_torques = {
      200., 200., 200., 200., 200., 200., 200.,
  };

  auto client = std::make_shared<lbr_fri_ros2::AsyncClient>(KUKA::FRI::EClientCommandMode::POSITION,
                                                            pid_params, cmd_guard_params, "default",
                                                            state_interface_params, true);
  lbr_fri_ros2::App app(client);

  app.open_udp_socket();
  app.run_async();

  // 2. build some sort of dummy udp connection to emulate the robot

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
  while (rclcpp::ok()) {
    // read state
    if (!state_initialized) {
      state = client->get_state_interface()->get_state();
      state_initialized = true;
    }
    RCLCPP_INFO(node->get_logger(), "Measured digital inputs: %li %li %li %li %li %li %li %li ",
                state.measured_digital_value[0], state.measured_digital_value[1],
                state.measured_digital_value[2], state.measured_digital_value[3],
                state.measured_digital_value[4], state.measured_digital_value[5],
                state.measured_digital_value[6], state.measured_digital_value[7]);

    // set command
    command.digital_value.fill(0);
    command.digital_value[0] = 1;
    command.digital_value[2] = 1;
    client->get_command_interface()->buffer_command_target(command);

    // 3. test the interfaced for safe interaction
    auto command_target = client->get_command_interface()->get_command_target();
    command_target.digital_value[7] = 1; // must not change internal value!
    command_target = client->get_command_interface()->get_command_target();

    RCLCPP_INFO(node->get_logger(), "Command digital io values: %li %li %li %li %li %li %li %li",
                command_target.digital_value[0], command.digital_value[1],
                command_target.digital_value[2], command.digital_value[3],
                command_target.digital_value[4], command.digital_value[5],
                command_target.digital_value[6], command.digital_value[7]);

    // spin
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  app.request_stop();
  app.close_udp_socket();

  return 0;
}