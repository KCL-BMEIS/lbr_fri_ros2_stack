#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "friClientIf.h"

#include "lbr_fri_ros2/app.hpp"
#include "lbr_fri_ros2/async_client.hpp"
#include "lbr_fri_ros2/command_guard.hpp"

int main() {
  rclcpp::init(0, nullptr);

  double joint_position_tau = 0.04;
  lbr_fri_ros2::CommandGuardParameters cmd_guard_params;
  lbr_fri_ros2::StateInterfaceParameters state_interface_params;

  // #include "hardware_interface/hardware_info.hpp"
  // hardware_interface::HardwareInfo hardware_info;

  // 1. read this info!!!! from robot description

  cmd_guard_params.joint_names = {"lbr_A1", "lbr_A2", "lbr_A3", "lbr_A4",
                                  "lbr_A5", "lbr_A6", "lbr_A7"};
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

  auto client = std::make_shared<lbr_fri_ros2::AsyncClient>(
      KUKA::FRI::EClientCommandMode::TORQUE, joint_position_tau, cmd_guard_params, "default",
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
  while (rclcpp::ok()) {
    // read state
    auto state = client->get_state_interface()->get_state();
    RCLCPP_INFO(node->get_logger(), "Measured joint position: %f %f %f %f %f %f %f",
                state.measured_joint_position[0], state.measured_joint_position[1],
                state.measured_joint_position[2], state.measured_joint_position[3],
                state.measured_joint_position[4], state.measured_joint_position[5],
                state.measured_joint_position[6]);

    // spin
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  app.request_stop();
  app.close_udp_socket();

  return 0;
}
