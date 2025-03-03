#include <gtest/gtest.h>
#include <random>
#include <stdexcept>

#include "friClientApplication.h"
#include "friClientIf.h"
#include "friClientVersion.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"

#include "lbr_fri_idl/msg/lbr_command.hpp"
#include "lbr_fri_ros2/interfaces/base_command.hpp"
#include "lbr_fri_ros2/interfaces/position_command.hpp"
#include "lbr_fri_ros2/interfaces/state.hpp"
#include "lbr_fri_ros2/interfaces/torque_command.hpp"
#include "lbr_fri_ros2/interfaces/wrench_command.hpp"

class TestCommandInterfaces : public ::testing::Test {
public:
  TestCommandInterfaces() : random_engine_(std::random_device{}()) {
    cmd_guard_params_ = lbr_fri_ros2::CommandGuardParameters();
    state_interface_params_ = lbr_fri_ros2::StateInterfaceParameters();

    state_interface_ = std::make_shared<lbr_fri_ros2::StateInterface>(state_interface_params_);

    // pseudo application
    udp_connection_ = std::make_unique<KUKA::FRI::UdpConnection>();
    lbr_client_ = std::make_shared<KUKA::FRI::LBRClient>();
    pseudo_application_ =
        std::make_unique<KUKA::FRI::ClientApplication>(*udp_connection_, *lbr_client_);
  }

  void set_up(const KUKA::FRI::EClientCommandMode &client_command_mode) {
    switch (client_command_mode) {
#if FRI_CLIENT_VERSION_MAJOR == 1
    case KUKA::FRI::EClientCommandMode::POSITION:
#endif
#if FRI_CLIENT_VERSION_MAJOR >= 2
    case KUKA::FRI::EClientCommandMode::JOINT_POSITION:
#endif
    {
      command_interface_ = std::make_shared<lbr_fri_ros2::PositionCommandInterface>(
          joint_position_tau_, cmd_guard_params_);
      break;
    }
    case KUKA::FRI::EClientCommandMode::TORQUE:
      command_interface_ = std::make_shared<lbr_fri_ros2::TorqueCommandInterface>(
          joint_position_tau_, cmd_guard_params_);
      break;
    case KUKA::FRI::EClientCommandMode::WRENCH:
      command_interface_ = std::make_shared<lbr_fri_ros2::WrenchCommandInterface>(
          joint_position_tau_, cmd_guard_params_);
      break;
    default:
      throw std::runtime_error("Unsupported client command mode.");
    }
  }

  lbr_fri_idl::msg::LBRCommand random_idl_command() {
    lbr_fri_idl::msg::LBRCommand idl_command;
    for (auto &joint_position : idl_command.joint_position) {
      joint_position = uniform_real_dist_(random_engine_);
    }
    for (auto &torque : idl_command.torque) {
      torque = uniform_real_dist_(random_engine_);
    }
    for (auto &wrench : idl_command.wrench) {
      wrench = uniform_real_dist_(random_engine_);
    }
    return idl_command;
  }

protected:
  void test_simple() {
    // test read only
    auto idl_command = command_interface_->get_command();
    auto idl_command_target = command_interface_->get_command_target();

    // modify and expect unchanged
    idl_command.joint_position[0] += 1.0;
    idl_command_target.joint_position[0] += 1.0;

    EXPECT_FALSE(idl_command.joint_position[0] ==
                 command_interface_->get_command().joint_position[0]);
    EXPECT_FALSE(idl_command_target.joint_position[0] ==
                 command_interface_->get_command_target().joint_position[0]);

    // assume user sets random command target
    idl_command_target = random_idl_command();
    command_interface_->buffer_command_target(idl_command_target);

    // initialize commands to state
    state_interface_->set_state(lbr_client_->robotState());
    command_interface_->init_command(
        state_interface_->get_state()); // get state from state interface

    // expect command target is state now (zero initialized here)
    for (std::size_t i = 0; i < idl_command_target.joint_position.size(); ++i) {
      // expect joint position is initialized with current robot state
      EXPECT_DOUBLE_EQ(command_interface_->get_command().joint_position[i],
                       lbr_client_->robotState().getMeasuredJointPosition()[i]);
      EXPECT_DOUBLE_EQ(command_interface_->get_command_target().joint_position[i],
                       lbr_client_->robotState().getMeasuredJointPosition()[i]);

      // expect torques are zero
      EXPECT_DOUBLE_EQ(command_interface_->get_command().torque[i], 0.0);
      EXPECT_DOUBLE_EQ(command_interface_->get_command_target().torque[i], 0.0);
    }
    for (std::size_t i = 0; i < idl_command_target.wrench.size(); ++i) {
      // expect wrenches are zero
      EXPECT_DOUBLE_EQ(command_interface_->get_command().wrench[i], 0.0);
      EXPECT_DOUBLE_EQ(command_interface_->get_command_target().wrench[i], 0.0);
    }

    // buffer a random command and expect invalid client command mode
    bool invalid_mode_triggered = false;
    try {
      idl_command_target = random_idl_command();
      command_interface_->buffer_command_target(idl_command_target);
      command_interface_->buffered_command_to_fri(lbr_client_->robotCommand(),
                                                  state_interface_->get_state());
    } catch (const std::exception &) {
      invalid_mode_triggered = true;
    }
    EXPECT_TRUE(invalid_mode_triggered);
  }

  std::default_random_engine random_engine_;
  std::uniform_real_distribution<double> uniform_real_dist_{-1.0, 1.0};

  double joint_position_tau_{0.04};
  lbr_fri_ros2::CommandGuardParameters cmd_guard_params_;
  lbr_fri_ros2::StateInterfaceParameters state_interface_params_;

  std::shared_ptr<lbr_fri_ros2::StateInterface> state_interface_;
  std::shared_ptr<lbr_fri_ros2::BaseCommandInterface> command_interface_;

  // pseudo application
  std::shared_ptr<KUKA::FRI::LBRClient> lbr_client_;
  std::unique_ptr<KUKA::FRI::UdpConnection> udp_connection_;
  std::unique_ptr<KUKA::FRI::ClientApplication> pseudo_application_;
};

TEST_F(TestCommandInterfaces, TestPositionCommandInterface) {
#if FRI_CLIENT_VERSION_MAJOR == 1
  set_up(KUKA::FRI::EClientCommandMode::POSITION);
#endif
#if FRI_CLIENT_VERSION_MAJOR >= 2
  set_up(KUKA::FRI::EClientCommandMode::JOINT_POSITION);
#endif
  test_simple();
}

TEST_F(TestCommandInterfaces, TestTorqueCommandInterface) {
  set_up(KUKA::FRI::EClientCommandMode::TORQUE);
  test_simple();
}

TEST_F(TestCommandInterfaces, TestWrenchCommandInterface) {
  set_up(KUKA::FRI::EClientCommandMode::WRENCH);
  test_simple();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
