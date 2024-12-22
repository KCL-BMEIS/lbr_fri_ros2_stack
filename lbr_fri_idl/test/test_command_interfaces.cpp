#include <gtest/gtest.h>

#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_command.hpp"
#include "lbr_fri_idl/msg/lbr_joint_position_command.hpp"
#include "lbr_fri_idl/msg/lbr_torque_command.hpp"
#include "lbr_fri_idl/msg/lbr_wrench_command.hpp"

TEST(lbr_fri_idl, test_command_size) {
  using namespace KUKA::FRI;

  lbr_fri_idl::msg::LBRCommand lbr_command;
  EXPECT_EQ(lbr_command.joint_position.size(), LBRState::NUMBER_OF_JOINTS);
  EXPECT_EQ(lbr_command.torque.size(), LBRState::NUMBER_OF_JOINTS);

  lbr_fri_idl::msg::LBRJointPositionCommand lbr_position_command;
  EXPECT_EQ(lbr_position_command.joint_position.size(), LBRState::NUMBER_OF_JOINTS);

  lbr_fri_idl::msg::LBRTorqueCommand lbr_torque_command;
  EXPECT_EQ(lbr_torque_command.torque.size(), LBRState::NUMBER_OF_JOINTS);
  EXPECT_EQ(lbr_torque_command.joint_position.size(), LBRState::NUMBER_OF_JOINTS);

  lbr_fri_idl::msg::LBRWrenchCommand lbr_wrench_command;
  EXPECT_EQ(lbr_wrench_command.joint_position.size(), LBRState::NUMBER_OF_JOINTS);
  EXPECT_EQ(lbr_wrench_command.wrench.size(), 6);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}