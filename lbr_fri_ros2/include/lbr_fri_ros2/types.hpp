#ifndef LBR_FRI_ROS2__TYPES_HPP_
#define LBR_FRI_ROS2__TYPES_HPP_

#include <array>
#include <cstdint>
#include <string>

#include "friLBRClient.h"

#include "lbr_fri_idl/msg/lbr_command.hpp"
#include "lbr_fri_idl/msg/lbr_state.hpp"

namespace lbr_fri_ros2 {
// joint DoF alias
constexpr std::uint8_t N_JNTS = KUKA::FRI::LBRState::NUMBER_OF_JOINTS;

// joint positions, velocities, accelerations, torques etc.
using jnt_array_t = std::array<double, N_JNTS>;
using jnt_array_t_ref = jnt_array_t &;
using const_jnt_array_t_ref = const jnt_array_t &;

// joint names
using jnt_name_array_t = std::array<std::string, N_JNTS>;
using jnt_name_array_t_ref = jnt_name_array_t &;
using const_jnt_name_array_t_ref = const jnt_name_array_t &;

// Cartesian DoF
constexpr std::uint8_t CARTESIAN_DOF = 6;

// Cartesian positions, velocities, accelerations, wrenches etc.
using cart_array_t = std::array<double, CARTESIAN_DOF>;
using cart_array_t_ref = cart_array_t &;
using const_cart_array_t_ref = const cart_array_t &;

// FRI types
using fri_command_t = KUKA::FRI::LBRCommand;
using fri_command_t_ref = fri_command_t &;
using const_fri_command_t_ref = const fri_command_t &;
using fri_state_t = KUKA::FRI::LBRState;
using fri_state_t_ref = fri_state_t &;
using const_fri_state_t_ref = const fri_state_t &;

// ROS IDL types
using idl_command_t = lbr_fri_idl::msg::LBRCommand;
using idl_command_t_ref = idl_command_t &;
using const_idl_command_t_ref = const idl_command_t &;
using idl_state_t = lbr_fri_idl::msg::LBRState;
using idl_state_t_ref = idl_state_t &;
using const_idl_state_t_ref = const idl_state_t &;
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__TYPES_HPP_
