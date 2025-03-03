#ifndef LBR_ROS2_CONTROL__LBR_WRENCH_COMMAND_CONTROLLER_HPP_
#define LBR_ROS2_CONTROL__LBR_WRENCH_COMMAND_CONTROLLER_HPP_

#include <array>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"

#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_wrench_command.hpp"
#include "lbr_fri_ros2/types.hpp"
#include "lbr_ros2_control/system_interface_type_values.hpp"

namespace lbr_ros2_control {
class LBRWrenchCommandController : public controller_interface::ControllerInterface {
  static constexpr uint8_t CARTESIAN_DOF = 6;

public:
  LBRWrenchCommandController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::return_type update(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

protected:
  bool reference_command_interfaces_();
  void clear_command_interfaces_();
  void configure_joint_names_();

  lbr_fri_ros2::jnt_name_array_t joint_names_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      joint_position_command_interfaces_, wrench_command_interfaces_;

  realtime_tools::RealtimeBuffer<lbr_fri_idl::msg::LBRWrenchCommand::SharedPtr>
      rt_lbr_wrench_command_ptr_;
  rclcpp::Subscription<lbr_fri_idl::msg::LBRWrenchCommand>::SharedPtr
      lbr_wrench_command_subscription_ptr_;
};
} // namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__LBR_WRENCH_COMMAND_CONTROLLER_HPP_
