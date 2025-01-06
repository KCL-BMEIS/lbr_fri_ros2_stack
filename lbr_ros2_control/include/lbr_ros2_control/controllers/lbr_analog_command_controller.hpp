#ifndef LBR_ROS2_CONTROL__LBR_ANALOG_COMMAND_CONTROLLER_HPP_
#define LBR_ROS2_CONTROL__LBR_ANALOG_COMMAND_CONTROLLER_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"

#include "lbr_fri_idl/msg/lbr_analog_command.hpp"

namespace lbr_ros2_control {

class LBRAnalogCommandController : public controller_interface::ControllerInterface {
public:
  LBRAnalogCommandController();

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
  // 模拟 IO 通道的名称
  std::vector<std::string> analog_io_names_ = {"analog_io_1", "analog_io_2"};

  // 实时缓冲区，用于存储模拟命令
  realtime_tools::RealtimeBuffer<lbr_fri_idl::msg::LBRAnalogCommand::SharedPtr>
      rt_analog_command_ptr_;
  
  // ROS 2 订阅，用于接收模拟 IO 命令
  rclcpp::Subscription<lbr_fri_idl::msg::LBRAnalogCommand>::SharedPtr analog_command_subscription_ptr_;
};

} // namespace lbr_ros2_control

#endif // LBR_ROS2_CONTROL__LBR_ANALOG_COMMAND_CONTROLLER_HPP_
