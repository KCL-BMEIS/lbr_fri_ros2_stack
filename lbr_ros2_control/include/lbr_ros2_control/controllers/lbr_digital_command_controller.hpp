#ifndef LBR_ROS2_CONTROL__LBR_DIGITAL_COMMAND_CONTROLLER_HPP_
#define LBR_ROS2_CONTROL__LBR_DIGITAL_COMMAND_CONTROLLER_HPP_

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

#include "lbr_fri_idl/msg/lbr_digital_command.hpp"

namespace lbr_ros2_control {

class LBRDigitalCommandController : public controller_interface::ControllerInterface {
public:
  LBRDigitalCommandController();

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
  // 数字 IO 通道的名称（8 个通道）
  std::vector<std::string> digital_io_names_ = {
      "digital_io_1", "digital_io_2", "digital_io_3", "digital_io_4",
      "digital_io_5", "digital_io_6", "digital_io_7", "digital_io_8"};

  // 实时缓冲区，用于存储数字命令
  realtime_tools::RealtimeBuffer<lbr_fri_idl::msg::LBRDigitalCommand::SharedPtr>
      rt_digital_command_ptr_;
  
  // ROS 2 订阅，用于接收数字 IO 命令
  rclcpp::Subscription<lbr_fri_idl::msg::LBRDigitalCommand>::SharedPtr digital_command_subscription_ptr_;
};

} // namespace lbr_ros2_control

#endif // LBR_ROS2_CONTROL__LBR_DIGITAL_COMMAND_CONTROLLER_HPP_
