#ifndef LBR_ROS2_CONTROL__IO_COMMAND_CONTROLLER_HPP_
#define LBR_ROS2_CONTROL__IO_COMMAND_CONTROLLER_HPP_

#include <string>
#include <array>
#include <vector>
#include <unordered_map>
#include <memory>
#include <functional>
#include <stdexcept>
#include <map>
#include <algorithm>
#include <atomic>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"

#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_analog_command.hpp"
#include "lbr_fri_idl/msg/lbr_boolean_command.hpp"
#include "lbr_fri_idl/msg/lbr_digital_command.hpp"
#include "lbr_fri_ros2/types.hpp"

namespace lbr_ros2_control {

class IOCommandController : public controller_interface::ControllerInterface {
public:
  IOCommandController();

  // 配置发送接口，所有IO信号的完整名称由参数指定
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  // 本控制器不需要状态接口，故返回NONE
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // IO信号名称，从参数服务器中获取
  std::vector<std::string> analog_io_names_;
  std::vector<std::string> boolean_io_names_;
  std::vector<std::string> digital_io_names_;

  // 存储所有需要发送的命令接口映射，key 为接口全名
  std::unordered_map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_interfaces_map_;

  // 实时缓冲区存储订阅到的IO命令
  realtime_tools::RealtimeBuffer<lbr_fri_idl::msg::LBRAnalogCommand::SharedPtr> rt_analog_command_ptr_;
  realtime_tools::RealtimeBuffer<lbr_fri_idl::msg::LBRBooleanCommand::SharedPtr> rt_boolean_command_ptr_;
  realtime_tools::RealtimeBuffer<lbr_fri_idl::msg::LBRDigitalCommand::SharedPtr> rt_digital_command_ptr_;

  // 订阅IO命令消息
  rclcpp::Subscription<lbr_fri_idl::msg::LBRAnalogCommand>::SharedPtr analog_command_subscriber_;
  rclcpp::Subscription<lbr_fri_idl::msg::LBRBooleanCommand>::SharedPtr boolean_command_subscriber_;
  rclcpp::Subscription<lbr_fri_idl::msg::LBRDigitalCommand>::SharedPtr digital_command_subscriber_;

  // 可根据需要添加模板帮助函数（例如从命令接口列表中创建映射）
  template <typename T>
  std::unordered_map<std::string, std::reference_wrapper<T>> create_interface_map(
    const std::vector<std::string> & interface_names,
    std::vector<T> & configured_interfaces);
};

}  // namespace lbr_ros2_control

#endif  // LBR_ROS2_CONTROL__IO_COMMAND_CONTROLLER_HPP_
