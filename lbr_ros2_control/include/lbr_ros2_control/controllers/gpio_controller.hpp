#ifndef LBR_ROS2_CONTROL__GPIO_CONTROLLER_HPP_
#define LBR_ROS2_CONTROL__GPIO_CONTROLLER_HPP_

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

#include "lbr_fri_idl/msg/spindle_control.hpp"
#include "lbr_fri_ros2/types.hpp"
#include "lbr_ros2_control/system_interface_type_values.hpp"

namespace lbr_ros2_control {

class GPIOController : public controller_interface::ControllerInterface {
public:
  GPIOController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::return_type update(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

protected:
  // 修改后不再使用 gpio_names_ 和 configure_gpio_names_()
  bool reference_command_interfaces_();
  void clear_command_interfaces_();

  // 用于存放已查找到的命令接口（仅一组接口）
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> 
    spindle_speed_command_interfaces_, spindle_start_command_interfaces_;

  realtime_tools::RealtimeBuffer<lbr_fri_idl::msg::SpindleControl::SharedPtr> rt_spindle_command_ptr_;
  rclcpp::Subscription<lbr_fri_idl::msg::SpindleControl>::SharedPtr spindle_command_subscription_ptr_;
};

} // namespace lbr_ros2_control

#endif // LBR_ROS2_CONTROL__GPIO_CONTROLLER_HPP_
