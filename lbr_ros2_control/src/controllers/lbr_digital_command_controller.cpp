#include "lbr_ros2_control/controllers/lbr_digital_command_controller.hpp"

namespace lbr_ros2_control {

LBRDigitalCommandController::LBRDigitalCommandController()
    : rt_digital_command_ptr_(nullptr), digital_command_subscription_ptr_(nullptr) {}

controller_interface::InterfaceConfiguration
LBRDigitalCommandController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 为每个数字 IO 通道添加命令接口
  for (const auto &io_name : digital_io_names_) {
    interface_configuration.names.push_back(io_name + "/value");
  }

  return interface_configuration;
}

controller_interface::InterfaceConfiguration
LBRDigitalCommandController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn LBRDigitalCommandController::on_init() {
  try {
    // 创建订阅，用于接收数字 IO 命令
    digital_command_subscription_ptr_ =
        this->get_node()->create_subscription<lbr_fri_idl::msg::LBRDigitalCommand>(
            "command/digital_io", 1,
            [this](const lbr_fri_idl::msg::LBRDigitalCommand::SharedPtr msg) {
              rt_digital_command_ptr_.writeFromNonRT(msg);
            });
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to initialize LBR digital command controller with: %s.", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
LBRDigitalCommandController::update(const rclcpp::Time & /*time*/,
                                    const rclcpp::Duration & /*period*/) {
  auto digital_command = rt_digital_command_ptr_.readFromRT();
  if (!digital_command || !(*digital_command)) {
    return controller_interface::return_type::OK;
  }

  // 将接收到的数字命令值写入硬件接口
  size_t idx = 0;
  for (auto &command_interface : command_interfaces_) {
    if (idx < (*digital_command)->digital_value.size()) {
      command_interface.set_value((*digital_command)->digital_value[idx++]);
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn LBRDigitalCommandController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRDigitalCommandController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LBRDigitalCommandController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::LBRDigitalCommandController,
                       controller_interface::ControllerInterface)
