#include "lbr_ros2_control/controllers/gpio_controller.hpp"

namespace lbr_ros2_control {

GPIOController::GPIOController()
    : rt_spindle_command_ptr_(nullptr), spindle_command_subscription_ptr_(nullptr) {}

controller_interface::InterfaceConfiguration GPIOController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 仅添加两组接口：模拟量速度和布尔量启动
  interface_configuration.names.push_back(std::string(HW_IF_GPIO_PREFIX) + "/" + HW_IF_ANALOG_SPEED);
  interface_configuration.names.push_back(std::string(HW_IF_GPIO_PREFIX) + "/" + HW_IF_BOOLEAN_START);

  return interface_configuration;
}

controller_interface::InterfaceConfiguration GPIOController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn GPIOController::on_init() {
  try {
    spindle_command_subscription_ptr_ =
        this->get_node()->create_subscription<lbr_fri_idl::msg::SpindleControl>(
            "command/spindle", 1, [this](const lbr_fri_idl::msg::SpindleControl::SharedPtr msg) {
                // 在接收到消息时增加调试输出
              RCLCPP_DEBUG(this->get_node()->get_logger(), 
                           "Subscription callback: Received spindle command - Speed: %.2f, Start: %.2f",
                           msg->spindle_speed, msg->spindle_start);
              rt_spindle_command_ptr_.writeFromNonRT(msg);
            });

    // 如果有必要，可在此处声明其它参数（例如 robot_name ），但这里已不再使用 gpio_names_
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to initialize GPIO controller with: %s.", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GPIOController::update(const rclcpp::Time & /*time*/,
                                                         const rclcpp::Duration & /*period*/) {
  auto spindle_command = rt_spindle_command_ptr_.readFromRT();
  if (!spindle_command || !(*spindle_command)) {
    RCLCPP_DEBUG(this->get_node()->get_logger(), "No spindle command available in update().");
    return controller_interface::return_type::OK;
  }

    // 输出收到的命令数据
  RCLCPP_DEBUG(this->get_node()->get_logger(), 
               "Update(): Received spindle command - Speed: %.2f, Start: %.2f", 
               (*spindle_command)->spindle_speed, (*spindle_command)->spindle_start);

  // 设置 spindle speed（模拟量接口）
  spindle_speed_command_interfaces_[0].get().set_value((*spindle_command)->spindle_speed);

  // 设置 spindle start（布尔接口）
  spindle_start_command_interfaces_[0].get().set_value(static_cast<bool>((*spindle_command)->spindle_start));

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn GPIOController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GPIOController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!reference_command_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GPIOController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  clear_command_interfaces_();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool GPIOController::reference_command_interfaces_() {
  // 清空已有接口引用
  spindle_speed_command_interfaces_.clear();
  spindle_start_command_interfaces_.clear();

  // 明确匹配两组接口
  bool found_speed = false;
  bool found_start = false;

  for (auto &command_interface : command_interfaces_) {
    // 期望的接口名称为： "gpio/analog_speed" 和 "gpio/boolean_start"
    if ((command_interface.get_prefix_name() == std::string(HW_IF_GPIO_PREFIX)) &&
        (command_interface.get_interface_name() == HW_IF_ANALOG_SPEED)) {
      spindle_speed_command_interfaces_.emplace_back(std::ref(command_interface));
      found_speed = true;
      RCLCPP_DEBUG(this->get_node()->get_logger(), "Found spindle speed interface: %s/%s",
                   HW_IF_GPIO_PREFIX, HW_IF_ANALOG_SPEED);
    } else if ((command_interface.get_prefix_name() == std::string(HW_IF_GPIO_PREFIX)) &&
               (command_interface.get_interface_name() == HW_IF_BOOLEAN_START)) {
      spindle_start_command_interfaces_.emplace_back(std::ref(command_interface));
      found_start = true;
      RCLCPP_DEBUG(this->get_node()->get_logger(), "Found spindle start interface: %s/%s",
                   HW_IF_GPIO_PREFIX, HW_IF_BOOLEAN_START);
    }
  }

  if (!found_speed) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Spindle speed interface not found.");
    return false;
  }
  if (!found_start) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Spindle start interface not found.");
    return false;
  }

  return true;
}

void GPIOController::clear_command_interfaces_() {
  spindle_speed_command_interfaces_.clear();
  spindle_start_command_interfaces_.clear();
}

} // namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::GPIOController, controller_interface::ControllerInterface)
