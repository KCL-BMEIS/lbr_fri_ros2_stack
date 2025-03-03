#include "lbr_ros2_control/controllers/io_command_controller.hpp"

namespace lbr_ros2_control {

IOCommandController::IOCommandController()
: rt_analog_command_ptr_(nullptr), rt_boolean_command_ptr_(nullptr), rt_digital_command_ptr_(nullptr)
{
}

controller_interface::InterfaceConfiguration IOCommandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 将模拟、布尔、数字IO信号接口名称全部加入配置
  for (const auto & name : analog_io_names_) {
    config.names.push_back(name);
  }
  for (const auto & name : boolean_io_names_) {
    config.names.push_back(name);
  }
  for (const auto & name : digital_io_names_) {
    config.names.push_back(name);
  }
  return config;
}

controller_interface::InterfaceConfiguration IOCommandController::state_interface_configuration() const
{
  // 本控制器不需要状态接口
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE
  };
}

controller_interface::CallbackReturn IOCommandController::on_init()
{
  try {
    // 从参数服务器中获取IO信号名称（你可以在启动文件中设置这些参数）
    if (!this->get_node()->has_parameter("analog_io_names")) {
      this->get_node()->declare_parameter("analog_io_names", std::vector<std::string>());
    }
    if (!this->get_node()->has_parameter("boolean_io_names")) {
      this->get_node()->declare_parameter("boolean_io_names", std::vector<std::string>());
    }
    if (!this->get_node()->has_parameter("digital_io_names")) {
      this->get_node()->declare_parameter("digital_io_names", std::vector<std::string>());
    }
    analog_io_names_ = this->get_node()->get_parameter("analog_io_names").as_string_array();
    boolean_io_names_ = this->get_node()->get_parameter("boolean_io_names").as_string_array();
    digital_io_names_ = this->get_node()->get_parameter("digital_io_names").as_string_array();

    // 创建订阅者
    analog_command_subscriber_ = this->get_node()->create_subscription<lbr_fri_idl::msg::LBRAnalogCommand>(
      "analog_command_topic", rclcpp::SystemDefaultsQoS(),
      [this](const lbr_fri_idl::msg::LBRAnalogCommand::SharedPtr msg) {
        rt_analog_command_ptr_.writeFromNonRT(msg);
      });

    boolean_command_subscriber_ = this->get_node()->create_subscription<lbr_fri_idl::msg::LBRBooleanCommand>(
      "boolean_command_topic", rclcpp::SystemDefaultsQoS(),
      [this](const lbr_fri_idl::msg::LBRBooleanCommand::SharedPtr msg) {
        rt_boolean_command_ptr_.writeFromNonRT(msg);
      });

    digital_command_subscriber_ = this->get_node()->create_subscription<lbr_fri_idl::msg::LBRDigitalCommand>(
      "digital_command_topic", rclcpp::SystemDefaultsQoS(),
      [this](const lbr_fri_idl::msg::LBRDigitalCommand::SharedPtr msg) {
        rt_digital_command_ptr_.writeFromNonRT(msg);
      });

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Exception in IOCommandController on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IOCommandController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 在这里设置命令接口和映射
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IOCommandController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  (void)previous_state;  // 防止未使用警告
  // 在此处继续进行其他激活操作
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IOCommandController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  (void)previous_state;  // 防止未使用警告
  // 在此处继续进行其他停用操作
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type IOCommandController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 读取实时缓冲区中的IO命令消息
  auto analog_command = rt_analog_command_ptr_.readFromRT();
  auto boolean_command = rt_boolean_command_ptr_.readFromRT();
  auto digital_command = rt_digital_command_ptr_.readFromRT();

  if (analog_command) {
    // 更新模拟命令处理
  }

  if (boolean_command) {
    // 更新布尔命令处理
  }

  if (digital_command) {
    // 更新数字命令处理
  }

  return controller_interface::return_type::OK;
}

}  // namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::IOCommandController, controller_interface::ControllerInterface)