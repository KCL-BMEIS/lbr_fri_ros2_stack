#include "lbr_ros2_control/controllers/lbr_joint_position_command_controller.hpp"

namespace lbr_ros2_control {
LBRJointPositionCommandController::LBRJointPositionCommandController()
    : rt_lbr_joint_position_command_ptr_(nullptr),
      lbr_joint_position_command_subscription_ptr_(nullptr) {}

controller_interface::InterfaceConfiguration
LBRJointPositionCommandController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return interface_configuration;
}

controller_interface::InterfaceConfiguration
LBRJointPositionCommandController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn LBRJointPositionCommandController::on_init() {
  try {
    lbr_joint_position_command_subscription_ptr_ =
        this->get_node()->create_subscription<lbr_fri_idl::msg::LBRJointPositionCommand>(
            "command/joint_position", 1,
            [this](const lbr_fri_idl::msg::LBRJointPositionCommand::SharedPtr msg) {
              rt_lbr_joint_position_command_ptr_.writeFromNonRT(msg);
            });
    if (!this->get_node()->has_parameter("robot_name")) {
      this->get_node()->declare_parameter("robot_name", "lbr");
    }
    configure_joint_names_();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to initialize LBR position command controller with: %s.", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
LBRJointPositionCommandController::update(const rclcpp::Time & /*time*/,
                                          const rclcpp::Duration & /*period*/) {
  auto lbr_joint_position_command = rt_lbr_joint_position_command_ptr_.readFromRT();
  if (!lbr_joint_position_command || !(*lbr_joint_position_command)) {
    return controller_interface::return_type::OK;
  }
  std::for_each(
      command_interfaces_.begin(), command_interfaces_.end(),
      [lbr_joint_position_command, idx = 0](auto &command_interface) mutable {
        command_interface.set_value(
            (*lbr_joint_position_command)
                ->joint_position[idx++]); // important to post-increment idx, don't use ++idx
      });
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn LBRJointPositionCommandController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRJointPositionCommandController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LBRJointPositionCommandController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

void LBRJointPositionCommandController::configure_joint_names_() {
  if (joint_names_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint names (%ld) does not match the number of joints in the robot (%d).",
        joint_names_.size(), lbr_fri_ros2::N_JNTS);
    throw std::runtime_error("Failed to configure joint names.");
  }
  std::string robot_name = this->get_node()->get_parameter("robot_name").as_string();
  for (int i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    joint_names_[i] = robot_name + "_A" + std::to_string(i + 1);
  }
}
} // namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::LBRJointPositionCommandController,
                       controller_interface::ControllerInterface)
