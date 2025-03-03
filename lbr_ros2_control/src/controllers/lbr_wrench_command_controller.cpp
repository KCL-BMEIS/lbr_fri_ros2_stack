#include "lbr_ros2_control/controllers/lbr_wrench_command_controller.hpp"

namespace lbr_ros2_control {
LBRWrenchCommandController::LBRWrenchCommandController()
    : rt_lbr_wrench_command_ptr_(nullptr), lbr_wrench_command_subscription_ptr_(nullptr) {}

controller_interface::InterfaceConfiguration
LBRWrenchCommandController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  interface_configuration.names.push_back(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_FORCE_X);
  interface_configuration.names.push_back(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_FORCE_Y);
  interface_configuration.names.push_back(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_FORCE_Z);
  interface_configuration.names.push_back(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_TORQUE_X);
  interface_configuration.names.push_back(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_TORQUE_Y);
  interface_configuration.names.push_back(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_TORQUE_Z);
  return interface_configuration;
}

controller_interface::InterfaceConfiguration
LBRWrenchCommandController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn LBRWrenchCommandController::on_init() {
  try {
    lbr_wrench_command_subscription_ptr_ =
        this->get_node()->create_subscription<lbr_fri_idl::msg::LBRWrenchCommand>(
            "command/wrench", 1, [this](const lbr_fri_idl::msg::LBRWrenchCommand::SharedPtr msg) {
              rt_lbr_wrench_command_ptr_.writeFromNonRT(msg);
            });
    if (!this->get_node()->has_parameter("robot_name")) {
      this->get_node()->declare_parameter("robot_name", "lbr");
    }
    configure_joint_names_();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to initialize LBR wrench command controller with: %s.", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
LBRWrenchCommandController::update(const rclcpp::Time & /*time*/,
                                   const rclcpp::Duration & /*period*/) {
  auto lbr_wrench_command = rt_lbr_wrench_command_ptr_.readFromRT();
  if (!lbr_wrench_command || !(*lbr_wrench_command)) {
    return controller_interface::return_type::OK;
  }
  for (std::size_t idx = 0; idx < lbr_fri_ros2::N_JNTS; ++idx) {
    joint_position_command_interfaces_[idx].get().set_value(
        (*lbr_wrench_command)->joint_position[idx]);
  }
  for (std::size_t idx = 0; idx < CARTESIAN_DOF; ++idx) {
    wrench_command_interfaces_[idx].get().set_value((*lbr_wrench_command)->wrench[idx]);
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
LBRWrenchCommandController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRWrenchCommandController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!reference_command_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRWrenchCommandController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  clear_command_interfaces_();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool LBRWrenchCommandController::reference_command_interfaces_() {
  for (auto &command_interface : command_interfaces_) {
    if (command_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      joint_position_command_interfaces_.emplace_back(std::ref(command_interface));
    }
    if (command_interface.get_prefix_name() == HW_IF_WRENCH_PREFIX) {
      wrench_command_interfaces_.emplace_back(std::ref(command_interface));
    }
  }
  if (joint_position_command_interfaces_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint position command interfaces '%ld' does not match the number of joints "
        "in the robot '%d'.",
        joint_position_command_interfaces_.size(), lbr_fri_ros2::N_JNTS);
    return false;
  }
  if (wrench_command_interfaces_.size() != CARTESIAN_DOF) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Number of wrench command interfaces '%ld' does not equal %d.",
                 wrench_command_interfaces_.size(), CARTESIAN_DOF);
    return false;
  }
  return true;
}

void LBRWrenchCommandController::clear_command_interfaces_() {
  joint_position_command_interfaces_.clear();
  wrench_command_interfaces_.clear();
}

void LBRWrenchCommandController::configure_joint_names_() {
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

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::LBRWrenchCommandController,
                       controller_interface::ControllerInterface)
