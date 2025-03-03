#include "lbr_ros2_control/controllers/admittance_controller.hpp"

namespace lbr_ros2_control {
AdmittanceController::AdmittanceController() {}

controller_interface::InterfaceConfiguration
AdmittanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return interface_configuration;
}

controller_interface::InterfaceConfiguration
AdmittanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // joint position interface
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }

  // estimated force-torque sensor interface
  for (const auto &interface_name : estimated_ft_sensor_ptr_->get_state_interface_names()) {
    interface_configuration.names.push_back(interface_name);
  }

  // additional state interfaces
  interface_configuration.names.push_back(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                          HW_IF_SAMPLE_TIME);
  interface_configuration.names.push_back(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                          HW_IF_SESSION_STATE);
  return interface_configuration;
}

controller_interface::CallbackReturn AdmittanceController::on_init() {
  RCLCPP_ERROR(
      this->get_node()->get_logger(),
      "The admittance controller currently requires custom lbr_system_config.yaml configurations. "
      "Therefore, only experienced users should use it, then remove this error and strictly "
      "follow "
      "https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_demos/"
      "lbr_demos_advanced_cpp/doc/lbr_demos_advanced_cpp.html#admittance-controller. "
      "This error can be removed when a) the controller works with default system configurations "
      "and b) the "
      "controller checks that load data was successfully calibrated.");
  return controller_interface::CallbackReturn::ERROR;

  try {
    if (!this->get_node()->has_parameter("robot_description")) {
      this->get_node()->declare_parameter("robot_description", "");
    }
    if (!this->get_node()->has_parameter("robot_name")) {
      this->get_node()->declare_parameter("robot_name", "lbr");
    }
    if (!this->get_node()->has_parameter("admittance.mass")) {
      this->get_node()->declare_parameter("admittance.mass",
                                          std::vector<double>(lbr_fri_ros2::CARTESIAN_DOF, 1.0));
    }
    if (!this->get_node()->has_parameter("admittance.damping")) {
      this->get_node()->declare_parameter("admittance.damping",
                                          std::vector<double>(lbr_fri_ros2::CARTESIAN_DOF, 0.0));
    }
    if (!this->get_node()->has_parameter("admittance.stiffness")) {
      this->get_node()->declare_parameter("admittance.stiffness",
                                          std::vector<double>(lbr_fri_ros2::CARTESIAN_DOF, 0.0));
    }
    if (!this->get_node()->has_parameter("inv_jac_ctrl.chain_root")) {
      this->get_node()->declare_parameter("inv_jac_ctrl.chain_root", "lbr_link_0");
    }
    if (!this->get_node()->has_parameter("inv_jac_ctrl.chain_tip")) {
      this->get_node()->declare_parameter("inv_jac_ctrl.chain_tip", "lbr_link_ee");
    }
    if (!this->get_node()->has_parameter("inv_jac_ctrl.damping")) {
      this->get_node()->declare_parameter("inv_jac_ctrl.damping", 0.2);
    }
    if (!this->get_node()->has_parameter("inv_jac_ctrl.max_linear_velocity")) {
      this->get_node()->declare_parameter("inv_jac_ctrl.max_linear_velocity", 0.1);
    }
    if (!this->get_node()->has_parameter("inv_jac_ctrl.max_angular_velocity")) {
      this->get_node()->declare_parameter("inv_jac_ctrl.max_angular_velocity", 0.1);
    }
    if (!this->get_node()->has_parameter("inv_jac_ctrl.joint_gains")) {
      this->get_node()->declare_parameter("inv_jac_ctrl.joint_gains",
                                          std::vector<double>(lbr_fri_ros2::N_JNTS, 0.0));
    }
    if (!this->get_node()->has_parameter("inv_jac_ctrl.cartesian_gains")) {
      this->get_node()->declare_parameter("inv_jac_ctrl.cartesian_gains",
                                          std::vector<double>(lbr_fri_ros2::CARTESIAN_DOF, 0.0));
    }
    robot_description_ = this->get_node()->get_parameter("robot_description").as_string();
    if (robot_description_.empty()) {
      throw std::runtime_error("No robot description provided");
    }
    configure_joint_names_();
    configure_admittance_impl_();
    configure_inv_jac_ctrl_impl_();
    log_info_();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to initialize admittance controller with: %s.", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AdmittanceController::update(const rclcpp::Time & /*time*/,
                                                               const rclcpp::Duration &period) {
  // get estimated force-torque sensor values
  f_ext_.head(3) =
      Eigen::Map<Eigen::Matrix<double, 3, 1>>(estimated_ft_sensor_ptr_->get_forces().data());
  f_ext_.tail(3) =
      Eigen::Map<Eigen::Matrix<double, 3, 1>>(estimated_ft_sensor_ptr_->get_torques().data());

  // get joint positions
  std::for_each(q_.begin(), q_.end(), [&, i = 0](double &q_i) mutable {
    q_i = this->state_interfaces_[i].get_value();
    ++i;
  });

  // compute forward kinematics
  auto chain_tip_frame = inv_jac_ctrl_impl_ptr_->get_kinematics_ptr()->compute_fk(q_);
  t_ = Eigen::Map<Eigen::Matrix<double, 3, 1>>(chain_tip_frame.p.data);
  r_ = Eigen::Quaterniond(chain_tip_frame.M.data);

  // compute steady state position and orientation
  if (!initialized_) {
    t_init_ = t_;
    t_prev_ = t_init_;
    r_init_ = r_;
    r_prev_ = r_init_;
    initialized_ = true;
  }

  // compute translational delta and velocity
  delta_x_.head(3) = (t_ - t_init_);
  dx_.head(3) = (t_ - t_prev_) / period.seconds();

  // compute rotational delta and veloctity
  Eigen::AngleAxisd deltaa(r_.inverse() * r_init_);
  delta_x_.tail(4) = deltaa.axis() * deltaa.angle();
  Eigen::AngleAxisd da(r_.inverse() * r_prev_);
  dx_.tail(3) = da.axis() * da.angle();

  // update previous values
  t_prev_ = t_;
  r_prev_ = r_;

  // convert f_ext_ back to root frame
  f_ext_.head(3) = Eigen::Matrix3d::Map(chain_tip_frame.M.data).transpose() * f_ext_.head(3);
  f_ext_.tail(3) = Eigen::Matrix3d::Map(chain_tip_frame.M.data).transpose() * f_ext_.tail(3);

  // compute admittance
  admittance_impl_ptr_->compute(f_ext_, delta_x_, dx_, ddx_);

  // integrate ddx_ to command velocity
  twist_command_ = ddx_ * period.seconds();

  if (!inv_jac_ctrl_impl_ptr_) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Inverse Jacobian controller not initialized.");
    return controller_interface::return_type::ERROR;
  }
  if (static_cast<int>(session_state_interface_ptr_->get().get_value()) !=
      KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
    return controller_interface::return_type::OK;
  }

  // compute the joint velocity from the twist command target
  inv_jac_ctrl_impl_ptr_->compute(twist_command_, q_, dq_);

  // pass joint positions to hardware
  std::for_each(q_.begin(), q_.end(), [&, i = 0](const double &q_i) mutable {
    this->command_interfaces_[i].set_value(
        q_i + dq_[i] * sample_time_state_interface_ptr_->get().get_value());
    ++i;
  });

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
AdmittanceController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  estimated_ft_sensor_ptr_ = std::make_unique<semantic_components::ForceTorqueSensor>(
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_FORCE_X,
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_FORCE_Y,
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_FORCE_Z,
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_TORQUE_X,
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_TORQUE_Y,
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_TORQUE_Z);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AdmittanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!reference_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  zero_all_values_();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AdmittanceController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  clear_state_interfaces_();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool AdmittanceController::reference_state_interfaces_() {
  for (auto &state_interface : state_interfaces_) {
    if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      joint_position_state_interfaces_.emplace_back(std::ref(state_interface));
    }
    if (state_interface.get_interface_name() == HW_IF_SAMPLE_TIME) {
      sample_time_state_interface_ptr_ =
          std::make_unique<std::reference_wrapper<hardware_interface::LoanedStateInterface>>(
              std::ref(state_interface));
    }
    if (state_interface.get_interface_name() == HW_IF_SESSION_STATE) {
      session_state_interface_ptr_ =
          std::make_unique<std::reference_wrapper<hardware_interface::LoanedStateInterface>>(
              std::ref(state_interface));
    }
  }
  if (!estimated_ft_sensor_ptr_->assign_loaned_state_interfaces(state_interfaces_)) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to assign estimated force torque state interfaces.");
    return false;
  }
  if (joint_position_state_interfaces_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint position state interfaces '%ld' does not match the number of joints "
        "in the robot '%d'.",
        joint_position_state_interfaces_.size(), lbr_fri_ros2::N_JNTS);
    return false;
  }
  return true;
}

void AdmittanceController::clear_state_interfaces_() {
  joint_position_state_interfaces_.clear();
  estimated_ft_sensor_ptr_->release_interfaces();
}

void AdmittanceController::configure_joint_names_() {
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

void AdmittanceController::configure_admittance_impl_() {
  if (this->get_node()->get_parameter("admittance.mass").as_double_array().size() !=
      lbr_fri_ros2::CARTESIAN_DOF) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Number of mass values (%ld) does not match the number of cartesian degrees of "
                 "freedom (%d).",
                 this->get_node()->get_parameter("admittance.mass").as_double_array().size(),
                 lbr_fri_ros2::CARTESIAN_DOF);
    throw std::runtime_error("Failed to configure admittance parameters.");
  }
  if (this->get_node()->get_parameter("admittance.damping").as_double_array().size() !=
      lbr_fri_ros2::CARTESIAN_DOF) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of damping values (%ld) does not match the number of cartesian degrees of freedom "
        "(%d).",
        this->get_node()->get_parameter("admittance.damping").as_double_array().size(),
        lbr_fri_ros2::CARTESIAN_DOF);
    throw std::runtime_error("Failed to configure admittance parameters.");
  }
  if (this->get_node()->get_parameter("admittance.stiffness").as_double_array().size() !=
      lbr_fri_ros2::CARTESIAN_DOF) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Number of stiffness values (%ld) does not match the number of cartesian degrees "
                 "of freedom "
                 "(%d).",
                 this->get_node()->get_parameter("admittance.stiffness").as_double_array().size(),
                 lbr_fri_ros2::CARTESIAN_DOF);
    throw std::runtime_error("Failed to configure admittance parameters.");
  }
  lbr_fri_ros2::cart_array_t mass_array;
  for (unsigned int i = 0; i < lbr_fri_ros2::CARTESIAN_DOF; ++i) {
    mass_array[i] = this->get_node()->get_parameter("admittance.mass").as_double_array()[i];
  }
  lbr_fri_ros2::cart_array_t damping_array;
  for (unsigned int i = 0; i < lbr_fri_ros2::CARTESIAN_DOF; ++i) {
    damping_array[i] = this->get_node()->get_parameter("admittance.damping").as_double_array()[i];
  }
  lbr_fri_ros2::cart_array_t stiffness_array;
  for (unsigned int i = 0; i < lbr_fri_ros2::CARTESIAN_DOF; ++i) {
    stiffness_array[i] =
        this->get_node()->get_parameter("admittance.stiffness").as_double_array()[i];
  }
  admittance_impl_ptr_ = std::make_unique<lbr_fri_ros2::AdmittanceImpl>(
      lbr_fri_ros2::AdmittanceParameters{mass_array, damping_array, stiffness_array});
}

void AdmittanceController::configure_inv_jac_ctrl_impl_() {
  if (this->get_node()->get_parameter("inv_jac_ctrl.joint_gains").as_double_array().size() !=
      lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint gains (%ld) does not match the number of joints in the robot (%d).",
        this->get_node()->get_parameter("inv_jac_ctrl.joint_gains").as_double_array().size(),
        lbr_fri_ros2::N_JNTS);
    throw std::runtime_error("Failed to configure joint gains.");
  }
  if (this->get_node()->get_parameter("inv_jac_ctrl.cartesian_gains").as_double_array().size() !=
      lbr_fri_ros2::CARTESIAN_DOF) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of cartesian gains (%ld) does not match the number of cartesian degrees of freedom "
        "(%d).",
        this->get_node()->get_parameter("inv_jac_ctrl.cartesian_gains").as_double_array().size(),
        lbr_fri_ros2::CARTESIAN_DOF);
    throw std::runtime_error("Failed to configure cartesian gains.");
  }
  lbr_fri_ros2::jnt_array_t joint_gains_array;
  for (unsigned int i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    joint_gains_array[i] =
        this->get_node()->get_parameter("inv_jac_ctrl.joint_gains").as_double_array()[i];
  }
  lbr_fri_ros2::cart_array_t cartesian_gains_array;
  for (unsigned int i = 0; i < lbr_fri_ros2::CARTESIAN_DOF; ++i) {
    cartesian_gains_array[i] =
        this->get_node()->get_parameter("inv_jac_ctrl.cartesian_gains").as_double_array()[i];
  }
  inv_jac_ctrl_impl_ptr_ = std::make_unique<lbr_fri_ros2::InvJacCtrlImpl>(
      robot_description_,
      lbr_fri_ros2::InvJacCtrlParameters{
          this->get_node()->get_parameter("inv_jac_ctrl.chain_root").as_string(),
          this->get_node()->get_parameter("inv_jac_ctrl.chain_tip").as_string(),
          false, // always assume twist in root frame
          this->get_node()->get_parameter("inv_jac_ctrl.damping").as_double(),
          this->get_node()->get_parameter("inv_jac_ctrl.max_linear_velocity").as_double(),
          this->get_node()->get_parameter("inv_jac_ctrl.max_angular_velocity").as_double(),
          joint_gains_array, cartesian_gains_array});
}

void AdmittanceController::zero_all_values_() {
  f_ext_.setZero();
  delta_x_.setZero();
  dx_.setZero();
  ddx_.setZero();
  std::fill(dq_.begin(), dq_.end(), 0.0);
  twist_command_.setZero();
}

void AdmittanceController::log_info_() const {
  admittance_impl_ptr_->log_info();
  inv_jac_ctrl_impl_ptr_->log_info();
}
} // namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::AdmittanceController,
                       controller_interface::ControllerInterface)