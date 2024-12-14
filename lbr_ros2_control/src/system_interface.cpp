#include "lbr_ros2_control/system_interface.hpp"

namespace lbr_ros2_control {
controller_interface::CallbackReturn
SystemInterface::on_init(const hardware_interface::HardwareInfo &system_info) {
  auto ret = hardware_interface::SystemInterface::on_init(system_info);
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        lbr_fri_ros2::ColorScheme::ERROR << "Failed to initialize SystemInterface"
                                                         << lbr_fri_ros2::ColorScheme::ENDC);
    return ret;
  }

  // parameters_ from lbr_system_interface.xacro (default configurations located in
  // lbr_description/ros2_control/lbr_system_interface.xacro)
  if (!parse_parameters_(system_info)) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // setup driver
  lbr_fri_ros2::CommandGuardParameters command_guard_parameters;
  lbr_fri_ros2::StateInterfaceParameters state_interface_parameters;
  for (std::size_t idx = 0; idx < system_info.joints.size(); ++idx) {
    command_guard_parameters.joint_names[idx] = system_info.joints[idx].name;
    command_guard_parameters.max_positions[idx] =
        std::stod(system_info.joints[idx].parameters.at("max_position"));
    command_guard_parameters.min_positions[idx] =
        std::stod(system_info.joints[idx].parameters.at("min_position"));
    command_guard_parameters.max_velocities[idx] =
        std::stod(system_info.joints[idx].parameters.at("max_velocity"));
    command_guard_parameters.max_torques[idx] =
        std::stod(system_info.joints[idx].parameters.at("max_torque"));
  }
  state_interface_parameters.external_torque_tau = parameters_.external_torque_tau;
  state_interface_parameters.measured_torque_tau = parameters_.measured_torque_tau;

  try {
    async_client_ptr_ = std::make_shared<lbr_fri_ros2::AsyncClient>(
        parameters_.client_command_mode, parameters_.joint_position_tau, command_guard_parameters,
        parameters_.command_guard_variant, state_interface_parameters, parameters_.open_loop);
    app_ptr_ = std::make_unique<lbr_fri_ros2::App>(async_client_ptr_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Failed to instantiate AsyncClient or App with: " << e.what()
                            << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::CallbackReturn::ERROR;
  }

  nan_command_interfaces_();
  nan_state_interfaces_();
  nan_last_hw_states_();

  // setup force-torque estimator
  std::transform(info_.sensors[1].parameters.at("enabled").begin(),
                 info_.sensors[1].parameters.at("enabled").end(),
                 info_.sensors[1].parameters.at("enabled").begin(),
                 ::tolower); // convert to lower case
  ft_parameters_.enabled = info_.sensors[1].parameters.at("enabled") == "true";
  ft_parameters_.update_rate = std::stoul(info_.sensors[1].parameters.at("update_rate"));
  ft_parameters_.rt_prio = std::stoi(info_.sensors[1].parameters.at("rt_prio"));
  ft_parameters_.chain_root = info_.sensors[1].parameters.at("chain_root");
  ft_parameters_.chain_tip = info_.sensors[1].parameters.at("chain_tip");
  ft_parameters_.damping = std::stod(info_.sensors[1].parameters.at("damping"));
  ft_parameters_.force_x_th = std::stod(info_.sensors[1].parameters.at("force_x_th"));
  ft_parameters_.force_y_th = std::stod(info_.sensors[1].parameters.at("force_y_th"));
  ft_parameters_.force_z_th = std::stod(info_.sensors[1].parameters.at("force_z_th"));
  ft_parameters_.torque_x_th = std::stod(info_.sensors[1].parameters.at("torque_x_th"));
  ft_parameters_.torque_y_th = std::stod(info_.sensors[1].parameters.at("torque_y_th"));
  ft_parameters_.torque_z_th = std::stod(info_.sensors[1].parameters.at("torque_z_th"));
  if (ft_parameters_.enabled) {
    ft_estimator_impl_ptr_ = std::make_shared<lbr_fri_ros2::FTEstimatorImpl>(
        info_.original_xml, ft_parameters_.chain_root, ft_parameters_.chain_tip,
        lbr_fri_ros2::cart_array_t{
            ft_parameters_.force_x_th,
            ft_parameters_.force_y_th,
            ft_parameters_.force_z_th,
            ft_parameters_.torque_x_th,
            ft_parameters_.torque_y_th,
            ft_parameters_.torque_z_th,
        });
    ft_estimator_ptr_ = std::make_unique<lbr_fri_ros2::FTEstimator>(ft_estimator_impl_ptr_,
                                                                    ft_parameters_.update_rate);
  }

  if (!verify_number_of_joints_()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!verify_joint_command_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!verify_joint_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!verify_sensors_()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!verify_gpios_()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SystemInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // state interfaces of type double
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                                  &hw_lbr_state_.measured_joint_position[i]);

#if FRI_CLIENT_VERSION_MAJOR == 1
    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_COMMANDED_JOINT_POSITION,
                                  &hw_lbr_state_.commanded_joint_position[i]);
#endif

    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
                                  &hw_lbr_state_.measured_torque[i]);

    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_COMMANDED_TORQUE,
                                  &hw_lbr_state_.commanded_torque[i]);

    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_EXTERNAL_TORQUE,
                                  &hw_lbr_state_.external_torque[i]);

    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_IPO_JOINT_POSITION,
                                  &hw_lbr_state_.ipo_joint_position[i]);

    // additional velocity state interface
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
                                  &hw_velocity_[i]);
  }

  const auto &auxiliary_sensor = info_.sensors[0];
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_SAMPLE_TIME,
                                &hw_lbr_state_.sample_time);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_TRACKING_PERFORMANCE,
                                &hw_lbr_state_.tracking_performance);

  // state interfaces that require cast
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_SESSION_STATE, &hw_session_state_);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_CONNECTION_QUALITY,
                                &hw_connection_quality_);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_SAFETY_STATE, &hw_safety_state_);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_OPERATION_MODE, &hw_operation_mode_);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_DRIVE_STATE, &hw_drive_state_);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_CLIENT_COMMAND_MODE,
                                &hw_client_command_mode_);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_OVERLAY_TYPE, &hw_overlay_type_);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_CONTROL_MODE, &hw_control_mode_);

  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_TIME_STAMP_SEC, &hw_time_stamp_sec_);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_TIME_STAMP_NANO_SEC,
                                &hw_time_stamp_nano_sec_);

  // additional force-torque state interface (if enabled)
  if (ft_parameters_.enabled) {
    const auto &estimated_ft_sensor = info_.sensors[1];
    state_interfaces.emplace_back(estimated_ft_sensor.name, HW_IF_FORCE_X, &hw_ft_[0]);
    state_interfaces.emplace_back(estimated_ft_sensor.name, HW_IF_FORCE_Y, &hw_ft_[1]);
    state_interfaces.emplace_back(estimated_ft_sensor.name, HW_IF_FORCE_Z, &hw_ft_[2]);
    state_interfaces.emplace_back(estimated_ft_sensor.name, HW_IF_TORQUE_X, &hw_ft_[3]);
    state_interfaces.emplace_back(estimated_ft_sensor.name, HW_IF_TORQUE_Y, &hw_ft_[4]);
    state_interfaces.emplace_back(estimated_ft_sensor.name, HW_IF_TORQUE_Z, &hw_ft_[5]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SystemInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                                    &hw_lbr_command_.joint_position[i]);

    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
                                    &hw_lbr_command_.torque[i]);
  }

  // Cartesian impedance control command interfaces
  const auto &wrench = info_.gpios[0];
  command_interfaces.emplace_back(wrench.name, HW_IF_FORCE_X, &hw_lbr_command_.wrench[0]);
  command_interfaces.emplace_back(wrench.name, HW_IF_FORCE_Y, &hw_lbr_command_.wrench[1]);
  command_interfaces.emplace_back(wrench.name, HW_IF_FORCE_Z, &hw_lbr_command_.wrench[2]);
  command_interfaces.emplace_back(wrench.name, HW_IF_TORQUE_X, &hw_lbr_command_.wrench[3]);
  command_interfaces.emplace_back(wrench.name, HW_IF_TORQUE_Y, &hw_lbr_command_.wrench[4]);
  command_interfaces.emplace_back(wrench.name, HW_IF_TORQUE_Z, &hw_lbr_command_.wrench[5]);
  return command_interfaces;
}

hardware_interface::return_type
SystemInterface::prepare_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/,
                                             const std::vector<std::string> & /*stop_interfaces*/) {
  return hardware_interface::return_type::OK;
}

controller_interface::CallbackReturn SystemInterface::on_activate(const rclcpp_lifecycle::State &) {
  if (!async_client_ptr_) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME), lbr_fri_ros2::ColorScheme::ERROR
                                                             << "AsyncClient not configured"
                                                             << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!app_ptr_->open_udp_socket(parameters_.port_id, parameters_.remote_host)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  app_ptr_->run_async(parameters_.rt_prio);
  int attempt = 0;
  while (!async_client_ptr_->get_state_interface()->is_initialized()) {
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger(LOGGER_NAME),
        "Awaiting robot heartbeat. Attempt "
            << ++attempt << ", remote_host '" << lbr_fri_ros2::ColorScheme::OKBLUE
            << lbr_fri_ros2::ColorScheme::BOLD
            << (parameters_.remote_host == NULL ? "INADDR_ANY" : parameters_.remote_host)
            << lbr_fri_ros2::ColorScheme::ENDC << "', port_id '"
            << lbr_fri_ros2::ColorScheme::OKBLUE << lbr_fri_ros2::ColorScheme::BOLD
            << parameters_.port_id << "'" << lbr_fri_ros2::ColorScheme::ENDC);
    if (!rclcpp::ok()) {
      return controller_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME), lbr_fri_ros2::ColorScheme::OKGREEN
                                                          << "Robot connected"
                                                          << lbr_fri_ros2::ColorScheme::ENDC);
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME),
                     "Control mode '"
                         << lbr_fri_ros2::EnumMaps::control_mode_map(
                                async_client_ptr_->get_state_interface()->get_state().control_mode)
                                .c_str()
                         << "'");
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Sample time %.3f s / %.1f Hz",
              async_client_ptr_->get_state_interface()->get_state().sample_time,
              1. / async_client_ptr_->get_state_interface()->get_state().sample_time);
  while (!(async_client_ptr_->get_state_interface()->get_state().session_state >=
           KUKA::FRI::ESessionState::COMMANDING_WAIT)) {
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger(LOGGER_NAME),
        "Awaiting '" << lbr_fri_ros2::ColorScheme::BOLD << lbr_fri_ros2::ColorScheme::OKBLUE
                     << lbr_fri_ros2::EnumMaps::session_state_map(
                            KUKA::FRI::ESessionState::COMMANDING_WAIT)
                     << lbr_fri_ros2::ColorScheme::ENDC << "' state. Current state '"
                     << lbr_fri_ros2::ColorScheme::BOLD << lbr_fri_ros2::ColorScheme::OKBLUE
                     << lbr_fri_ros2::EnumMaps::session_state_map(
                            async_client_ptr_->get_state_interface()->get_state().session_state)
                     << lbr_fri_ros2::ColorScheme::ENDC << "'.");
    if (!rclcpp::ok()) {
      return controller_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // ft sensor
  if (!ft_estimator_ptr_ && ft_parameters_.enabled) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Failed to instantiate FTEstimator despite user request."
                            << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (ft_estimator_ptr_) {
    ft_estimator_ptr_->run_async(ft_parameters_.rt_prio);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SystemInterface::on_deactivate(const rclcpp_lifecycle::State &) {
  app_ptr_->request_stop();
  app_ptr_->close_udp_socket();
  if (ft_estimator_ptr_) {
    ft_estimator_ptr_->request_stop();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SystemInterface::read(const rclcpp::Time & /*time*/,
                                                      const rclcpp::Duration &period) {
  if (!async_client_ptr_->get_state_interface()->is_initialized()) {
    return hardware_interface::return_type::OK;
  }

  hw_lbr_state_ = async_client_ptr_->get_state_interface()->get_state();

  if (period.seconds() - hw_lbr_state_.sample_time * 0.2 > hw_lbr_state_.sample_time) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger(LOGGER_NAME),
                       lbr_fri_ros2::ColorScheme::WARNING
                           << "Increase update_rate parameter for controller_manager to "
                           << std::to_string(static_cast<int>(1. / hw_lbr_state_.sample_time))
                           << " Hz or more" << lbr_fri_ros2::ColorScheme::ENDC);
  }

  // exit once robot exits COMMANDING_ACTIVE (for safety)
  if (exit_commanding_active_(static_cast<KUKA::FRI::ESessionState>(hw_session_state_),
                              static_cast<KUKA::FRI::ESessionState>(hw_lbr_state_.session_state))) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "LBR left COMMANDING_ACTIVE. Please re-run lbr_bringup"
                            << lbr_fri_ros2::ColorScheme::ENDC);
    app_ptr_->request_stop();
    app_ptr_->close_udp_socket();
    ft_estimator_ptr_->request_stop();
    return hardware_interface::return_type::ERROR;
  }

  // state interfaces that require cast
  hw_session_state_ = static_cast<double>(hw_lbr_state_.session_state);
  hw_connection_quality_ = static_cast<double>(hw_lbr_state_.connection_quality);
  hw_safety_state_ = static_cast<double>(hw_lbr_state_.safety_state);
  hw_operation_mode_ = static_cast<double>(hw_lbr_state_.operation_mode);
  hw_drive_state_ = static_cast<double>(hw_lbr_state_.drive_state);
  hw_client_command_mode_ = static_cast<double>(hw_lbr_state_.client_command_mode);
  hw_overlay_type_ = static_cast<double>(hw_lbr_state_.overlay_type);
  hw_control_mode_ = static_cast<double>(hw_lbr_state_.control_mode);
  hw_time_stamp_sec_ = static_cast<double>(hw_lbr_state_.time_stamp_sec);
  hw_time_stamp_nano_sec_ = static_cast<double>(hw_lbr_state_.time_stamp_nano_sec);

  // additional velocity state interface
  compute_hw_velocity_();
  update_last_hw_states_();

  // additional force-torque state interface
  if (ft_parameters_.enabled) {
    // note that (if enabled) the computation is performed asynchronously to not block the main
    // thread
    ft_estimator_impl_ptr_->set_q(hw_lbr_state_.measured_joint_position);
    ft_estimator_impl_ptr_->set_tau_ext(hw_lbr_state_.external_torque);
    ft_estimator_impl_ptr_->get_f_ext_tf(hw_ft_);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SystemInterface::write(const rclcpp::Time & /*time*/,
                                                       const rclcpp::Duration & /*period*/) {
  if (hw_session_state_ != KUKA::FRI::COMMANDING_ACTIVE) {
    return hardware_interface::return_type::OK;
  }
  async_client_ptr_->get_command_interface()->buffer_command_target(hw_lbr_command_);
  return hardware_interface::return_type::OK;
}

bool SystemInterface::parse_parameters_(const hardware_interface::HardwareInfo &system_info) {
  try {
    parameters_.fri_client_sdk_major_version =
        std::stoul(system_info.hardware_parameters.at("fri_client_sdk_major_version"));
    parameters_.fri_client_sdk_minor_version =
        std::stoul(system_info.hardware_parameters.at("fri_client_sdk_minor_version"));
    if (parameters_.fri_client_sdk_major_version != FRI_CLIENT_VERSION_MAJOR) {
      RCLCPP_ERROR_STREAM(
          rclcpp::get_logger(LOGGER_NAME),
          lbr_fri_ros2::ColorScheme::ERROR
              << "Expected FRI client SDK version '" << FRI_CLIENT_VERSION_MAJOR << "', got '"
              << std::to_string(parameters_.fri_client_sdk_major_version)
              << "'. Update lbr_system_parameters.yaml or compile against correct FRI version."
              << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
    std::string client_command_mode = system_info.hardware_parameters.at("client_command_mode");
    if (client_command_mode == "position") {
#if FRI_CLIENT_VERSION_MAJOR == 1
      parameters_.client_command_mode = KUKA::FRI::EClientCommandMode::POSITION;
#endif
#if FRI_CLIENT_VERSION_MAJOR >= 2
      parameters_.client_command_mode = KUKA::FRI::EClientCommandMode::JOINT_POSITION;
#endif
    } else if (client_command_mode == "torque") {
      parameters_.client_command_mode = KUKA::FRI::EClientCommandMode::TORQUE;
    } else if (client_command_mode == "wrench") {
      parameters_.client_command_mode = KUKA::FRI::EClientCommandMode::WRENCH;
    } else {
      RCLCPP_ERROR_STREAM(
          rclcpp::get_logger(LOGGER_NAME),
          lbr_fri_ros2::ColorScheme::ERROR
              << "Expected client_command_mode 'position', 'torque' or 'wrench', got '"
              << lbr_fri_ros2::ColorScheme::BOLD << parameters_.client_command_mode << "'"
              << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
    parameters_.port_id = std::stoul(info_.hardware_parameters["port_id"]);
    if (parameters_.port_id < 30200 || parameters_.port_id > 30209) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          lbr_fri_ros2::ColorScheme::ERROR
                              << "Expected port_id in [30200, 30209], got '"
                              << lbr_fri_ros2::ColorScheme::BOLD << parameters_.port_id << "'"
                              << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
    info_.hardware_parameters["remote_host"] == "INADDR_ANY"
        ? parameters_.remote_host = NULL
        : parameters_.remote_host = info_.hardware_parameters["remote_host"].c_str();
    parameters_.rt_prio = std::stoul(info_.hardware_parameters["rt_prio"]);
    std::transform(info_.hardware_parameters["open_loop"].begin(),
                   info_.hardware_parameters["open_loop"].end(),
                   info_.hardware_parameters["open_loop"].begin(),
                   ::tolower); // convert to lower case
    parameters_.open_loop = info_.hardware_parameters["open_loop"] == "true";
    parameters_.joint_position_tau = std::stod(info_.hardware_parameters["joint_position_tau"]);
    parameters_.command_guard_variant = system_info.hardware_parameters.at("command_guard_variant");
    parameters_.external_torque_tau = std::stod(info_.hardware_parameters["external_torque_tau"]);
    parameters_.measured_torque_tau = std::stod(info_.hardware_parameters["measured_torque_tau"]);
  } catch (const std::out_of_range &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Failed to parse hardware parameters with: " << e.what()
                            << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  return true;
}

void SystemInterface::nan_command_interfaces_() {
  hw_lbr_command_.joint_position.fill(std::numeric_limits<double>::quiet_NaN());
  hw_lbr_command_.torque.fill(std::numeric_limits<double>::quiet_NaN());
  hw_lbr_command_.wrench.fill(std::numeric_limits<double>::quiet_NaN());
}

void SystemInterface::nan_state_interfaces_() {
  // state interfaces of type double
  hw_lbr_state_.measured_joint_position.fill(std::numeric_limits<double>::quiet_NaN());
#if FRI_CLIENT_VERSION_MAJOR == 1
  hw_lbr_state_.commanded_joint_position.fill(std::numeric_limits<double>::quiet_NaN());
#endif
  hw_lbr_state_.measured_torque.fill(std::numeric_limits<double>::quiet_NaN());
  hw_lbr_state_.commanded_torque.fill(std::numeric_limits<double>::quiet_NaN());
  hw_lbr_state_.external_torque.fill(std::numeric_limits<double>::quiet_NaN());
  hw_lbr_state_.ipo_joint_position.fill(std::numeric_limits<double>::quiet_NaN());
  hw_lbr_state_.sample_time = std::numeric_limits<double>::quiet_NaN();
  hw_lbr_state_.tracking_performance = std::numeric_limits<double>::quiet_NaN();

  // state interfaces that require cast
  hw_session_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_connection_quality_ = std::numeric_limits<double>::quiet_NaN();
  hw_safety_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_operation_mode_ = std::numeric_limits<double>::quiet_NaN();
  hw_drive_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_client_command_mode_ = std::numeric_limits<double>::quiet_NaN();
  hw_overlay_type_ = std::numeric_limits<double>::quiet_NaN();
  hw_control_mode_ = std::numeric_limits<double>::quiet_NaN();
  hw_time_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
  hw_time_stamp_nano_sec_ = std::numeric_limits<double>::quiet_NaN();

  // additional velocity state interface
  hw_velocity_.fill(std::numeric_limits<double>::quiet_NaN());

  // additional force-torque state interface
  hw_ft_.fill(std::numeric_limits<double>::quiet_NaN());
}

bool SystemInterface::verify_number_of_joints_() {
  if (info_.joints.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Expected '" << lbr_fri_ros2::N_JNTS << "' joints in URDF, got '"
                            << info_.joints.size() << "'" << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  return true;
}

bool SystemInterface::verify_joint_command_interfaces_() {
  // check command interfaces
  for (auto &joint : info_.joints) {
    if (joint.command_interfaces.size() != LBR_FRI_COMMAND_INTERFACE_SIZE) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          lbr_fri_ros2::ColorScheme::ERROR
                              << "Joint '" << joint.name.c_str()
                              << "' received invalid number of command interfaces. Received '"
                              << joint.command_interfaces.size() << "', expected "
                              << static_cast<int>(LBR_FRI_COMMAND_INTERFACE_SIZE)
                              << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
    for (auto &ci : joint.command_interfaces) {
      if (ci.name != hardware_interface::HW_IF_POSITION &&
          ci.name != hardware_interface::HW_IF_EFFORT) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                            lbr_fri_ros2::ColorScheme::ERROR
                                << "Joint '" << joint.name.c_str()
                                << "' received invalid command interface '" << ci.name.c_str()
                                << "'. Expected '" << hardware_interface::HW_IF_POSITION << "' or '"
                                << hardware_interface::HW_IF_EFFORT << "'"
                                << lbr_fri_ros2::ColorScheme::ENDC);
        return false;
      }
    }
  }
  return true;
}

bool SystemInterface::verify_joint_state_interfaces_() {
  // check state interfaces
  for (auto &joint : info_.joints) {
    if (joint.state_interfaces.size() != LBR_FRI_STATE_INTERFACE_SIZE) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          lbr_fri_ros2::ColorScheme::ERROR
                              << "Joint '" << joint.name.c_str()
                              << "' received invalid number of state interfaces. Received '"
                              << joint.state_interfaces.size() << "', expected '"
                              << static_cast<int>(LBR_FRI_STATE_INTERFACE_SIZE) << "'"
                              << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
    for (auto &si : joint.state_interfaces) {
      if (si.name != hardware_interface::HW_IF_POSITION &&
          si.name != HW_IF_COMMANDED_JOINT_POSITION &&
          si.name != hardware_interface::HW_IF_EFFORT && si.name != HW_IF_COMMANDED_TORQUE &&
          si.name != HW_IF_EXTERNAL_TORQUE && si.name != HW_IF_IPO_JOINT_POSITION &&
          si.name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger(LOGGER_NAME),
            lbr_fri_ros2::ColorScheme::ERROR
                << "Joint '" << joint.name.c_str() << "' received invalid state interface '"
                << si.name.c_str() << "'. Expected one of '" << hardware_interface::HW_IF_POSITION
                << "', '" << HW_IF_COMMANDED_JOINT_POSITION << "', '"
                << hardware_interface::HW_IF_EFFORT << "', '" << HW_IF_COMMANDED_TORQUE << "', '"
                << HW_IF_EXTERNAL_TORQUE << "', '" << HW_IF_IPO_JOINT_POSITION << "' or '"
                << hardware_interface::HW_IF_VELOCITY << "'" << lbr_fri_ros2::ColorScheme::ENDC);
        return false;
      }
    }
  }
  return true;
}

bool SystemInterface::verify_sensors_() {
  // check lbr specific state interfaces
  if (info_.sensors.size() != LBR_FRI_SENSORS) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Expected '" << static_cast<int>(LBR_FRI_SENSORS)
                            << "' sensors, got '" << info_.sensors.size() << "'"
                            << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  if (!verify_auxiliary_sensor_()) {
    return false;
  }
  if (ft_parameters_.enabled) {
    if (!verify_estimated_ft_sensor_()) {
      return false;
    }
  }
  return true;
}

bool SystemInterface::verify_auxiliary_sensor_() {
  // check all interfaces are defined in lbr_system_interface.xacro (located in
  // lbr_description/ros2_control/lbr_system_interface.xacro)
  const auto &auxiliary_sensor = info_.sensors[0];
  if (auxiliary_sensor.name != HW_IF_AUXILIARY_PREFIX) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Sensor '" << auxiliary_sensor.name.c_str()
                            << "' received invalid name. Expected '" << HW_IF_AUXILIARY_PREFIX
                            << "'" << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  if (auxiliary_sensor.state_interfaces.size() != AUXILIARY_SENSOR_SIZE) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Sensor '" << auxiliary_sensor.name.c_str()
                            << "' received invalid number of state interfaces." << " Received '"
                            << auxiliary_sensor.state_interfaces.size() << "', expected '"
                            << static_cast<int>(AUXILIARY_SENSOR_SIZE) << "'"
                            << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  // check only valid interfaces are defined
  for (const auto &si : auxiliary_sensor.state_interfaces) {
    if (si.name != HW_IF_SAMPLE_TIME && si.name != HW_IF_SESSION_STATE &&
        si.name != HW_IF_CONNECTION_QUALITY && si.name != HW_IF_SAFETY_STATE &&
        si.name != HW_IF_OPERATION_MODE && si.name != HW_IF_DRIVE_STATE &&
        si.name != HW_IF_CLIENT_COMMAND_MODE && si.name != HW_IF_OVERLAY_TYPE &&
        si.name != HW_IF_CONTROL_MODE && si.name != HW_IF_TIME_STAMP_SEC &&
        si.name != HW_IF_TIME_STAMP_NANO_SEC && si.name != HW_IF_COMMANDED_JOINT_POSITION &&
        si.name != HW_IF_COMMANDED_TORQUE && si.name != HW_IF_EXTERNAL_TORQUE &&
        si.name != HW_IF_IPO_JOINT_POSITION && si.name != HW_IF_TRACKING_PERFORMANCE) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          lbr_fri_ros2::ColorScheme::ERROR
                              << "Sensor '" << auxiliary_sensor.name.c_str()
                              << "' received invalid state interface '" << si.name.c_str() << "'"
                              << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
  }
  return true;
}

bool SystemInterface::verify_estimated_ft_sensor_() {
  const auto &estimated_ft_sensor = info_.sensors[1];
  if (estimated_ft_sensor.name != HW_IF_ESTIMATED_FT_PREFIX) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Sensor '" << estimated_ft_sensor.name.c_str()
                            << "' received invalid name. Expected '" << HW_IF_ESTIMATED_FT_PREFIX
                            << "'" << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  if (estimated_ft_sensor.state_interfaces.size() != ESTIMATED_FT_SENSOR_SIZE) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Sensor '" << estimated_ft_sensor.name.c_str()
                            << "' received invalid number of state interfaces. Received '"
                            << estimated_ft_sensor.state_interfaces.size() << "', expected '"
                            << static_cast<int>(ESTIMATED_FT_SENSOR_SIZE) << "'"
                            << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  // check only valid interfaces are defined
  for (const auto &si : estimated_ft_sensor.state_interfaces) {
    if (si.name != HW_IF_FORCE_X && si.name != HW_IF_FORCE_Y && si.name != HW_IF_FORCE_Z &&
        si.name != HW_IF_TORQUE_X && si.name != HW_IF_TORQUE_Y && si.name != HW_IF_TORQUE_Z) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          lbr_fri_ros2::ColorScheme::ERROR
                              << "Sensor '" << estimated_ft_sensor.name.c_str()
                              << "' received invalid state interface '" << si.name.c_str() << "'"
                              << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
  }
  return true;
}

bool SystemInterface::verify_gpios_() {
  if (info_.gpios.size() != GPIO_SIZE) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Expected '" << static_cast<int>(GPIO_SIZE) << "' GPIOs, got '"
                            << info_.gpios.size() << "'" << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  if (info_.gpios[0].name != HW_IF_WRENCH_PREFIX) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        lbr_fri_ros2::ColorScheme::ERROR << "GPIO '" << info_.gpios[0].name.c_str()
                                                         << "' received invalid name. Expected '"
                                                         << HW_IF_WRENCH_PREFIX << "'"
                                                         << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  if (info_.gpios[0].command_interfaces.size() != hw_lbr_command_.wrench.size()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "GPIO '" << info_.gpios[0].name.c_str()
                            << "' received invalid number of command interfaces. Received '"
                            << info_.gpios[0].command_interfaces.size() << "', expected '"
                            << hw_lbr_command_.wrench.size() << "'"
                            << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  return true;
}

bool SystemInterface::exit_commanding_active_(
    const KUKA::FRI::ESessionState &previous_session_state,
    const KUKA::FRI::ESessionState &session_state) {
  if (previous_session_state == KUKA::FRI::ESessionState::COMMANDING_ACTIVE &&
      previous_session_state != session_state) {
    return true;
  }
  return false;
}

double SystemInterface::time_stamps_to_sec_(const double &sec, const double &nano_sec) const {
  return sec + nano_sec / 1.e9;
}

void SystemInterface::nan_last_hw_states_() {
  last_hw_measured_joint_position_.fill(std::numeric_limits<double>::quiet_NaN());
  last_hw_time_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
  last_hw_time_stamp_nano_sec_ = std::numeric_limits<double>::quiet_NaN();
}

void SystemInterface::update_last_hw_states_() {
  last_hw_measured_joint_position_ = hw_lbr_state_.measured_joint_position;
  last_hw_time_stamp_sec_ = hw_time_stamp_sec_;
  last_hw_time_stamp_nano_sec_ = hw_time_stamp_nano_sec_;
}

void SystemInterface::compute_hw_velocity_() {
  // state uninitialized
  if (std::isnan(last_hw_time_stamp_nano_sec_) || std::isnan(last_hw_measured_joint_position_[0])) {
    return;
  }

  // state wasn't updated
  if (last_hw_time_stamp_sec_ == hw_time_stamp_sec_ &&
      last_hw_time_stamp_nano_sec_ == hw_time_stamp_nano_sec_) {
    return;
  }

  double dt = time_stamps_to_sec_(hw_time_stamp_sec_, hw_time_stamp_nano_sec_) -
              time_stamps_to_sec_(last_hw_time_stamp_sec_, last_hw_time_stamp_nano_sec_);
  std::size_t i = 0;
  std::for_each(hw_velocity_.begin(), hw_velocity_.end(), [&](double &v) {
    v = (hw_lbr_state_.measured_joint_position[i] - last_hw_measured_joint_position_[i]) / dt;
    ++i;
  });
}

} // namespace lbr_ros2_control

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::SystemInterface, hardware_interface::SystemInterface)
