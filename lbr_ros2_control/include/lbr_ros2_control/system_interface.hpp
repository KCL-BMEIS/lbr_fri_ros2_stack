#ifndef LBR_ROS2_CONTROL__SYSTEM_INTERFACE_HPP_
#define LBR_ROS2_CONTROL__SYSTEM_INTERFACE_HPP_

#include <algorithm>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "friClientVersion.h"
#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_command.hpp"
#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/app.hpp"
#include "lbr_fri_ros2/async_client.hpp"
#include "lbr_fri_ros2/command_guard.hpp"
#include "lbr_fri_ros2/filters.hpp"
#include "lbr_fri_ros2/formatting.hpp"
#include "lbr_fri_ros2/ft_estimator.hpp"
#include "lbr_fri_ros2/interfaces/state.hpp"
#include "lbr_ros2_control/system_interface_type_values.hpp"

namespace lbr_ros2_control {
struct SystemInterfaceParameters {
  uint8_t fri_client_sdk_major_version{1};
  uint8_t fri_client_sdk_minor_version{15};
#if FRI_CLIENT_VERSION_MAJOR == 1
  KUKA::FRI::EClientCommandMode client_command_mode{KUKA::FRI::EClientCommandMode::POSITION};
#endif
#if FRI_CLIENT_VERSION_MAJOR >= 2
  KUKA::FRI::EClientCommandMode client_command_mode{KUKA::FRI::EClientCommandMode::JOINT_POSITION};
#endif
  int32_t port_id{30200};
  const char *remote_host{nullptr};
  int32_t rt_prio{80};
  bool open_loop{true};
  double pid_p{0.0};
  double pid_i{0.0};
  double pid_d{0.0};
  double pid_i_max{0.0};
  double pid_i_min{0.0};
  double pid_antiwindup{0.0};
  std::string command_guard_variant{"default"};
  double external_torque_cutoff_frequency{10.0};
  double measured_torque_cutoff_frequency{10.0};
};

struct EstimatedFTSensorParameters {
  std::string chain_root{"base_link"};
  std::string chain_tip{"link_tool"};
  double damping{0.2};
  double force_x_th{2.0};
  double force_y_th{2.0};
  double force_z_th{2.0};
  double torque_x_th{0.5};
  double torque_y_th{0.5};
  double torque_z_th{0.5};
};

class SystemInterface : public hardware_interface::SystemInterface {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_ros2_control::SystemInterface";

#if FRI_CLIENT_VERSION_MAJOR == 1
  static constexpr uint8_t LBR_FRI_STATE_INTERFACE_SIZE = 7;
#endif
#if FRI_CLIENT_VERSION_MAJOR >= 2
  static constexpr uint8_t LBR_FRI_STATE_INTERFACE_SIZE = 6;
#endif
  static constexpr uint8_t LBR_FRI_COMMAND_INTERFACE_SIZE = 2;
  static constexpr uint8_t LBR_FRI_SENSORS = 2;
  static constexpr uint8_t AUXILIARY_SENSOR_SIZE = 12;
  static constexpr uint8_t ESTIMATED_FT_SENSOR_SIZE = 6;
  static constexpr uint8_t GPIO_SIZE = 1;

public:
  SystemInterface() = default;

  // hardware interface
  controller_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces) override; // not supported in FRI

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) override;

protected:
  // setup
  bool parse_parameters_(const hardware_interface::HardwareInfo &info);
  void nan_command_interfaces_();
  void nan_state_interfaces_();
  bool verify_number_of_joints_();
  bool verify_joint_command_interfaces_();
  bool verify_joint_state_interfaces_();
  bool verify_sensors_();
  bool verify_auxiliary_sensor_();
  bool verify_estimated_ft_sensor_();
  bool verify_gpios_();

  // monitor end of commanding active
  bool exit_commanding_active_(const KUKA::FRI::ESessionState &previous_session_state,
                               const KUKA::FRI::ESessionState &session_state);

  // robot parameters
  SystemInterfaceParameters parameters_;
  EstimatedFTSensorParameters ft_parameters_;

  // robot driver
  std::shared_ptr<lbr_fri_ros2::AsyncClient> async_client_ptr_;
  std::unique_ptr<lbr_fri_ros2::App> app_ptr_;

  // exposed state interfaces (ideally these are taken from async_client_ptr_ but
  // ros2_control ReadOnlyHandle does not allow for const pointers, refer
  // https://github.com/ros-controls/ros2_control/issues/1196)
  lbr_fri_idl::msg::LBRState hw_lbr_state_;

  // exposed state interfaces that require casting
  double hw_session_state_;
  double hw_connection_quality_;
  double hw_safety_state_;
  double hw_operation_mode_;
  double hw_drive_state_;
  double hw_client_command_mode_;
  double hw_overlay_type_;
  double hw_control_mode_;
  double hw_time_stamp_sec_;
  double hw_time_stamp_nano_sec_;

  // additional velocity state interface
  lbr_fri_idl::msg::LBRState::_measured_joint_position_type last_hw_measured_joint_position_;
  double last_hw_time_stamp_sec_;
  double last_hw_time_stamp_nano_sec_;
  lbr_fri_idl::msg::LBRState::_measured_joint_position_type hw_velocity_;

  // compute velocity for state interface
  double time_stamps_to_sec_(const double &sec, const double &nano_sec) const;
  void nan_last_hw_states_();
  void update_last_hw_states_();
  void compute_hw_velocity_();

  // additional force-torque state interface
  lbr_fri_ros2::FTEstimator::cart_array_t hw_ft_;
  std::unique_ptr<lbr_fri_ros2::FTEstimator> ft_estimator_ptr_;

  // exposed command interfaces
  lbr_fri_idl::msg::LBRCommand hw_lbr_command_;
};
} // namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__SYSTEM_INTERFACE_HPP_
