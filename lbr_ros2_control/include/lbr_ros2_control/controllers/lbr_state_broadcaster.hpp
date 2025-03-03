#ifndef LBR_ROS2_CONTROL__LBR_STATE_BROADCASTER_HPP_
#define LBR_ROS2_CONTROL__LBR_STATE_BROADCASTER_HPP_

#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"

#include "friClientIf.h"
#include "friClientVersion.h"
#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/types.hpp"
#include "lbr_ros2_control/system_interface_type_values.hpp"

namespace lbr_ros2_control {
class LBRStateBroadcaster : public controller_interface::ControllerInterface {
public:
  LBRStateBroadcaster() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::return_type update(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

protected:
  void init_state_interface_map_();
  void init_state_msg_();
  void configure_joint_names_();

  lbr_fri_ros2::jnt_name_array_t joint_names_;
  std::unordered_map<std::string, std::unordered_map<std::string, double>> state_interface_map_;

  rclcpp::Publisher<lbr_fri_idl::msg::LBRState>::SharedPtr state_publisher_ptr_;
  std::shared_ptr<realtime_tools::RealtimePublisher<lbr_fri_idl::msg::LBRState>>
      rt_state_publisher_ptr_;
};
} // namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__LBR_STATE_BROADCASTER_HPP_
