#ifndef LBR_ROS2_CONTROL__SYSTEM_INTERFACE_TYPE_VALUES_HPP_
#define LBR_ROS2_CONTROL__SYSTEM_INTERFACE_TYPE_VALUES_HPP_

// see
// https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp
namespace lbr_ros2_control {
// additional LBR state interfaces, reference KUKA::FRI::LBRState
constexpr char HW_IF_SAMPLE_TIME[] = "sample_time";
constexpr char HW_IF_SESSION_STATE[] = "session_state";
constexpr char HW_IF_CONNECTION_QUALITY[] = "connection_quality";
constexpr char HW_IF_SAFETY_STATE[] = "safety_state";
constexpr char HW_IF_OPERATION_MODE[] = "operation_mode";
constexpr char HW_IF_DRIVE_STATE[] = "drive_state";
constexpr char HW_IF_CLIENT_COMMAND_MODE[] = "client_command_mode";
constexpr char HW_IF_OVERLAY_TYPE[] = "overlay_type";
constexpr char HW_IF_CONTROL_MODE[] = "control_mode";

constexpr char HW_IF_TIME_STAMP_SEC[] = "time_stamp_sec";
constexpr char HW_IF_TIME_STAMP_NANO_SEC[] = "time_stamp_nano_sec";

constexpr char HW_IF_COMMANDED_JOINT_POSITION[] = "commanded_joint_position";
constexpr char HW_IF_COMMANDED_TORQUE[] = "commanded_torque";

constexpr char HW_IF_EXTERNAL_TORQUE[] = "external_torque";

constexpr char HW_IF_IPO_JOINT_POSITION[] = "ipo_joint_position";
constexpr char HW_IF_TRACKING_PERFORMANCE[] = "tracking_performance";

// additional force-torque command and state interfaces
constexpr char HW_IF_FORCE_X[] = "force.x";
constexpr char HW_IF_FORCE_Y[] = "force.y";
constexpr char HW_IF_FORCE_Z[] = "force.z";
constexpr char HW_IF_TORQUE_X[] = "torque.x";
constexpr char HW_IF_TORQUE_Y[] = "torque.y";
constexpr char HW_IF_TORQUE_Z[] = "torque.z";

// additional LBR command interfaces, reference KUKA::FRI::LBRCommand
constexpr char HW_IF_WRENCH_PREFIX[] = "wrench";
constexpr char HW_IF_AUXILIARY_PREFIX[] = "auxiliary_sensor";
constexpr char HW_IF_ESTIMATED_FT_PREFIX[] = "estimated_ft_sensor";

// 新增的GPIO相关接口定义：
constexpr char HW_IF_GPIO_PREFIX[] = "gpio";         // GPIO接口的前缀
constexpr char HW_IF_ANALOG_SPEED[] = "analog_speed";  // 对应模拟量的速度接口
constexpr char HW_IF_BOOLEAN_START[] = "boolean_start";// 对应布尔量的启动接口
constexpr char HW_IF_ANALOG[] = "analog";              // 通用的模拟量接口
} // namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__SYSTEM_INTERFACE_TYPE_VALUES_HPP_
