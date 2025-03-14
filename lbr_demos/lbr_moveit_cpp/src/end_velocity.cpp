#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

class EndEffectorVelocityNode : public rclcpp::Node
{
public:
  EndEffectorVelocityNode() : Node("ee_velocity_node")
  {
    // 初始化发布器和订阅器
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("ee_velocity", 10);
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/lbr/joint_states", 10,
      std::bind(&EndEffectorVelocityNode::jointStateCallback, this, std::placeholders::_1));

    // 延迟初始化 MoveIt 相关对象，确保节点完全初始化
    init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { this->init(); });

    // 定时器每秒生成一次日志输出
    log_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { this->logVelocity(); });
  }

private:
  void init()
  {
    // 确保只初始化一次
    if (initialized_) return;
    
    try {
      // 加载机器人模型
      robot_model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(
        this->shared_from_this(), "robot_description");
      robot_model_ = robot_model_loader_->getModel();
      if (!robot_model_) {
        RCLCPP_ERROR(this->get_logger(), "无法加载机器人模型");
        return;
      }

      // 初始化机器人状态
      robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
      robot_state_->setToDefaultValues();

      // 获取关节组信息
      joint_group_ = robot_model_->getJointModelGroup("arm");  // 修改为您的规划组名称
      if (!joint_group_) {
        RCLCPP_ERROR(this->get_logger(), "无法获取关节组 'arm'");
        return;
      }

      // 获取末端执行器 Link 名称
      ee_link_ = "lbr_link_7";  // 修改为您的末端 Link 名称
      if (!robot_state_->knowsFrameTransform(ee_link_)) {
        RCLCPP_ERROR(this->get_logger(), "未知的末端 Link: %s", ee_link_.c_str());
        return;
      }

      // 获取关节名称列表，用于后续匹配顺序
      joint_names_ = joint_group_->getVariableNames();
      RCLCPP_INFO(this->get_logger(), "模型关节顺序:");
      for (const auto& name : joint_names_) {
        RCLCPP_INFO(this->get_logger(), "- %s", name.c_str());
      }

      initialized_ = true;
      init_timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "初始化成功");

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "初始化异常: %s", e.what());
    }
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // 如果机器人状态没有初始化，则进行提示
    if (!initialized_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "未初始化完成");
      return;
    }

    if (msg->name.size() != msg->velocity.size()) {
      RCLCPP_ERROR(this->get_logger(), "关节名称和关节速度的数量不匹配");
      return;
    }

    // 将接收到的关节状态按模型顺序排序
    std::vector<double> ordered_velocities(joint_names_.size(), 0.0);
    std::vector<double> ordered_positions(joint_names_.size(), 0.0);

    // 校验关节数据
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it == msg->name.end()) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "未找到关节: %s", joint_names_[i].c_str());
        return;
      }
      size_t msg_index = std::distance(msg->name.begin(), it);
      ordered_velocities[i] = msg->velocity[msg_index];
      ordered_positions[i] = msg->position[msg_index];
    }

    // 更新机器人状态
    try {
      robot_state_->setJointGroupPositions(joint_group_, ordered_positions);
      robot_state_->setJointGroupVelocities(joint_group_, ordered_velocities);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "更新机器人状态失败: %s", e.what());
      return;
    }

    // 计算雅可比矩阵
    Eigen::Vector3d reference_point(0.0, 0.0, 0.0); // 末端坐标系原点
    Eigen::MatrixXd jacobian;
    try {
      robot_state_->getJacobian(
        joint_group_,
        robot_state_->getLinkModel(ee_link_),
        reference_point,
        jacobian);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "计算雅可比矩阵失败: %s", e.what());
      return;
    }

    // 提取关节速度
    Eigen::VectorXd joint_velocities(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joint_velocities[i] = ordered_velocities[i];
    }

    // 计算末端速度
    Eigen::VectorXd ee_velocity = jacobian * joint_velocities;
    linear_velocity_ = ee_velocity.head<3>();
    angular_velocity_ = ee_velocity.tail<3>();

    // 转换为 Twist 消息并发布
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = linear_velocity_.x();
    twist_msg.linear.y = linear_velocity_.y();
    twist_msg.linear.z = linear_velocity_.z();
    twist_msg.angular.x = angular_velocity_.x();
    twist_msg.angular.y = angular_velocity_.y();
    twist_msg.angular.z = angular_velocity_.z();

    try {
      velocity_pub_->publish(twist_msg);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "发布消息失败: %s", e.what());
    }
  }

  // 每秒输出线速度和角速度
  void logVelocity()
  {
    if (initialized_) {
      RCLCPP_INFO(this->get_logger(),
        "线速度: [%.3f, %.3f, %.3f] 角速度: [%.3f, %.3f, %.3f]",
        linear_velocity_.x(), linear_velocity_.y(), linear_velocity_.z(),
        angular_velocity_.x(), angular_velocity_.y(), angular_velocity_.z());
    }
  }

  // MoveIt 相关
  std::unique_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  moveit::core::JointModelGroup* joint_group_ = nullptr;
  std::string ee_link_;
  std::vector<std::string> joint_names_;

  // 末端速度数据
  Eigen::Vector3d linear_velocity_;
  Eigen::Vector3d angular_velocity_;

  // ROS 通信
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr log_timer_; // 每秒输出日志
  bool initialized_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EndEffectorVelocityNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
