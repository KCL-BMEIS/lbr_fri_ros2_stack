#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <mutex>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;

// 增量式PID控制器类
class IncrementalPID
{
public:
  // 构造函数：kp, ki, kd, 最大输出（限幅）
  IncrementalPID(double kp, double ki, double kd, double max_output)
  : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output),
    prev_error1_(0.0), prev_error2_(0.0), output_(0.0)
  {}

  // 计算增量式PID的输出
  double compute(double error)
  {
    double delta = kp_ * (error - prev_error1_) + ki_ * error +
                   kd_ * (error - 2 * prev_error1_ + prev_error2_);
    double new_output = output_ + delta;
    if (new_output > max_output_) new_output = max_output_;
    else if (new_output < -max_output_) new_output = -max_output_;

    prev_error2_ = prev_error1_;
    prev_error1_ = error;
    output_ = new_output;
    return new_output;
  }

private:
  double kp_, ki_, kd_;
  double max_output_;
  double prev_error1_, prev_error2_;
  double output_;
};

class MotionPublisherAndSubscriber : public rclcpp::Node
{
public:
  MotionPublisherAndSubscriber()
    : Node("motion_publisher_and_subscriber"),
      max_time_(this->declare_parameter("max_time", 10.0)),
      target_linear_x_(this->declare_parameter("target_linear_x", 0.0)),
      target_linear_y_(this->declare_parameter("target_linear_y", 0.001)),
      target_linear_z_(this->declare_parameter("target_linear_z", 0.0)),
      safety_threshold_(this->declare_parameter("safety_threshold", 0.1)),
      message_timeout_(this->declare_parameter("message_timeout", 1.0))
  {
    // 从参数服务器获取各轴PID参数
    double kp_x = this->declare_parameter("kp_x", 0.0);
    double ki_x = this->declare_parameter("ki_x", 0.0);
    double kd_x = this->declare_parameter("kd_x", 0.0);
    double max_output_x = this->declare_parameter("max_output_x", 0.1);

    double kp_y = this->declare_parameter("kp_y", 1.0);
    double ki_y = this->declare_parameter("ki_y", 0.0);
    double kd_y = this->declare_parameter("kd_y", 0.0);
    double max_output_y = this->declare_parameter("max_output_y", 0.1);

    double kp_z = this->declare_parameter("kp_z", 0.0);
    double ki_z = this->declare_parameter("ki_z", 0.0);
    double kd_z = this->declare_parameter("kd_z", 0.0);
    double max_output_z = this->declare_parameter("max_output_z", 0.1);

    pid_x_ = std::make_unique<IncrementalPID>(kp_x, ki_x, kd_x, max_output_x);
    pid_y_ = std::make_unique<IncrementalPID>(kp_y, ki_y, kd_y, max_output_y);
    pid_z_ = std::make_unique<IncrementalPID>(kp_z, ki_z, kd_z, max_output_z);

    // 订阅 /ee_velocity 话题，接收实际速度数据
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/ee_velocity", 10,
        std::bind(&MotionPublisherAndSubscriber::topic_callback, this, std::placeholders::_1));

    // 发布到 /lbr/command/twist 话题
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/lbr/command/twist", 10);

    start_time_ = this->now();
    last_msg_time_ = this->now();

    // 以10ms周期创建定时器
    timer_ = this->create_wall_timer(10ms, std::bind(&MotionPublisherAndSubscriber::timer_callback, this));
  }

private:
  // 订阅回调：更新实际速度数据，并检查数据有效性
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
        !std::isfinite(msg->linear.z)) {
      RCLCPP_WARN(this->get_logger(), "Received invalid velocity data. Ignoring message.");
      return;
    }
    real_linear_x_ = msg->linear.x;
    real_linear_y_ = msg->linear.y;
    real_linear_z_ = msg->linear.z;
    last_msg_time_ = this->now();
  }

  // 定时器回调，包装了主控制逻辑并捕获异常
  void timer_callback()
  {
    try {
      publishTwist();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in timer_callback: %s", e.what());
    }
  }

  // 根据当前状态计算控制量并发布 Twist 消息
  void publishTwist()
  {
    rclcpp::Time current_time = this->now();
    double elapsed_time = (current_time - start_time_).seconds();

    // 超时退出
    if (elapsed_time >= max_time_) {
      RCLCPP_INFO(this->get_logger(), "Reached maximum runtime (%.2f s). Shutting down.", elapsed_time);
      rclcpp::shutdown();
      return;
    }

    // 检查最近消息接收超时情况
    double time_since_last_msg = (current_time - last_msg_time_).seconds();
    if (time_since_last_msg > message_timeout_) {
      RCLCPP_WARN(this->get_logger(), "No /ee_velocity message received for %.2f seconds.", time_since_last_msg);
      // 可根据需求在此处执行安全措施，例如下发停止指令
    }

    double rx, ry, rz;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      rx = real_linear_x_;
      ry = real_linear_y_;
      rz = real_linear_z_;
    }

    // 安全检查：当实际速度超出安全阈值时，立即下发停止命令并退出
    if (std::abs(rx) > safety_threshold_ ||
        std::abs(ry) > safety_threshold_ ||
        std::abs(rz) > safety_threshold_) {
      RCLCPP_ERROR(this->get_logger(),
                   "Error: Speed too large! x: %.4f, y: %.4f, z: %.4f. Initiating safe shutdown.",
                   rx, ry, rz);
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = 0.0;
      twist_msg.linear.y = 0.0;
      twist_msg.linear.z = 0.0;
      twist_msg.angular.x = 0.0;
      twist_msg.angular.y = 0.0;
      twist_msg.angular.z = 0.0;
      publisher_->publish(twist_msg);
      rclcpp::shutdown();
      return;
    }

    // 计算PID控制误差：目标（前馈） - 实际
    double error_x = target_linear_x_ - rx;
    double error_y = target_linear_y_ - ry;
    double error_z = target_linear_z_ - rz;

    // 计算增量式PID修正量
    double delta_pid_x = pid_x_->compute(error_x);
    double delta_pid_y = pid_y_->compute(error_y);
    double delta_pid_z = pid_z_->compute(error_z);

    // 前馈 + PID修正
    double control_x = target_linear_x_ + delta_pid_x;
    double control_y = target_linear_y_ + delta_pid_y;
    double control_z = target_linear_z_ + delta_pid_z;

    // 检查计算结果有效性
    if (!std::isfinite(control_x) || !std::isfinite(control_y) || !std::isfinite(control_z)) {
      RCLCPP_ERROR(this->get_logger(), "Control command contains invalid values. Stopping.");
      rclcpp::shutdown();
      return;
    }

    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = control_x;
    twist_msg.linear.y = control_y;
    twist_msg.linear.z = control_z;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;

    RCLCPP_INFO(this->get_logger(),
                "Time: %.2f s, Feedforward: [x: %.4f, y: %.4f, z: %.4f], Real Vel: [x: %.4f, y: %.4f, z: %.4f], "
                "Delta PID: [x: %.4f, y: %.4f, z: %.4f], Control: [x: %.4f, y: %.4f, z: %.4f]",
                elapsed_time,
                target_linear_x_, target_linear_y_, target_linear_z_,
                rx, ry, rz,
                delta_pid_x, delta_pid_y, delta_pid_z,
                control_x, control_y, control_z);

    publisher_->publish(twist_msg);
  }

  // 成员变量
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double max_time_;
  double target_linear_x_, target_linear_y_, target_linear_z_;
  double safety_threshold_;
  double message_timeout_;

  double real_linear_x_{0.0}, real_linear_y_{0.0}, real_linear_z_{0.0};
  rclcpp::Time start_time_;
  rclcpp::Time last_msg_time_;
  std::mutex mutex_;

  std::unique_ptr<IncrementalPID> pid_x_;
  std::unique_ptr<IncrementalPID> pid_y_;
  std::unique_ptr<IncrementalPID> pid_z_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MotionPublisherAndSubscriber>();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node is starting...");
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception caught in main: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
