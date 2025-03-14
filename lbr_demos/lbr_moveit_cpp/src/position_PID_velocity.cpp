#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <chrono>

// 简单的 PID 控制器类
class PID
{
public:
  PID(double kp, double ki, double kd)
  : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

  // 根据误差和时间步长计算控制输出
  double compute(double error, double dt)
  {
    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    prev_error_ = error;
    return output;
  }

private:
  double kp_, ki_, kd_;
  double prev_error_;
  double integral_;
};

class MotionPublisherAndSubscriber : public rclcpp::Node
{
public:
  MotionPublisherAndSubscriber()
      : Node("motion_publisher_and_subscriber"),
        max_time_(10.0),   // 最大运行时间 10 秒
        real_linear_x_(0.0),
        real_linear_y_(0.0),
        real_linear_z_(0.0),
        target_linear_x_(0.005),
        target_linear_y_(0.0),
        target_linear_z_(0.0),
        pid_x_(1.0, 0.0, 0.0),
        pid_y_(0.0, 0.0, 0.0),
        pid_z_(0.0, 0.0, 0.0)
  {
    // 订阅 /ee_velocity 话题，获取 Twist 消息
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/ee_velocity", 10,
        std::bind(&MotionPublisherAndSubscriber::topic_callback, this, std::placeholders::_1));

    // 创建发布器，发布到 /lbr/command/twist 话题
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/lbr/command/twist", 10);

    // 记录当前系统时间作为起始时间
    start_time_ = std::chrono::steady_clock::now();
  }

  void start_publishing()
  {
    // 设置发布频率，例如 100Hz
    rclcpp::WallRate loop_rate(100);

    // 开始主循环
    while (rclcpp::ok())
    {
      publishTwist();  // 定期调用 publishTwist
      rclcpp::spin_some(this->shared_from_this());  // 处理订阅回调
      loop_rate.sleep();  // 按照设定的频率休眠
    }
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // 从接收到的 Twist 消息中提取 linear x, y, z 速度分量
    real_linear_x_ = msg->linear.x;
    real_linear_y_ = msg->linear.y;
    real_linear_z_ = msg->linear.z;

    // 打印接收到的实际速度
    RCLCPP_INFO(this->get_logger(), "Received linear velocities: x: %.4f, y: %.4f, z: %.4f",
                real_linear_x_, real_linear_y_, real_linear_z_);
  }

  void publishTwist()
  {
    // 如果 real_linear_x_, real_linear_y_, 或 real_linear_z_ 超过 0.01，停止发布消息
    if (std::abs(real_linear_x_) > 0.01 || std::abs(real_linear_y_) > 0.01 || std::abs(real_linear_z_) > 0.01)
    {
      RCLCPP_ERROR(this->get_logger(), "Error: Speed too large! x: %.4f, y: %.4f, z: %.4f", 
                   real_linear_x_, real_linear_y_, real_linear_z_);
      return; // 终止发布消息
    }

    // 获取当前系统时间
    auto current_time = std::chrono::steady_clock::now();

    // 计算系统运行时间，单位为秒
    std::chrono::duration<double> elapsed_time = current_time - start_time_;
    double t_ = elapsed_time.count();

    // 检查是否超过最大运行时间
    if (t_ >= max_time_)
    {
      RCLCPP_INFO(this->get_logger(), "Reached maximum runtime. Shutting down.");
      rclcpp::shutdown();
      return;
    }

    // PID 控制部分：
    // 计算目标速度与实际速度之间的误差
    double error_x = target_linear_x_ - real_linear_x_;
    double error_y = target_linear_y_ - real_linear_y_;
    double error_z = target_linear_z_ - real_linear_z_;

    // 通过 PID 控制器计算控制输出
    double control_x = pid_x_.compute(error_x, 0.01); // 每次回调时间间隔 10ms
    double control_y = pid_y_.compute(error_y, 0.01);
    double control_z = pid_z_.compute(error_z, 0.01);

    // 创建 Twist 消息并设置线性速度（控制输出作为命令速度）
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = control_x;
    twist_msg.linear.y = control_y;
    twist_msg.linear.z = control_z;

    // 角速度保持为 0，可根据需要扩展 PID 控制
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;

    // 输出日志，显示当前时间、目标速度、实际速度以及控制量
    RCLCPP_INFO(this->get_logger(),
                "Time: %.2f s, Target Vel: [x: %.4f, y: %.4f, z: %.4f], Real Vel: [x: %.4f, y: %.4f, z: %.4f], Control: [x: %.4f, y: %.4f, z: %.4f]",
                t_,
                target_linear_x_, target_linear_y_, target_linear_z_,
                real_linear_x_, real_linear_y_, real_linear_z_,
                control_x, control_y, control_z);

    // 发布 Twist 消息
    publisher_->publish(twist_msg);
  }

  // 成员变量：订阅器、发布器
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  // 最大运行时间
  double max_time_;

  // 存储接收到的实际线性速度分量
  double real_linear_x_;
  double real_linear_y_;
  double real_linear_z_;

  // 目标线性速度
  double target_linear_x_;
  double target_linear_y_;
  double target_linear_z_;

  // PID 控制器（分别为 x, y, z 创建 PID 控制器）
  PID pid_x_;
  PID pid_y_;
  PID pid_z_;

  // 系统起始时间
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node is starting...");
  auto node = std::make_shared<MotionPublisherAndSubscriber>();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node is running...");
  node->start_publishing();  // 启动发布循环
  rclcpp::shutdown();
  return 0;
}