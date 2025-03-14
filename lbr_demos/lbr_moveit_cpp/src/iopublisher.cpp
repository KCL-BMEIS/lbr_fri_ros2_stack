#include <rclcpp/rclcpp.hpp>
#include <lbr_fri_idl/msg/spindle_control.hpp>  // 这是你的 SpindleControl 消息类型
#include <chrono>
#include <functional>

class GpioCommandPublisher : public rclcpp::Node
{
public:
  GpioCommandPublisher() : Node("gpio_command_publisher")
  {
    // 创建 Publisher，发布到 "/lbr/command/spindle" 主题
    spindle_command_publisher_ = this->create_publisher<lbr_fri_idl::msg::SpindleControl>("/lbr/command/spindle", 10);

    // 使用定时器定期发布命令（例如每隔 10 毫秒）
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&GpioCommandPublisher::publish_spindle_command, this));
  }

private:
  void publish_spindle_command()
  {
    // 根据原始逻辑计算参数
    float speed         = 2000.0f;
    float feedr         = 0.01f;
    float feed          = speed * feedr / 60.0f;
    float unitIncrement = 20.0f / 65535.0f;
    float voltage       = 1.5f * speed / 10000.0f;
    float increment     = voltage / unitIncrement;

    // 创建消息并填充数据
    auto message = lbr_fri_idl::msg::SpindleControl();
    
    // 例如，将计算得到的 increment 用作主轴速度
    message.spindle_speed = increment;
    // 启动主轴
    message.spindle_start = 1.0;

    // 打印计算结果和消息数据
    RCLCPP_INFO(this->get_logger(), "Publishing Spindle Speed: %.2f, Spindle Start: %s, Feed: %.2f, Increment: %.2f", 
                message.spindle_speed, message.spindle_start ? "true" : "false", feed, increment);
    
    // 发布消息
    spindle_command_publisher_->publish(message);
  }

  rclcpp::Publisher<lbr_fri_idl::msg::SpindleControl>::SharedPtr spindle_command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  // 初始化 ROS2
  rclcpp::init(argc, argv);

  // 创建并运行节点
  rclcpp::spin(std::make_shared<GpioCommandPublisher>());

  // 清理 ROS2
  rclcpp::shutdown();
  return 0;
}
