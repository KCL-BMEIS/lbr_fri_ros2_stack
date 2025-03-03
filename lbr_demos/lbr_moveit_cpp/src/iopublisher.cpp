#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class IoCommandControllerNode : public rclcpp::Node
{
public:
  IoCommandControllerNode()
  : Node("io_command_controller_node")
  {
    // 订阅 /lbr/io_command_controller/transition_event 话题（这里以 Bool 消息为例）
    transition_event_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/lbr/io_command_controller/transition_event",
      10,
      std::bind(&IoCommandControllerNode::transition_event_callback, this, std::placeholders::_1));

    // 创建向Speed和Start发送消息的发布者
    speed_pub_ = this->create_publisher<std_msgs::msg::Int32>("/lbr/analog_command_topic/Speed", 10);
    start_pub_ = this->create_publisher<std_msgs::msg::Bool>("/lbr/boolean_command_topic/Start", 10);

    // 定时器，每10ms调用一次publish_commands()函数
    timer_ = this->create_wall_timer(10ms, std::bind(&IoCommandControllerNode::publish_commands, this));
  }

private:
  // 订阅回调函数，打印接收到的transition_event消息
  void transition_event_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received transition event: %s", msg->data ? "true" : "false");
  }

  // 定时器回调函数，每10ms发布Speed和Start消息
  void publish_commands()
  {
    auto speed_msg = std_msgs::msg::Int32();
    speed_msg.data = 1000;
    speed_pub_->publish(speed_msg);

    auto start_msg = std_msgs::msg::Bool();
    start_msg.data = true;
    start_pub_->publish(start_msg);

    RCLCPP_INFO(this->get_logger(), "Published Speed: %d, Start: %s",
                speed_msg.data, start_msg.data ? "true" : "false");
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr transition_event_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IoCommandControllerNode>());
  rclcpp::shutdown();
  return 0;
}
