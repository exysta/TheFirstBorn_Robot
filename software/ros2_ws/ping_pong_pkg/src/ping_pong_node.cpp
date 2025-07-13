#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class PingPongNode : public rclcpp::Node
{
public:
  PingPongNode() : Node("ping_pong_node"), count_(0)
  {
    // Publisher to send 'ping' messages to the MCU
    ping_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/ping_to_mcu", 10);

    // Subscriber to receive 'pong' messages from the MCU
    pong_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "/pong_from_mcu", 10, std::bind(&PingPongNode::pong_callback, this, std::placeholders::_1));
    
    // Timer to call the 'ping_timer_callback' every 1 second
    ping_timer_ = this->create_wall_timer(
      1s, std::bind(&PingPongNode::ping_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Ping-Pong node started. Waiting for MCU...");
  }

private:
  void ping_timer_callback()
  {
    auto message = std_msgs::msg::Int32();
    message.data = count_++;
    RCLCPP_INFO(this->get_logger(), "PC -> MCU: Publishing ping '%d'", message.data);
    ping_publisher_->publish(message);
  }

  void pong_callback(const std_msgs::msg::Int32 & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "PC <- MCU: Received pong '%d'", msg.data);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ping_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pong_subscription_;
  rclcpp::TimerBase::SharedPtr ping_timer_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PingPongNode>());
  rclcpp::shutdown();
  return 0;
}