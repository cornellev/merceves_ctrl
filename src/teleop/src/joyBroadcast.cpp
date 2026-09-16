#include <memory>
#include <chrono>
#include <numbers>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"

using std::placeholders::_1;


class VelocityTranslator : public rclcpp::Node
{
  public:
    VelocityTranslator() : Node("joyBroadcast")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&VelocityTranslator::topic_callback, this, _1));
      publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("ackDrive", 10);
    }

  private:
    void topic_callback(const sensor_msgs::msg::Joy & msg) const
    {
      float velocity = -1.0 * 2048.0 * (msg.axes[5] - 1);
      if(msg.axes[4] < .5f) { velocity *= -1; }
      //RCLCPP_INFO(this->get_logger(), "I heard: '%f'", velocity);
      auto message = ackermann_msgs::msg::AckermannDrive();
      message.speed = velocity;
      message.steering_angle = std::numbers::pi * 0.5 * (msg.axes[0]);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityTranslator>());
  rclcpp::shutdown();
  return 0;
}
