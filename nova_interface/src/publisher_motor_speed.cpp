#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "actuator_msgs/msg/actuators.hpp"

using namespace std::chrono_literals;

class PublisherMotorSpeed : public rclcpp::Node
{
public:
  PublisherMotorSpeed()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<actuator_msgs::msg::Actuators>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        actuator_msgs::msg::Actuators message = actuator_msgs::msg::Actuators();
        std_msgs::msg::Header header;
        std::vector<double, std::allocator<double>> position;
        std::vector<double, std::allocator<double>> velocity;
        std::vector<double, std::allocator<double>> normalized;
        message.set__header(header);
        message.set__position(position);
        message.set__velocity(velocity);
        message.set__normalized(normalized);
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherMotorSpeed>());
  rclcpp::shutdown();

  return 0;
}