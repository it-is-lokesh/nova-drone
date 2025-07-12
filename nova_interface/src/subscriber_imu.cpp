#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class SubscriberIMU : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
public:
    SubscriberIMU() : Node("node_subscriber_imu"){
        auto topic_callback = [this](sensor_msgs::msg::Imu::UniquePtr msg) -> void {
            RCLCPP_INFO(this->get_logger(), "Received info, %p", (void *)&msg);
        };
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("/nova/imu", 10, topic_callback);
    }
};


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberIMU>());
    rclcpp::shutdown();
    
    return 0;
}