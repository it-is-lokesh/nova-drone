#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/altimeter.hpp>

class SubscriberAltimeter : public rclcpp::Node {
private:
    rclcpp::Subscription<ros_gz_interfaces::msg::Altimeter>::SharedPtr subscription_;
public:
    SubscriberAltimeter() : Node("node_subscriber_altimeter"){
        auto topic_callback = [this](ros_gz_interfaces::msg::Altimeter::UniquePtr msg) -> void {
            RCLCPP_INFO(this->get_logger(), "Received info, %p", (void *)&msg);
        };
        subscription_ = this->create_subscription<ros_gz_interfaces::msg::Altimeter>("/nova/altimeter", 10, topic_callback);
    }
};


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberAltimeter>());
    rclcpp::shutdown();
    
    return 0;
}