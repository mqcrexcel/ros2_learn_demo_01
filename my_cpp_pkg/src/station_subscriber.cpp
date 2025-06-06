#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::placeholders;

class GroundStation : public rclcpp::Node
{
public:
    GroundStation() : Node("Ground_Station")
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::String>(
            "robot_dog_news", 10, 
            std::bind(&GroundStation::stationCallBack, this, _1));

        RCLCPP_INFO(this->get_logger(), "Ground station started");     
    }
    
private:
    void stationCallBack(const example_interfaces::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Ground station received: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GroundStation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
