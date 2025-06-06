#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::placeholders;

class GroundStation : public rclcpp::Node
{
public:
    GroundStation() : Node("Ground_Station")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "robot_dog_news", 10, 
            std::bind(&GroundStation::stationCallBack, this, _1));

        RCLCPP_INFO(this->get_logger(), "Ground station started");     
    }
    
private:
    void stationCallBack(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Ground station received: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GroundStation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
