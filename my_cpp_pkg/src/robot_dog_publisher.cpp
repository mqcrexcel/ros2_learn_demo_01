#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::chrono_literals;

class RobotDogTalk : public rclcpp::Node 
{
public:
    RobotDogTalk() : Node("Robot_Dog_Talk"), robot_name_("RobotDog_01")
    {
        publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_dog_news", 10);
        timer_ = this->create_wall_timer(200ms, //std::chrono::milliseconds(200)
                                         std::bind(&RobotDogTalk::timerCallback, this));     
        RCLCPP_INFO(this->get_logger(), "Robot news publisher started");                                    
    }

private:
    void timerCallback()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("Hello, my name is ") + robot_name_;
        // RCLCPP_INFO(this->get_logger(), "Robot news publish: %d", counter_);
        counter_++;
        if (counter_ > 50) 
        {
            counter_ = 0;
            RCLCPP_INFO(this->get_logger(), "Counter reset to 0");
        }
        publisher_->publish(msg);
    }

    std::string robot_name_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotDogTalk>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
