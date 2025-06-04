#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("cpp_demo_1st_node"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello world");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                         std::bind(&MyNode::timerCallback, this));
    }

private:
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Hello, this is 1st node with counter: %d", counter_);
        counter_++;
        if (counter_ > 10)
        {
            counter_ = 0; // Reset counter after reaching 50
            RCLCPP_INFO(this->get_logger(), "Counter reset to 0");
        }
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}