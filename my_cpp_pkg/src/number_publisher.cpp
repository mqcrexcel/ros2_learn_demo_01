#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"

using namespace std::chrono_literals;

class NumberPublisher : public rclcpp::Node
{
public:
    NumberPublisher() : Node("number_publisher"), counter_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::UInt16>("number_topic", 10);
        timer_ = this->create_wall_timer(
            200ms, // std::chrono::milliseconds(200)
            std::bind(&NumberPublisher::timer_callback, this));            

        RCLCPP_INFO(this->get_logger(), "Number publisher started");
    }

private:
    void timer_callback()
    {
        auto msg = std_msgs::msg::UInt16();
        msg.data = counter_;
        counter_++;
        RCLCPP_INFO(this->get_logger(), "Counter: %d", counter_);

        if (counter_ > 50) 
        {
            counter_ = 0;
            RCLCPP_INFO(this->get_logger(), "Counter reset to 0");
        }
        publisher_->publish(msg);
    }
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
