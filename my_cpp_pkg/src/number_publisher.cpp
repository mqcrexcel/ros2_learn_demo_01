#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class NumberPublisher : public rclcpp::Node
{
public:
    NumberPublisher() : Node("number_publisher"), counter_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::UInt16>("number_topic", 10);
        timer_ = this->create_wall_timer(
            200ms, // std::chrono::milliseconds(200)
            std::bind(&NumberPublisher::timer_callback, this));            
        
        client_ = this->create_client<std_srvs::srv::SetBool>("reset_counter"); 

        RCLCPP_INFO(this->get_logger(), "Number publisher started");
    }

    void reset_counter()
    {
        while (!client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for service");  
        }

        auto request = std::make_shared<std_srvs::srv::SetBool_Request>();
        request->data = true; // Reset counter request

        client_->async_send_request(
            request,
            std::bind(&NumberPublisher::callbackResetCounter, this, _1)); // std::placeholders::_1        
    }


private:
    void timer_callback()
    {
        auto msg = std_msgs::msg::UInt16();
        msg.data = counter_;
        counter_++;
        RCLCPP_INFO(this->get_logger(), "Counter: %d", counter_);
        publisher_->publish(msg);
    }

    void callbackResetCounter(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "Counter reset to 0");
            counter_ = 0; // Reset the counter
        }
    }

    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisher>();
    node->reset_counter();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
