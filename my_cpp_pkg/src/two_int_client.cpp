#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class AddTwoIntsClient : public rclcpp::Node
{
public:
    AddTwoIntsClient() : Node("add_two_ints_client")
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        RCLCPP_INFO(this->get_logger(), "Service 'add_two_ints' is available.");       
    }
    
    void send_two_ints_request(int64_t a, int64_t b)
    {
        while (!client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for service");  
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        client_->async_send_request(
            request,
            std::bind(&AddTwoIntsClient::callbackAddTwoInts, this, _1)); // std::placeholders::_1
    }

private:
    void callbackAddTwoInts(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Sum is: %ld", response->sum);
    }

    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClient>();
    node->send_two_ints_request(10, 18);
    node->send_two_ints_request(10, 89);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
