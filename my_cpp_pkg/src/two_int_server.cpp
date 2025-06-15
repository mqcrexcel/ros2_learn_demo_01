#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::placeholders;

class AddTwoIntsServer : public rclcpp::Node
{
public:
    AddTwoIntsServer() : Node("add_two_ints_server")
    {
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&AddTwoIntsServer::add_two_ints_callback, this, _1, _2)); // std::placeholders::_1, _2
            RCLCPP_INFO(this->get_logger(), "Service 'add_two_ints' is ready to receive requests.");
    }

private:
    void add_two_ints_callback(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Incoming request: a=%ld, b=%ld, sum=%ld",
                    request->a, request->b, response->sum); 
    }
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
