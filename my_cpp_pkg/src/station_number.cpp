#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class StationNumber : public rclcpp::Node 
{
public:
    StationNumber() : Node("station_number"), data_out_(0)
    {
        subscriber_ = this->create_subscription<std_msgs::msg::UInt16>(
            "number_topic", 10, 
            std::bind(&StationNumber::stationCallback, this, _1));

        publisher_ = this->create_publisher<std_msgs::msg::UInt16>("number_count", 10);

        server_ = this->create_service<std_srvs::srv::SetBool>(
            "reset_counter",
            std::bind(&StationNumber::resetCounterCallback, this, _1, _2)); 
            RCLCPP_INFO(this->get_logger(), "Service 'reset_counter' is ready to receive requests.");
            
            
    }

private:
    void stationCallback(const std_msgs::msg::UInt16::SharedPtr msg)
    {
        data_out_ = msg->data; // Store the received data
        RCLCPP_INFO(this->get_logger(), "Station received number: %d", data_out_);

        auto msg_out = std_msgs::msg::UInt16();
        msg_out.data = data_out_; // Use the stored data
        publisher_->publish(msg_out);

    }

    void resetCounterCallback(
        const std::shared_ptr<std_srvs::srv::SetBool_Request> request,
        const std::shared_ptr<std_srvs::srv::SetBool_Response> response)
    {
        resetNumber(request->data);
        response->success = true; // Indicate success
        response->message = "Counter reset to 0";
    }

    void resetNumber(bool bIs_reset_counter_)
    {
        if (bIs_reset_counter_)
        {
            RCLCPP_INFO(this->get_logger(), "Reset counter to 0");
            bIs_reset_counter_ = false; 
        }
        else
        {
            // Do Nothing
        }
    }


    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_;
    uint16_t data_out_;
    bool bIs_reset_counter_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StationNumber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
