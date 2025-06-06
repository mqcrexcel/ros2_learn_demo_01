#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"

using namespace std::chrono_literals;

class StationNumber : public rclcpp::Node 
{
public:
    StationNumber() : Node("station_number"), data_out_(0)
    {
        subscriber_ = this->create_subscription<std_msgs::msg::UInt16>(
            "number_topic", 10, 
            std::bind(&StationNumber::stationCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::UInt16>("number_count", 10);
               
    }

private:
    void stationCallback(const std_msgs::msg::UInt16::SharedPtr msg)
    {
        data_out_ = msg->data; // Store the received data
        RCLCPP_INFO(this->get_logger(), "Station received number: %d", msg->data);

        auto msg_out = std_msgs::msg::UInt16();
        msg_out.data = data_out_; // Use the stored data
        publisher_->publish(msg_out);

    }

    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
    uint16_t data_out_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StationNumber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
