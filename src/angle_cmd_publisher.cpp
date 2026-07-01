#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class AngleCmdPublisher : public rclcpp::Node
{
public:
    AngleCmdPublisher()
        : Node("angle_cmd_publisher"), messages_(
        {
            "A0r-30", "A0r-29", "A0r-28", "A0r-27", "A0r-26", "A0r-25",
            "A0r-24", "A0r-23", "A0r-22", "A0r-21", "A0r-20", "A0r-19",
            "A0r-18", "A0r-17", "A0r-16", "A0r-15", "A0r-14", "A0r-13",
            "A0r-12", "A0r-11", "A0r-10", "A0r-09", "A0r-08", "A0r-07",
            "A0r-06", "A0r-05", "A0r-04", "A0r-03", "A0r-02", "A0r-01",
            "A0r+00", "A0r+01", "A0r+02", "A0r+03", "A0r+04", "A0r+05",
            "A0r+06", "A0r+07", "A0r+08", "A0r+09", "A0r+10", "A0r+11",
            "A0r+12", "A0r+13", "A0r+14", "A0r+15", "A0r+16", "A0r+17",
            "A0r+18", "A0r+19", "A0r+20", "A0r+21", "A0r+22", "A0r+23",
            "A0r+24", "A0r+25", "A0r+26", "A0r+27", "A0r+28", "A0r+29",
            "A0r+30",
        }),
        current_index_(0)

    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/angle_cmd", 10);
        timer_ = this->create_wall_timer(2s, std::bind(&AngleCmdPublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        auto message = std_msgs::msg::String();
        message.data = messages_[current_index_];
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);

        // Move to the next message in the list
        current_index_ = (current_index_ + 1) % messages_.size();
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> messages_;
    size_t current_index_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngleCmdPublisher>());
    rclcpp::shutdown();
    return 0;
}
