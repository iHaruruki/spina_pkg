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
            "A0r-120", "A0r-115", "A0r-110", "A0r-105", "A0r-100",
            "A0r-095", "A0r-090", "A0r-085", "A0r-080", "A0r-075",
            "A0r-070", "A0r-065", "A0r-060", "A0r-055", "A0r-050",
            "A0r-045", "A0r-040", "A0r-035", "A0r-030", "A0r-025",
            "A0r-020", "A0r-015", "A0r-010", "A0r-005", "A0r+000",
            "A0r+005", "A0r+010", "A0r+015", "A0r+020", "A0r+025",
            "A0r+030", "A0r+035", "A0r+040", "A0r+045", "A0r+050",
            "A0r+055", "A0r+060", "A0r+065", "A0r+070", "A0r+075",
            "A0r+080", "A0r+085", "A0r+090", "A0r+095", "A0r+100",
            "A0r+105", "A0r+110", "A0r+115", "A0r+120",
            "A0r+115", "A0r+110", "A0r+105", "A0r+100", "A0r+095",
            "A0r+090", "A0r+085", "A0r+080", "A0r+075", "A0r+070",
            "A0r+065", "A0r+060", "A0r+055", "A0r+050", "A0r+045",
            "A0r+040", "A0r+035", "A0r+030", "A0r+025", "A0r+020",
            "A0r+015", "A0r+010", "A0r+005", "A0r+000", "A0r-005",
            "A0r-010", "A0r-015", "A0r-020", "A0r-025", "A0r-030",
            "A0r-035", "A0r-040", "A0r-045", "A0r-050", "A0r-055",
            "A0r-060", "A0r-065", "A0r-070", "A0r-075", "A0r-080",
            "A0r-085", "A0r-090", "A0r-095", "A0r-100", "A0r-105",
            "A0r-110", "A0r-115", "A0r-120",
        }),
    current_index_(0)

    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/angle_cmd", 10);
        timer_ = this->create_wall_timer(0.1s, std::bind(&AngleCmdPublisher::publish_message, this));
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
