#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class AngleCmdPublisher : public rclcpp::Node
{
public:
    AngleCmdPublisher()
        : Node("angle_cmd_publisher"),
          messages_(
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
        angle_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/angle_value", 10);

        timer_ = this->create_wall_timer(0.1s, std::bind(&AngleCmdPublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        const std::string &cmd = messages_[current_index_];

        std_msgs::msg::String string_msg;
        string_msg.data = cmd;
        publisher_->publish(string_msg);

        // 数値だけ取り出す
        int angle = parse_angle(cmd);

        std_msgs::msg::Int32 int_msg;
        int_msg.data = angle;
        angle_publisher_->publish(int_msg);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%s' angle=%d",
                    string_msg.data.c_str(), angle);

        current_index_ = (current_index_ + 1) % messages_.size();
    }

    int parse_angle(const std::string &cmd)
    {
        // "A0r+005" -> "+005" の部分を取り出して整数化
        return std::stoi(cmd.substr(3));
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr angle_publisher_;
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
