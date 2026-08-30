#include <chrono>
#include <memory>
#include <string>
#include <algorithm>
#include <cstdio>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

class AngleCmdPublisher : public rclcpp::Node
{
public:
  AngleCmdPublisher()
  : Node("angle_cmd_publisher"),
    yaw_angle_(0),    // r
    pitch_angle_(0)   // p
  {
    cmd_publisher_   = this->create_publisher<std_msgs::msg::String>("/angle_cmd", 10);
    yaw_publisher_   = this->create_publisher<std_msgs::msg::Int32>("/yaw_value", 10);
    pitch_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/pitch_value", 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&AngleCmdPublisher::joy_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(50ms, std::bind(&AngleCmdPublisher::publish_message, this));
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->axes.size() <= 4) return;

    const float yaw_axis   = msg->axes[3]; // 右スティック左右 -> r(yaw)
    const float pitch_axis = -msg->axes[4]; // 右スティック上下 -> p(pitch)

    const float deadzone = 0.2f;
    const int step = 2;

    if (std::fabs(yaw_axis) >= deadzone) {
      yaw_angle_ += (yaw_axis > 0.0f) ? step : -step;
    }

    if (std::fabs(pitch_axis) >= deadzone) {
      pitch_angle_ += (pitch_axis < 0.0f) ? step : -step;
    }

    // リミッタ
    yaw_angle_   = std::clamp(yaw_angle_,   -180, 180);
    pitch_angle_ = std::clamp(pitch_angle_, -180, 180);

    RCLCPP_DEBUG(this->get_logger(), "joy axes: yaw_axis=%.3f pitch_axis=%.3f", yaw_axis, pitch_axis);
  }

  std::string format_cmd(char axis, int angle)
  {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "A0%c%+04d", axis, angle);
    return std::string(buf);
  }

  void publish_message()
  {
    const std::string yaw_cmd   = format_cmd('r', yaw_angle_);
    const std::string pitch_cmd = format_cmd('p', pitch_angle_);

    // 受信側互換性のため「別メッセージで送る」
    std_msgs::msg::String msg1;
    msg1.data = yaw_cmd;
    cmd_publisher_->publish(msg1);

    std_msgs::msg::String msg2;
    msg2.data = pitch_cmd;
    cmd_publisher_->publish(msg2);

    std_msgs::msg::Int32 ymsg;
    ymsg.data = yaw_angle_;
    yaw_publisher_->publish(ymsg);

    std_msgs::msg::Int32 pmsg;
    pmsg.data = pitch_angle_;
    pitch_publisher_->publish(pmsg);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s', '%s' (yaw=%d, pitch=%d)",
                yaw_cmd.c_str(), pitch_cmd.c_str(), yaw_angle_, pitch_angle_);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr yaw_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pitch_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int yaw_angle_;
  int pitch_angle_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngleCmdPublisher>());
  rclcpp::shutdown();
  return 0;
}