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
    roll_angle_(0),
    pitch_angle_(0)
  {
    cmd_publisher_   = this->create_publisher<std_msgs::msg::String>("/angle_cmd", 10);
    roll_publisher_  = this->create_publisher<std_msgs::msg::Int32>("/roll_value", 10);
    pitch_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/pitch_value", 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&AngleCmdPublisher::joy_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(100ms, std::bind(&AngleCmdPublisher::publish_message, this));
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // 右スティック index（要確認）
    // axes[3] : 右スティック左右 -> roll
    // axes[4] : 右スティック上下 -> pitch
    if (msg->axes.size() <= 4) return;

    const float roll_axis  = msg->axes[3];
    const float pitch_axis = msg->axes[4];

    const float deadzone = 0.2f;
    const int step = 2;  // 1 callbackあたりの角度変化量

    if (std::fabs(roll_axis) >= deadzone) {
      roll_angle_ += (roll_axis > 0.0f) ? step : -step;
    }

    // 多くのゲームパッドは上方向が -1 なので、直感操作に合わせて符号反転
    if (std::fabs(pitch_axis) >= deadzone) {
      pitch_angle_ += (pitch_axis < 0.0f) ? step : -step;
    }

    // リミッタ ±90°
    roll_angle_  = std::clamp(roll_angle_,  -180, 180);
    pitch_angle_ = std::clamp(pitch_angle_, -180, 180);
  }

  std::string format_joint_cmd(const char* joint_prefix, int angle)
  {
    // 例: "A0r+005", "A1p-030"
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%s%+04d", joint_prefix, angle);
    return std::string(buf);
  }

  void publish_message()
  {
    // 2軸コマンドを1文字列にまとめる（受信側仕様に合わせて区切りを変更）
    // 例: "A0r+010 A1p-020"
    const std::string roll_cmd  = format_joint_cmd("A0r", roll_angle_);
    const std::string pitch_cmd = format_joint_cmd("A0p", pitch_angle_);
    const std::string full_cmd  = roll_cmd + " " + pitch_cmd;

    std_msgs::msg::String cmd_msg;
    cmd_msg.data = full_cmd;
    cmd_publisher_->publish(cmd_msg);

    std_msgs::msg::Int32 roll_msg;
    roll_msg.data = roll_angle_;
    roll_publisher_->publish(roll_msg);

    std_msgs::msg::Int32 pitch_msg;
    pitch_msg.data = pitch_angle_;
    pitch_publisher_->publish(pitch_msg);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s' (roll=%d, pitch=%d)",
                full_cmd.c_str(), roll_angle_, pitch_angle_);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr roll_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pitch_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int roll_angle_;
  int pitch_angle_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngleCmdPublisher>());
  rclcpp::shutdown();
  return 0;
}