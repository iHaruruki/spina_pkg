#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <string>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <cmath>     // std::abs, std::round

using namespace std::chrono_literals;

class AngleSendNode : public rclcpp::Node
{
public:
    AngleSendNode()
    : Node("angle_send_node"),
      up_down_(0.0),
      right_left_(0.0)
    {
        // Parameters (overridable at launch)
        publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 100.0);
        step_            = this->declare_parameter<double>("step", 0.5);
        min_deg_         = this->declare_parameter<double>("min_deg", -30.0);
        max_deg_         = this->declare_parameter<double>("max_deg",  30.0);
        deadzone_        = this->declare_parameter<double>("deadzone", 0.2);

        // Validate range
        if (min_deg_ >= max_deg_) {
            min_deg_ = -180.0;
            max_deg_ =  180.0;
        }
        if (step_ <= 0.0) {
            step_ = 0.5;
        }

        // Publisher
        pub_ = this->create_publisher<std_msgs::msg::String>(
            "/angle_cmd", rclcpp::QoS(10).reliable());

        // Joy subscriber
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            rclcpp::SensorDataQoS(),
            std::bind(&AngleSendNode::joy_callback, this, std::placeholders::_1));

        // Timer period from publish_rate_hz_
        auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / std::max(1e-6, publish_rate_hz_)));

        timer_ = this->create_wall_timer(period, std::bind(&AngleSendNode::on_timer, this));

        RCLCPP_INFO(this->get_logger(),
                    "publish @ %.2f Hz, step=%.3f, range=[%.1f,%.1f], deadzone=%.2f",
                    publish_rate_hz_, step_, min_deg_, max_deg_, deadzone_);
    }

private:
    double clamp_deg(double v) const
    {
        if (v > max_deg_) return max_deg_;
        if (v < min_deg_) return min_deg_;
        return v;
    }

    // Format: prefix + sign + zero-padded 3 digits (e.g., A0p+005, A0r-090)
    // NOTE: value is rounded to nearest int before formatting.
    std::string format_signed3(const std::string& prefix, double value) const
    {
        const int iv = static_cast<int>(std::round(value));

        std::ostringstream ss;
        ss << prefix
           << (iv < 0 ? '-' : '+')
           << std::setw(3) << std::setfill('0') << std::abs(iv);
        return ss.str();
    }

    void publish_value(const char* prefix, double value)
    {
        std_msgs::msg::String out;
        out.data = format_signed3(prefix, clamp_deg(value));
        pub_->publish(out);
    }

    // Periodic send: publish both channels every cycle
    void on_timer()
    {
        publish_value("A0p", up_down_);
        publish_value("A0r", right_left_);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "A0p=%.2f (%s)  A0r=%.2f (%s)",
            clamp_deg(up_down_),
            format_signed3("A0p", clamp_deg(up_down_)).c_str(),
            clamp_deg(right_left_),
            format_signed3("A0r", clamp_deg(right_left_)).c_str());
    }

    // Right stick control (typical mapping):
    // axes[3] = right stick left/right
    // axes[4] = right stick up/down
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() <= 4) {
            return;
        }

        const double rx = static_cast<double>(msg->axes[3]);  // right stick horizontal
        const double ry = static_cast<double>(msg->axes[4]);  // right stick vertical

        // Vertical -> up_down_
        if (ry > deadzone_) {
            up_down_ = clamp_deg(up_down_ + step_);
        } else if (ry < -deadzone_) {
            up_down_ = clamp_deg(up_down_ - step_);
        }

        // Horizontal -> right_left_
        if (rx > deadzone_) {
            right_left_ = clamp_deg(right_left_ - step_);
        } else if (rx < -deadzone_) {
            right_left_ = clamp_deg(right_left_ + step_);
        }

        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "joy rx=%.3f ry=%.3f -> up_down=%.2f right_left=%.2f",
            rx, ry, up_down_, right_left_);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double up_down_;
    double right_left_;

    double publish_rate_hz_;
    double step_;
    double min_deg_;
    double max_deg_;
    double deadzone_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngleSendNode>());
    rclcpp::shutdown();
    return 0;
}