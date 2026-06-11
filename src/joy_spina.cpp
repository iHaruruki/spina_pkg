#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include <iomanip>
#include <sstream>
#include <chrono>
using namespace std::chrono_literals;

class AngleSendNode : public rclcpp::Node
{
public:
    AngleSendNode()
    : Node("angle_send_node"),
      up_down_(0), right_left_(0)
    {
        // パラメータ（必要なら起動時に上書き可能）
        publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
        step_            = this->declare_parameter<int>("step", 2);
        min_deg_         = this->declare_parameter<int>("min_deg", -90);
        max_deg_         = this->declare_parameter<int>("max_deg",  90);

        // min/max の妥当化
        if (min_deg_ >= max_deg_) { min_deg_ = -180; max_deg_ = 180; }

        pub_ = this->create_publisher<std_msgs::msg::String>(
            "angle_cmd", rclcpp::QoS(10).reliable());

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::SensorDataQoS(),
            std::bind(&AngleSendNode::joy_callback, this, std::placeholders::_1));

        auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / std::max(1e-6, publish_rate_hz_)));

        timer_ = this->create_wall_timer(period, std::bind(&AngleSendNode::on_timer, this));

        RCLCPP_INFO(this->get_logger(),
            "publish @ %.2f Hz, step=%d, range=[%d,%d]",
            publish_rate_hz_, step_, min_deg_, max_deg_);
    }

private:
    // 範囲サチュレート
    int clamp_deg(int v) const {
        if (v > max_deg_) return max_deg_;
        if (v < min_deg_) return min_deg_;
        return v;
    }

    // "+123" / "-005" 形式で3桁ゼロ埋め
    std::string format_signed3(const std::string& prefix, int value) const {
        std::ostringstream ss;
        ss << prefix
           << (value < 0 ? '-' : '+')
           << std::setw(3) << std::setfill('0') << std::abs(value);
        return ss.str();
    }

    void publish_value(const char* prefix, int value) {
        std_msgs::msg::String out;
        out.data = format_signed3(prefix, clamp_deg(value));
        pub_->publish(out);
    }

    // 定期送信：現在値を必ず [-180,180] に収めて2本送る
    void on_timer() {
        publish_value("A0p", up_down_);
        publish_value("A0r", right_left_);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "A0p=%s  A0r=%s",
            format_signed3("A0p", clamp_deg(up_down_)).c_str(),
            format_signed3("A0r", clamp_deg(right_left_)).c_str());
    }

    // Joy入力：状態更新時点でサチュレート
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() > 7) {
            //if (msg->axes[7] ==  1) up_down_   = clamp_deg(up_down_   - step_);
            //if (msg->axes[7] == -1) up_down_   = clamp_deg(up_down_   + step_);
            if (msg->axes[3] >  0.2) right_left_= clamp_deg(right_left_- step_);
            if (msg->axes[3] < -0.2) right_left_= clamp_deg(right_left_+ step_);
            RCLCPP_INFO(this->get_logger(), "Subscribe /joy topic");
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int up_down_;
    int right_left_;
    double publish_rate_hz_;
    int step_;
    int min_deg_;
    int max_deg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngleSendNode>());
    rclcpp::shutdown();
    return 0;
}
