#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <array>
#include <optional>

class ArmController : public rclcpp::Node
{
public:
  ArmController()
  : Node("arm_controller")
  {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 2000000);

    serial_port_ = this->get_parameter("serial_port").as_string();
    int baudrate_param = this->get_parameter("baudrate").as_int();
    baudrate_ = to_speed_t(baudrate_param);

    if (!open_and_configure_serial()) {
      rclcpp::shutdown();
      return;
    }

    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/angle_cmd", 50,
      std::bind(&ArmController::cmdCallback, this, std::placeholders::_1));

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ArmController::onSetParameters, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ArmController node started (port=%s, baud=%d)",
                serial_port_.c_str(), baudrate_param);
  }

  ~ArmController() override
  {
    close_serial();
  }

private:
  // ============ シリアル制御 ============
  static speed_t to_speed_t(int b)
  {
    // C++17 以降、構造化バインディングで効率化
    static constexpr std::array<std::pair<int, speed_t>, 6> baud_map{{
      {9600, B9600}, {19200, B19200}, {38400, B38400},
      {57600, B57600}, {115200, B115200}, {2000000, B2000000}
    }};

    for (const auto& [rate, speed] : baud_map) {
      if (rate == b) return speed;
    }
    return B2000000; // デフォルト
  }

  void close_serial()
  {
    if (fd_ >= 0) {
      tcsetattr(fd_, TCSANOW, &oldtio_);
      close(fd_);
      fd_ = -1;
    }
  }

  bool open_and_configure_serial()
  {
    close_serial();

    fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open %s: %s",
                   serial_port_.c_str(), std::strerror(errno));
      return false;
    }

    struct termios newtio {};
    if (tcgetattr(fd_, &oldtio_) != 0) {
      RCLCPP_FATAL(this->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }

    // RAWモード設定
    cfmakeraw(&newtio);
    cfsetispeed(&newtio, baudrate_);
    cfsetospeed(&newtio, baudrate_);
    newtio.c_cflag |= (CLOCAL | CREAD | CS8);
#ifdef CRTSCTS
    newtio.c_cflag &= ~CRTSCTS;
#endif
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
    newtio.c_cc[VMIN]  = 0;
    newtio.c_cc[VTIME] = 1;

    tcflush(fd_, TCIOFLUSH);
    if (tcsetattr(fd_, TCSANOW, &newtio) != 0) {
      RCLCPP_FATAL(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      close_serial();
      return false;
    }

    return true;
  }

  // Jazzy 互換シグネチャ：const 参照で受け取る
  rcl_interfaces::msg::SetParametersResult onSetParameters(
      const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.set__successful(true);
    result.set__reason("ok");

    std::string new_port = serial_port_;
    speed_t new_baud = baudrate_;
    bool need_reopen = false;

    for (const auto & p : params) {
      if (p.get_name() == "serial_port" && 
          p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        new_port = p.as_string();
        need_reopen = true;
      } else if (p.get_name() == "baudrate" && 
                 p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        new_baud = to_speed_t(p.as_int());
        need_reopen = true;
      }
    }

    if (need_reopen) {
      const auto old_port = serial_port_;
      const auto old_baud = baudrate_;

      serial_port_ = new_port;
      baudrate_ = new_baud;

      if (!open_and_configure_serial()) {
        serial_port_ = old_port;
        baudrate_ = old_baud;
        (void)open_and_configure_serial();
        result.set__successful(false);
        result.set__reason("Failed to reopen serial with new parameters");
      } else {
        RCLCPP_INFO(this->get_logger(), "Serial reconfigured (port=%s)",
                    serial_port_.c_str());
      }
    }

    return result;
  }

  // ============ コマンド送信 ============
  struct Command {
    std::array<uint8_t, 11> data;
  };

  std::optional<Command> build_command(const std::string& buf, int module_id, int angle)
  {
    Command cmd;
    cmd.data[0] = 0xAA;
    cmd.data[1] = 0xC6;
    cmd.data[2] = 0x00;
    cmd.data[3] = 0x00;
    cmd.data[4] = 'C';
    cmd.data[5] = (module_id < 10) ? ('0' + module_id) : ('A' + (module_id - 10));
    cmd.data[6] = buf[2];
    cmd.data[7] = (angle < 0) ? '-' : '+';

    int abs_angle = std::abs(angle);
    cmd.data[8] = '0' + (abs_angle / 10);
    cmd.data[9] = '0' + (abs_angle % 10);
    cmd.data[10] = 0x55;

    return cmd;
  }

  void send_command(const Command& cmd)
  {
    if (fd_ >= 0) {
      ssize_t ret = write(fd_, cmd.data.data(), cmd.data.size());
      if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "write failed: %s", std::strerror(errno));
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Serial not open. Command skipped.");
    }
  }

  void cmdCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto& buf = msg->data;
    if (buf.size() < 7) {
      RCLCPP_WARN(this->get_logger(), "Invalid command length: %zu", buf.size());
      return;
    }

    int sign = (buf[3] == '-') ? -1 : 1;
    int angle = ((buf[4]-'0')*100) + ((buf[5]-'0')*10) + (buf[6]-'0');
    angle *= sign;

    if (buf[0] == 'C' && -30 <= angle && angle <= 30) {
      auto cmd = build_command(buf, std::stoi(std::string(1, buf[1])), angle);
      if (cmd) {
        send_command(*cmd);
        RCLCPP_DEBUG(this->get_logger(), "Sent command: %s", buf.c_str());
      }
    }
    else if (buf[0] == 'A' && -180 <= angle && angle <= 180) {
      for (int i = 1; i <= 6; i++) {
        auto cmd = build_command(buf, i, angle / 6);
        if (cmd) {
          send_command(*cmd);
          RCLCPP_DEBUG(this->get_logger(), "Sent command to module %d", i);
        }
        usleep(2000);
      }
    }
    else {
      RCLCPP_WARN(this->get_logger(), "Invalid value or out of range: %s", buf.c_str());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  int fd_{-1};
  struct termios oldtio_{};

  std::string serial_port_;
  speed_t baudrate_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmController>());
  rclcpp::shutdown();
  return 0;
}
