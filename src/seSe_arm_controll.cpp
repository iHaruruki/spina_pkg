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

class ArmController : public rclcpp::Node
{
public:
  ArmController()
  : Node("arm_controller")
  {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 2000000);

    // パラメータ取得
    serial_port_ = this->get_parameter("serial_port").as_string();
    int baudrate_param = this->get_parameter("baudrate").as_int();
    baudrate_ = to_speed_t(baudrate_param);

    // シリアル初期化
    if (!open_and_configure_serial()) {
      rclcpp::shutdown();
      return;
    }

    sub_ = this->create_subscription<std_msgs::msg::String>(
      "angle_cmd", 50,
      std::bind(&ArmController::cmdCallback, this, std::placeholders::_1));

    // パラメータ動的変更に対応（serial_port / baudrate）
    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ArmController::onSetParameters, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ArmController node started (port=%s, baud=%d)",
                serial_port_.c_str(), baudrate_param);
  }

  ~ArmController() override
  {
    if (fd_ >= 0) {
      tcsetattr(fd_, TCSANOW, &oldtio_);
      close(fd_);
      fd_ = -1;
    }
  }

private:
  // termios 速度設定用に整数→speed_t へ変換
  static speed_t to_speed_t(int b)
  {
    switch (b) {
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
#ifdef B2000000
      case 2000000: return B2000000;
#endif
#ifdef B1000000
      case 1000000: return B1000000;
#endif
      default:
        // 未対応の値なら最も一般的な 115200 にフォールバック
        return B115200;
    }
  }

  bool open_and_configure_serial()
  {
    // 既に開いていれば閉じる
    if (fd_ >= 0) {
      tcsetattr(fd_, TCSANOW, &oldtio_);
      close(fd_);
      fd_ = -1;
    }

    fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open %s: %s",
                   serial_port_.c_str(), std::strerror(errno));
      return false;
    }

    // 現在設定の保存
    if (tcgetattr(fd_, &oldtio_) != 0) {
      RCLCPP_FATAL(this->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }

    // RAWモード設定
    struct termios newtio;
    std::memset(&newtio, 0, sizeof(newtio));
    cfmakeraw(&newtio);
    cfsetispeed(&newtio, baudrate_);
    cfsetospeed(&newtio, baudrate_);
    newtio.c_cflag |= (CLOCAL | CREAD | CS8);
#ifdef CRTSCTS
    // ハード/ソフトフロー制御OFF
    newtio.c_cflag &= ~CRTSCTS;
#endif
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY);

    newtio.c_cc[VMIN]  = 0;
    newtio.c_cc[VTIME] = 1; // 0.1s

    tcflush(fd_, TCIOFLUSH);
    if (tcsetattr(fd_, TCSANOW, &newtio) != 0) {
      RCLCPP_FATAL(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      close(fd_);
      fd_ = -1;
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
      // 反映を試みる
      std::string old_port = serial_port_;
      speed_t old_baud = baudrate_;

      serial_port_ = new_port;
      baudrate_ = new_baud;

      if (!open_and_configure_serial()) {
        // 失敗したら元に戻す
        serial_port_ = old_port;
        baudrate_ = old_baud;
        (void)open_and_configure_serial(); // 元設定で再度オープン
        result.set__successful(false);
        result.set__reason("Failed to reopen serial with new parameters");
      } else {
        RCLCPP_INFO(this->get_logger(), "Serial reconfigured (port=%s)",
                    serial_port_.c_str());
      }
    }

    return result;
  }

  void cmdCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string &buf = msg->data;
    if (buf.size() < 7) {
      RCLCPP_WARN(this->get_logger(), "Invalid command length");
      return;
    }

    // ±ddd 形式を想定（publisher側で常に符号付け）
    int sign = (buf[3] == '-') ? -1 : 1;
    int angle = ((buf[4]-'0')*100) + ((buf[5]-'0')*10) + (buf[6]-'0');
    angle *= sign;

    char send[11];

    if (buf[0]=='C' && -30 <= angle && angle <= 30) {
      send[0] = 0xAA;
      send[1] = 0xC6;
      send[2] = 0x00;
      send[3] = 0x00;
      send[4] = 'C';
      send[5] = buf[1];
      send[6] = buf[2];
      send[7] = buf[3]; // 符号
      send[8] = (std::abs(angle)/10) + '0';
      send[9] = (std::abs(angle) - ((std::abs(angle)/10) * 10)) + '0';
      send[10] = 0x55;
      if (fd_ >= 0) {
        write(fd_, send, sizeof(send));
        RCLCPP_INFO(this->get_logger(), "Sent command: %s", buf.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Serial not open. Command skipped.");
      }
    }
    else if (buf[0]=='A' && -180 <= angle && angle <= 180) {
      // 全体角度制御：各モジュールへ短いインターバルで送信
      for (int i = 0; i < 6; i++) {
        send[0] = 0xAA;
        send[1] = 0xC6;
        send[2] = 0x00;
        send[3] = 0x00;
        send[4] = 'C';
        send[5] = i+1+'0';
        send[6] = buf[2];
        send[7] = buf[3]; // 符号
        // 角度→プロトコルの2桁表現
        int a = std::abs(angle);
        send[8] = (a/60) + '0';
        send[9] = (a/6 - ((a/60) * 10)) + '0';
        send[10] = 0x55;
        if (fd_ >= 0) {
          write(fd_, send, sizeof(send));
          RCLCPP_INFO(this->get_logger(), "Sent command to module %d: %s", i+1, buf.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Serial not open. Command skipped for module %d.", i+1);
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

  // Jazzy では型名が変更されているため auto で型推論させる
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmController>());
  rclcpp::shutdown();
  return 0;
}
