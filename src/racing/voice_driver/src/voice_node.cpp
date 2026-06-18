#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>  // 添加 SMBus 底层定义
#include <cerrno>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class RdkVoiceNode : public rclcpp::Node
{
public:
  RdkVoiceNode() : Node("rdk_voice_node"), i2c_file_(-1)
  {
    // 1. 初始化 I2C 设备
    const char *device = "/dev/i2c-5";
    i2c_file_ = open(device, O_RDWR);
    
    if (i2c_file_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "无法打开 I2C 总线: %s", device);
    } else {
      int addr = 0x2b;  // Origincar/RDK expansion: i2cdetect -y -r 5 
      if (ioctl(i2c_file_, I2C_SLAVE, addr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "无法获取总线访问权限");
      } else {
        RCLCPP_INFO(this->get_logger(), "I2C 总线连接成功！准备使用 SMBus 协议...");
      }
    }

    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "play_voice_id", 10, std::bind(&RdkVoiceNode::voice_cmd_callback, this, _1));
    
    RCLCPP_INFO(this->get_logger(), "语音控制节点(C++)已启动，等待接收播报 ID...");
  }

  ~RdkVoiceNode() {
    if (i2c_file_ >= 0) {
      close(i2c_file_);
    }
  }

private:
  // CI13XX IIC 被动播报: [0x03, cmd_id, 0x03+cmd_id, 0x5A]
  int ci13_play_packet(int fd, uint8_t cmd_id) const {
    uint8_t reg = 0x03;
    uint8_t packet[4] = {
      reg,
      cmd_id,
      static_cast<uint8_t>(reg + cmd_id),
      0x5A,
    };

    struct i2c_msg msg;
    msg.addr = 0x2b;
    msg.flags = 0;
    msg.len = sizeof(packet);
    msg.buf = packet;

    struct i2c_rdwr_ioctl_data ioctl_data;
    ioctl_data.msgs = &msg;
    ioctl_data.nmsgs = 1;

    return ioctl(fd, I2C_RDWR, &ioctl_data);
  }

  void voice_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg) const
  {
    if (i2c_file_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "I2C 设备未就绪，无法播报");
      return;
    }

    uint8_t cmd_id = static_cast<uint8_t>(msg->data) & 0xFF;

    if (ci13_play_packet(i2c_file_, cmd_id) < 0) {
      RCLCPP_ERROR(this->get_logger(), "播报失败，底层系统报错: %s (errno: %d)", strerror(errno), errno);
    } else {
      RCLCPP_INFO(this->get_logger(), "播报成功! CI13 包 ID: 0x%02X", cmd_id);
    }
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  int i2c_file_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RdkVoiceNode>());
  rclcpp::shutdown();
  return 0;
}