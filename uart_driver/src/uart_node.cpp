#include "uart_node.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cstring>

using namespace std::chrono_literals;

UartNode::UartNode() : Node("uart_node")
{
    fd = open("/dev/ttyS5", O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "串口打开失败");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "串口 /dev/ttyS5 打开成功");

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= CS8 | CREAD | CLOCAL;
    options.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    options.c_iflag = 0;
    options.c_lflag = 0;
    options.c_oflag = 0;
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 10;
    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIOFLUSH);

    pub_ = this->create_publisher<std_msgs::msg::String>("uart_rx", 10);
    key_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("key_state", 10);
    joy_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    joy_.gearInit();

    timer_ = this->create_wall_timer(25ms, std::bind(&UartNode::readLoop, this));
}

UartNode::~UartNode()
{
    if (fd >= 0) close(fd);
}

uint16_t UartNode::crc16_update(uint16_t crc, uint8_t data)
{
    crc ^= static_cast<uint16_t>(data);
    for (int i = 0; i < 8; i++)
    {
        if (crc & 0x0001)
            crc = (crc >> 1) ^ 0xA001;
        else
            crc >>= 1;
    }
    return crc;
}

uint16_t UartNode::crc16_calc(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
        crc = crc16_update(crc, data[i]);
    return crc;
}

KeyState UartNode::parseKeyToState(uint16_t key_mask)
{
    KeyState ks;
    ks.speed_up      = (key_mask & (1U << PANEL_KEY_BIT_SPEED_GEAR_UP))     ? 1 : 0;
    ks.speed_down    = (key_mask & (1U << PANEL_KEY_BIT_SPEED_GEAR_DOWN))   ? 1 : 0;
    ks.putter_up     = (key_mask & (1U << PANEL_KEY_BIT_PUSH_ROD_GEAR_UP))  ? 1 : 0;
    ks.putter_down   = (key_mask & (1U << PANEL_KEY_BIT_PUSH_ROD_GEAR_DOWN))? 1 : 0;
    ks.auto_nav      = (key_mask & (1U << PANEL_KEY_BIT_AUTO_NAV))          ? 1 : 0;
    ks.voice_mode    = (key_mask & (1U << PANEL_KEY_BIT_VOICE_MODE))        ? 1 : 0;
    ks.sos           = (key_mask & (1U << PANEL_KEY_BIT_SOS))               ? 1 : 0;
    ks.buzzer        = (key_mask & (1U << PANEL_KEY_BIT_BUZZER))            ? 1 : 0;
    ks.ambient_light = (key_mask & (1U << PANEL_KEY_BIT_AMBIENT_LIGHT))     ? 1 : 0;

    joy_.setGearUpEvt(ks.speed_up);
    joy_.setGearDnEvt(ks.speed_down);
    joy_.gearUpdate();

    return ks;
}

JoystickData UartNode::parseJoystickRaw(const uint8_t* frame)
{
    uint16_t adc_x = static_cast<uint16_t>(frame[5] | (frame[6] << 8));
    uint16_t adc_y = static_cast<uint16_t>(frame[7] | (frame[8] << 8));

    joy_.stateMachineUpdate(adc_x, adc_y);

    const JoySpeed& sp = joy_.getSpeed();
    //直接原始浮点，不乘以1000缩放
    if((abs_f(sp.line_speed) < 0.001f) && (abs_f(sp.angle_speed) < 0.001f) \
       && abs_f(joystick_data_.speed_x) < 0.001f && abs_f(joystick_data_.speed_y) < 0.001f)
    {
        curve_line_.lineInit();
        curve_angle_.angleInit();
        joystick_data_.speed_x = 0.0f;
        joystick_data_.speed_y = 0.0f;
    }
    else
    {
        joystick_data_.speed_x = curve_line_.calc(sp.line_speed);
        joystick_data_.speed_y = curve_angle_.calc(sp.angle_speed);
    }
    return joystick_data_;
}

void UartNode::publishKeyState(const KeyState& ks)
{
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = {
        ks.speed_up, ks.speed_down, ks.putter_up, ks.putter_down,
        ks.auto_nav, ks.voice_mode, ks.sos, ks.buzzer, ks.ambient_light
    };
    key_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "按键发布");
}

void UartNode::publishJoystickData(const JoystickData& js)
{
    geometry_msgs::msg::Twist msg;
    uint8_t buf[8];
    // 单精度float各4字节，打包8字节二进制
    std::memcpy(buf, &js.speed_x, sizeof(float));
    std::memcpy(buf + sizeof(float), &js.speed_y, sizeof(float));
    msg.linear.x = js.speed_x;
    msg.angular.z = js.speed_y;

    joy_pub_->publish(msg);
//    RCLCPP_INFO(this->get_logger(), "摇杆速度发布");
}

void UartNode::parseFrame(const std::vector<uint8_t>& frame)
{
    if (frame.size() < 8) return;
    uint8_t head1 = frame[0], head2 = frame[1];
    uint8_t len = frame[3], func = frame[4];
    uint8_t crc_l = frame[frame.size()-3];
    uint8_t crc_h = frame[frame.size()-2];
    uint8_t tail = frame.back();

    if (head1 != 0xAA || head2 != 0x55 || tail != 0xEF) return;
    if (frame.size() != static_cast<size_t>(4 + len)) return;

    uint16_t rx_crc = static_cast<uint16_t>((crc_h << 8) | crc_l);
    uint16_t calc_crc = crc16_calc(&frame[2], len - 1);
    if (rx_crc != calc_crc)
    {
        RCLCPP_WARN(this->get_logger(), "CRC校验错误：接收=0x%04X，计算=0x%04X", rx_crc, calc_crc);
        return;
    }

    if (func == DATA_PROC_FUNC_KEY_CLICK)
    {
        uint16_t key_mask = static_cast<uint16_t>(frame[5] | (frame[6] << 8));
        KeyState state = parseKeyToState(key_mask);
        publishKeyState(state);
    }
    if (func == DATA_PROC_FUNC_JOYSTICK)
    {
        JoystickData js = parseJoystickRaw(frame.data());
        publishJoystickData(js);
    }
}

bool UartNode::findHeader(std::vector<uint8_t>& buf, size_t& pos)
{
    for (size_t i = 0; i < buf.size() - 1; i++)
    {
        if (buf[i] == 0xAA && buf[i+1] == 0x55)
        {
            pos = i;
            return true;
        }
    }
    return false;
}

void UartNode::readLoop()
{
    uint8_t buf[1024];
    int n = read(fd, buf, sizeof(buf));
    if (n <= 0) return;
    rx_buffer_.insert(rx_buffer_.end(), buf, buf + n);

    while (rx_buffer_.size() >= 8)
    {
        size_t pos = 0;
        if (!findHeader(rx_buffer_, pos))
        {
            rx_buffer_.clear();
            break;
        }
        if (pos > 0)
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + pos);
        if (rx_buffer_.size() < 4) break;

        uint8_t len = rx_buffer_[3];
        size_t total = static_cast<size_t>(4 + len);
        if (rx_buffer_.size() < total) break;

        std::vector<uint8_t> frame(rx_buffer_.begin(), rx_buffer_.begin() + total);
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total);
        parseFrame(frame);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UartNode>());
    rclcpp::shutdown();
    return 0;
}
