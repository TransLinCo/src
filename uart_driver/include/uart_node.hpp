#ifndef UART_NODE_HPP
#define UART_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <vector>
#include <cstdint>
#include "per_joystick.hpp"

#define DATA_PROC_FUNC_HEARTBEAT          (0x01U)
#define DATA_PROC_FUNC_JOYSTICK           (0x02U)
#define DATA_PROC_FUNC_KEY_CLICK          (0x03U)

typedef enum
{
    PANEL_KEY_BIT_SPEED_GEAR_UP = 0,
    PANEL_KEY_BIT_SPEED_GEAR_DOWN,
    PANEL_KEY_BIT_PUSH_ROD_GEAR_UP,
    PANEL_KEY_BIT_PUSH_ROD_GEAR_DOWN,
    PANEL_KEY_BIT_AUTO_NAV,
    PANEL_KEY_BIT_VOICE_MODE,
    PANEL_KEY_BIT_SOS,
    PANEL_KEY_BIT_BUZZER,
    PANEL_KEY_BIT_AMBIENT_LIGHT,
} PANEL_KEY_BIT;

typedef struct
{
    uint8_t speed_up      = 0;
    uint8_t speed_down    = 0;
    uint8_t putter_up     = 0;
    uint8_t putter_down   = 0;
    uint8_t auto_nav      = 0;
    uint8_t voice_mode    = 0;
    uint8_t sos           = 0;
    uint8_t buzzer        = 0;
    uint8_t ambient_light = 0;
} KeyState;

// 浮点速度结构体，不再缩放int16
typedef struct
{
    float speed_x = 0.0f;
    float speed_y = 0.0f;
} JoystickData;

class UartNode : public rclcpp::Node
{
public:
    UartNode();
    ~UartNode() override;

private:
    int fd = -1;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr key_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joy_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<uint8_t> rx_buffer_;

    Joystick joy_;
    SCurveFilter curve_line_;
    SCurveFilter curve_angle_;


    uint16_t crc16_update(uint16_t crc, uint8_t data);
    uint16_t crc16_calc(const uint8_t *data, uint16_t len);

    KeyState parseKeyToState(uint16_t key_mask);
    JoystickData parseJoystickRaw(const uint8_t* frame);

    void publishKeyState(const KeyState& ks);
    void publishJoystickData(const JoystickData& js);

    void parseFrame(const std::vector<uint8_t>& frame);
    bool findHeader(std::vector<uint8_t>& buf, size_t& pos);
    void readLoop();
};

#endif
