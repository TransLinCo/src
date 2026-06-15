#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include <cstdint>
#include <cmath>
#include "s_curve.hpp"

// 挡位定义
#define SPEED_GEAR_1    0
#define SPEED_GEAR_2    1
#define SPEED_GEAR_3    2
#define SPEED_GEAR_4    3

// 基础硬件参数
#define WHEEL_RADIUS                   (0.12f)
#define PI                             (3.1415926f)
#define REDUCTION_RATIO                (16.8f)

// 摇杆配置参数
#define JOY_CALIB_CNT                   (40)
#define JOY_DEAD_ZONE                   (60)
#define JOY_FILTER_ALPHA                (0.2f)
#define JOY_RAW_CENTAL_X                (2035u)
#define JOY_RAW_CENTAL_Y                (2030u)
#define JOY_RAW_CENTAL_ZONE             (130u)
#define ADC_FULL_SCALE                  (2035u)
#define ACTIVE_ENTER_TH                 (300)
#define ACTIVE_EXIT_TH                  (150)
#define STATE_CONFIRM_CNT               (3)

// 转弯限速配置（宏保留但代码逻辑移除）
#define JOY_TURN_RAMP_GEAR_TH         (SPEED_GEAR_2)
#define JOY_TURN_TH                  (0.15f)
#define JOY_TURN_HYS_EXIT_TH        (0.10f)

// 摇杆状态枚举
enum class JoyState
{
    INIT = 0,
    IDLE,
    ACTIVE,
    FAULT
};

// 挡位参数表
struct GearParam
{
    float vmax; // m/s
    float wmax; // rad/s
};

// 精简速度结构体，仅保留需要字段
struct JoySpeed
{
    float line_speed = 0.0f;
    float angle_speed = 0.0f;
};

// 挡位控制
struct GearCtrl
{
    uint8_t gear = SPEED_GEAR_1;
    uint8_t gear_up_evt = 0;
    uint8_t gear_dn_evt = 0;
};

// 摇杆C++类封装
class Joystick
{
public:
    Joystick();

    // 对外接口
    void gearInit();
    void gearUpdate();
    uint8_t getGear() const;
    JoyState getState() const;
    void stateMachineUpdate(uint16_t adc_x, uint16_t adc_y);
    float lineSpeedToRpm(float line_speed);
    void sCurveCalc();
    const JoySpeed& getSpeed() const;
    void setGearUpEvt(uint8_t val);
    void setGearDnEvt(uint8_t val);
private:
    // 原始ADC
    uint16_t adc_x_ = 0;
    uint16_t adc_y_ = 0;

    // 零点标定
    int16_t center_x_ = 0;
    int16_t center_y_ = 0;
    uint32_t calib_sum_x_ = 0;
    uint32_t calib_sum_y_ = 0;
    uint8_t calib_cnt_ = 0;

    // 差值与滤波
    int16_t delta_x_ = 0;
    int16_t delta_y_ = 0;
    float filt_x_ = 0.0f;
    float filt_y_ = 0.0f;

    // 归一化输出
    float norm_x_ = 0.0f;
    float norm_y_ = 0.0f;

    // 状态机
    JoyState state_ = JoyState::INIT;
    uint8_t active_cnt_ = 0;
    uint8_t idle_cnt_ = 0;
    bool calibrated_ = false;

    // 速度、挡位
    JoySpeed speed_;
    GearCtrl gear_ctrl_;

    // S曲线实例
    SCurveFilter curve_line_;
    SCurveFilter curve_angle_;

    // 内部工具函数
    int16_t abs16(int16_t x);
    int16_t applyDeadzone(int16_t val);
    float limitFloat(float val, float max);
    float adcToNorm(int16_t delta);
    float absFloat(float x);
    void mapJoyToMotion();

    // 挡位表常量
    static constexpr GearParam gear_table_[4] =
    {
        {0.3333f, 0.5f},
        {0.6944f, 0.9f},
        {0.9722f, 1.1f},
        {1.2500f, 1.4f}
    };
};


#endif

