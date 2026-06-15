

#ifndef S_CURVE_HPP
#define S_CURVE_HPP

#include <cmath>
#include <cstdint>

// 浮点绝对值工具
static inline float abs_f(float x)
{
    return (x >= 0.0f) ? x : -x;
}

class SCurveFilter
{
public:
    SCurveFilter();

    // 线速度S曲线初始化参数
    void lineInit();
    // 角速度S曲线初始化参数
    void angleInit();
    // 核心计算函数，完全沿用原始逻辑
    float calc(float target_val);

private:
    float output_;
    float err_;
    float accel_;
    float accel_up_;
    float accel_down_;
    float jerk_up_;
    float jerk_down_;
    float dt_;
    float kp_;
    uint16_t curve_dir_;
    uint16_t curve_dir_last_;
};

#endif


