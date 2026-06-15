#include "per_joystick.hpp"

Joystick::Joystick()
{
    state_ = JoyState::INIT;
    // 初始化线、角速度各自S曲线参数
    curve_line_.lineInit();
    curve_angle_.angleInit();
}

int16_t Joystick::abs16(int16_t x)
{
    return (x >= 0) ? x : -x;
}

float Joystick::absFloat(float x)
{
    return (x >= 0.0f) ? x : -x;
}

int16_t Joystick::applyDeadzone(int16_t val)
{
    if (abs16(val) < JOY_DEAD_ZONE)
        return 0;
    return val;
}

float Joystick::limitFloat(float val, float max)
{
    if (val > max) return max;
    if (val < -max) return -max;
    return val;
}

float Joystick::adcToNorm(int16_t delta)
{
    float x = static_cast<float>(delta) / ADC_FULL_SCALE;
    return limitFloat(x, 1.0f);
}

void Joystick::gearInit()
{
    uint8_t init_gear = SPEED_GEAR_1;
    gear_ctrl_.gear = init_gear;
    gear_ctrl_.gear_up_evt = 0;
    gear_ctrl_.gear_dn_evt = 0;
}

void Joystick::gearUpdate()
{
    if (gear_ctrl_.gear_up_evt)
    {
        if (gear_ctrl_.gear < SPEED_GEAR_4)
            gear_ctrl_.gear++;
        gear_ctrl_.gear_up_evt = 0;
    }
    if (gear_ctrl_.gear_dn_evt)
    {
        if (gear_ctrl_.gear > SPEED_GEAR_1)
            gear_ctrl_.gear--;
        gear_ctrl_.gear_dn_evt = 0;
    }
}

uint8_t Joystick::getGear() const
{
    return gear_ctrl_.gear;
}

JoyState Joystick::getState() const
{
    return state_;
}

const JoySpeed& Joystick::getSpeed() const
{
    return speed_;
}

float Joystick::lineSpeedToRpm(float line_speed)
{
    return (line_speed * 60.0f) / (2.0f * PI * WHEEL_RADIUS) * REDUCTION_RATIO;
}

void Joystick::mapJoyToMotion()
{
    float joy_x = limitFloat(norm_x_, 1.0f);
    float joy_y = limitFloat(norm_y_, 1.0f);
    uint8_t gear = gear_ctrl_.gear;

    if (gear > 3u) gear = 3u;
    const GearParam& param = gear_table_[gear];

    float v_cmd = joy_y * param.vmax;
    float w_cmd = -joy_x * param.wmax;

    // 严格在此处调用S曲线计算，逻辑完全沿用你原始算法
    speed_.line_speed = curve_line_.calc(v_cmd);
    speed_.angle_speed = curve_angle_.calc(w_cmd);
}

void Joystick::sCurveCalc()
{
    // 接口保留，实际计算已经放在mapJoyToMotion赋值处
}

void Joystick::stateMachineUpdate(uint16_t adc_x, uint16_t adc_y)
{
    adc_x_ = adc_x;
    adc_y_ = adc_y;

    switch (state_)
    {
    case JoyState::INIT:
    {
            bool in_center_zone = (adc_x_ > (JOY_RAW_CENTAL_X - JOY_RAW_CENTAL_ZONE) && adc_x_ < (JOY_RAW_CENTAL_X + JOY_RAW_CENTAL_ZONE))
                               && (adc_y_ > (JOY_RAW_CENTAL_Y - JOY_RAW_CENTAL_ZONE) && adc_y_ < (JOY_RAW_CENTAL_Y + JOY_RAW_CENTAL_ZONE));
            if (in_center_zone)
            {
                calib_sum_x_ += adc_x_;
                calib_sum_y_ += adc_y_;
                calib_cnt_++;
                if (calib_cnt_ >= JOY_CALIB_CNT)
                {
                    center_x_ = static_cast<int16_t>(calib_sum_x_ / JOY_CALIB_CNT);
                    center_y_ = static_cast<int16_t>(calib_sum_y_ / JOY_CALIB_CNT);
                    calibrated_ = true;
                    state_ = JoyState::IDLE;
                    filt_x_ = 0.0f;
                    filt_y_ = 0.0f;
                }
            }
            else
            {
                calib_sum_x_ = 0;
                calib_sum_y_ = 0;
                calib_cnt_ = 0;
                center_x_ = JOY_RAW_CENTAL_X;
                center_y_ = JOY_RAW_CENTAL_Y;
            }
        break;
    }

    case JoyState::IDLE:
        speed_.line_speed = 0.0f;
        speed_.angle_speed = 0.0f;
        norm_x_ = 0.0f;
        norm_y_ = 0.0f;

        delta_x_ = static_cast<int16_t>(adc_x_) - center_x_;
        delta_y_ = static_cast<int16_t>(adc_y_) - center_y_;
        delta_x_ = applyDeadzone(delta_x_);
        delta_y_ = applyDeadzone(delta_y_);

        if (abs16(delta_x_) > ACTIVE_ENTER_TH || abs16(delta_y_) > ACTIVE_ENTER_TH)
        {
            active_cnt_++;
            if (active_cnt_ >= STATE_CONFIRM_CNT)
            {
                state_ = JoyState::ACTIVE;
                active_cnt_ = 0;
                filt_x_ = static_cast<float>(delta_x_);
                filt_y_ = static_cast<float>(delta_y_);
            }
        }
        else
        {
            active_cnt_ = 0;
        }
        break;

    case JoyState::ACTIVE:
        delta_x_ = static_cast<int16_t>(adc_x_) - center_x_;
        delta_y_ = static_cast<int16_t>(adc_y_) - center_y_;
        delta_x_ = applyDeadzone(delta_x_);
        delta_y_ = applyDeadzone(delta_y_);

        if (abs16(delta_x_) < ACTIVE_EXIT_TH && abs16(delta_y_) < ACTIVE_EXIT_TH)
        {
            idle_cnt_++;
            if (idle_cnt_ >= STATE_CONFIRM_CNT)
            {
                state_ = JoyState::IDLE;
                idle_cnt_ = 0;
                active_cnt_ = 0;
                filt_x_ = 0.0f;
                filt_y_ = 0.0f;
                norm_x_ = 0.0f;
                norm_y_ = 0.0f;
                speed_.line_speed = 0.0f;
                speed_.angle_speed = 0.0f;
                break;
            }
        }
        else
        {
            idle_cnt_ = 0;
        }

        filt_x_ = filt_x_ * (1.0f - JOY_FILTER_ALPHA) + static_cast<float>(delta_x_) * JOY_FILTER_ALPHA;
        filt_y_ = filt_y_ * (1.0f - JOY_FILTER_ALPHA) + static_cast<float>(delta_y_) * JOY_FILTER_ALPHA;

        norm_x_ = adcToNorm(static_cast<int16_t>(filt_x_));
        norm_y_ = adcToNorm(static_cast<int16_t>(filt_y_));

        mapJoyToMotion();
        break;

    case JoyState::FAULT:
    default:
        break;
    }
}


void Joystick::setGearUpEvt(uint8_t val)
{
    gear_ctrl_.gear_up_evt = val;
}
void Joystick::setGearDnEvt(uint8_t val)
{
    gear_ctrl_.gear_dn_evt = val;
}


