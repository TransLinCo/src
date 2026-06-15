#include "s_curve.hpp"

SCurveFilter::SCurveFilter()
{
    // ƒ¨»œ«Â¡„
    output_ = 0.0f;
    err_ = 0.0f;
    accel_ = 0.0f;
    accel_up_ = 0.0f;
    accel_down_ = 0.0f;
    jerk_up_ = 0.0f;
    jerk_down_ = 0.0f;
    dt_ = 0.005f;
    kp_ = 0.0f;
    curve_dir_ = 0;
    curve_dir_last_ = 0;
}

void SCurveFilter::lineInit()
{
    output_ = 0.0f;
    accel_ = 0.0f;
    accel_up_ = 1.0f;
    accel_down_ = -1.6f;
    jerk_up_ = 16.0f;
    jerk_down_ = 40.0f;
    dt_ = 0.005f;
    curve_dir_ = 0;
    curve_dir_last_ = 0;
}

void SCurveFilter::angleInit()
{
    output_ = 0.0f;
    accel_ = 0.0f;
    accel_up_ = 1.2f;
    accel_down_ = -1.4f;
    jerk_up_ = 24.0f;
    jerk_down_ = 40.0f;
    dt_ = 0.005f;
    curve_dir_ = 0;
    curve_dir_last_ = 0;
}

float SCurveFilter::calc(float target_val)
{
    float speed_err = target_val - output_;
    err_ = speed_err;

    if (abs_f(speed_err) < 0.003f)
    {
        output_ = target_val;
        return output_;
    }

    curve_dir_ = ((output_ * speed_err) >= 0.0f) ? 1u : 0u;

    if (curve_dir_ != curve_dir_last_)
    {
        curve_dir_last_ = curve_dir_;
        accel_ = 0.0f;
    }

    if (curve_dir_)
    {
        if (output_ >= 0.0f)
        {
            if (accel_ < accel_up_)
            {
                accel_ += jerk_up_ * dt_;
            }
            else
            {
                accel_ = accel_up_;
            }
        }
        else
        {
            if (accel_ > -accel_up_)
            {
                accel_ -= jerk_up_ * dt_;
            }
            else
            {
                accel_ = -accel_up_;
            }
        }

        if ((abs_f(output_) >= abs_f(target_val - (accel_ * dt_)))
            && (abs_f(output_) <= abs_f(target_val + (accel_ * dt_))))
        {
            output_ = target_val;
        }
        else
        {
            output_ += accel_ * dt_;
        }
    }
    else
    {
        if (output_ >= 0.0f)
        {
            if (accel_ > accel_down_)
            {
                accel_ -= jerk_down_ * dt_;
            }
            else
            {
                accel_ = accel_down_;
            }
        }
        else
        {
            if (accel_ < -accel_down_)
            {
                accel_ += jerk_down_ * dt_;
            }
            else
            {
                accel_ = -accel_down_;
            }
        }

        if ((abs_f(output_) <= abs_f(target_val + (accel_ * dt_)))
            && (abs_f(output_) >= abs_f(target_val - (accel_ * dt_))))
        {
            output_ = target_val;
        }
        else
        {
            output_ += accel_ * dt_;
        }
    }

    return output_;
}