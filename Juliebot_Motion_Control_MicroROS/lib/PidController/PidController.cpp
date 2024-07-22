#include <PidController.h>

// pid
PidController::PidController(float kp, float ki, float kd)
{
    reset();
    update_pid(kp, ki, kd);
}

float PidController::update_control(float control) // update the old control to a new output
{
    error_ = target_ - control;

    error_sum_ += error_;
    if (error_sum_ > Integral_up_)
        error_sum_ = Integral_up_;
    if (error_sum_ < -1 * Integral_up_)
        error_sum_ = -1 * Integral_up_;

    derror_ = error_last_ - error_;

    float output = kp_ * error_ + ki_ * error_sum_ + kd_ * derror_;

    if (output > output_max_)
        output = output_max_;
    if (output < output_min_)
        output = output_min_;

    // error_pre_ = error_last_;
    error_last_ = error_;
    output_last_ = output;

    return output;
}

void PidController::update_pid(float kp, float ki, float kd) // update 3 factors
{
    reset();
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PidController::reset() // reset PID Controller
{
    // pid
    kp_ = 0.0f; // Factor_Proportional
    ki_ = 0.0f; // Factor_Integral
    kd_ = 0.0f; // Factor_Derivative

    error_ = 0.0f;     // error between current_motor_speed and target_motor_speed
    error_sum_ = 0.0f; // error accumulation sum
    derror_ = 0.0f;    // error change rate

    // error_pre_ = 0.0f;  // last_last error ???????????
    error_last_ = 0.0f; // last_error

    // Integral_up_ = 2500; // integral maximum

    // data
    target_ = 0.0f;
    output_last_ = 0.0f;

    output_min_ = 0.0f;
    output_max_ = 0.0f;
}

// data
void PidController::update_target(float target)
{
    target_ = target;
}
void PidController::out_limit(float output_min, float output_max)
{
    output_min_ = output_min; // control output_min_
    output_max_ = output_max;
}