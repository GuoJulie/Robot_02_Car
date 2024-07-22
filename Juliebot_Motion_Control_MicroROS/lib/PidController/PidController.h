#ifndef __PIDCONTROLLER_H__
#define __PIDCONTROLLER_H__

class PidController
{
private:
    // pid
    float kp_; // Factor_Proportional
    float ki_; // Factor_Integral
    float kd_; // Factor_Derivative

    float error_;     // error between current_motor_speed and target_motor_speed
    float error_sum_; // error accumulation sum
    float derror_;    // error change rate

    float error_pre_;  // last_last error ???????????????
    float error_last_; // last_error

    float Integral_up_ = 2500; // integral maximum

    // data
    float target_;
    float output_last_;

    float output_min_;
    float output_max_;

public:
    PidController() = default;
    PidController(float kp, float ki, float kd);
    ~PidController() = default;

    // pid
    float update_control(float control);           // update the old control to a new output
    void update_pid(float kp, float ki, float kd); // update 3 factors
    void reset();                                  // reset PID Controller

    // data
    void update_target(float target);
    void out_limit(float output_min, float output_max);
};

#endif