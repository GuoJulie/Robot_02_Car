#include "Kinematics.h"

// motor
void Kinematics::set_motor_param(uint8_t motor_id, uint16_t reduction_ratio, uint16_t pulse_ratio, float diameter_wheel)
{
    // set
    motor_param_[motor_id].id = motor_id;
    motor_param_[motor_id].reduction_ratio = reduction_ratio;
    motor_param_[motor_id].pulse_ratio = pulse_ratio;
    motor_param_[motor_id].diameter_wheel = diameter_wheel;

    // calculate
    motor_param_[motor_id].distance_per_pulse = (diameter_wheel * PI) / (reduction_ratio * pulse_ratio);           // mm/pulse
    motor_param_[motor_id].speed_factor = (1000 * 1000) * (diameter_wheel * PI) / (reduction_ratio * pulse_ratio); // mm/s

    Serial.printf("init motor param %d: %f=%f*PI/(%d*%d) speed_factor=%d\n", motor_id, motor_param_[motor_id].distance_per_pulse, diameter_wheel, reduction_ratio, pulse_ratio, motor_param_[motor_id].speed_factor);
}

void Kinematics::update_ticks_speed(uint64_t time_current, int32_t tick0, int32_t tick1)
{
    uint32_t dt = time_current - motor_param_[0].last_update_time; // us, time difference
    int32_t dtick0 = tick0 - motor_param_[0].last_encoder_tick;    // tick difference
    int32_t dtick1 = tick1 - motor_param_[1].last_encoder_tick;

    // speed_wheel
    motor_param_[0].speed_motor = dtick0 * ((double)motor_param_[0].speed_factor / dt); // mm/s, dt : us
    motor_param_[1].speed_motor = dtick1 * ((double)motor_param_[1].speed_factor / dt);

    Serial.printf("<1> dt = %d, dtick0 = %d, dtick1 = %d, factor = %d\n", dt, dtick0, dtick1, motor_param_[0].speed_factor );

    motor_param_[0].last_encoder_tick = tick0; // update time_last & ticks_last
    motor_param_[1].last_encoder_tick = tick1;
    motor_param_[0].last_update_time = time_current;
    motor_param_[1].last_update_time = time_current;

    this->update_odom(dt);
}

float Kinematics::get_speed_motor(uint8_t motor_id)
{
    return motor_param_[motor_id].speed_motor;
}

// kinematics
void Kinematics::set_kinematic_param(float distance_wheels)
{
    distance_wheels_ = distance_wheels;
}

void Kinematics::kinematics_forward(float speed_wheel0, float speed_wheel1, float &out_speed_linear, float &out_speed_angular)
{
    /**
     * v = (v_l + v_r) / 2
     * w = (v_r - v_l) / l
     */

    out_speed_linear = (speed_wheel0 + speed_wheel1) / 2.0;
    out_speed_angular = (speed_wheel1 - speed_wheel0) / distance_wheels_;
}

void Kinematics::kinematics_inverse(float speed_linear, float speed_angular, float &out_speed_wheel0, float &out_speed_wheel1)
{
    /**
     * v_l = v - w * l / 2
     * v_r = v + w * l / 2
     */

    out_speed_wheel0 = speed_linear - (speed_angular * distance_wheels_) / 2.0;
    out_speed_wheel1 = speed_linear + (speed_angular * distance_wheels_) / 2.0;
}

// odom
void Kinematics::update_odom(uint32_t dt) // dt: us
{
    float dt_s = (float)(dt / 1000) / 1000; // dt: us -> s

    // twist: linear, angular
    static float speed_linear, speed_angular;
    this->kinematics_forward(motor_param_[0].speed_motor, motor_param_[1].speed_motor, speed_linear, speed_angular);
    odom_.speed_linear = speed_linear / 1000; // mm/s -> m/s
    odom_.speed_angular = speed_angular;

    // euler: yaw
    odom_.yaw += odom_.speed_angular * dt_s;
    this->angle2PI(odom_.yaw, odom_.yaw);

    // quaternion
    this->Euler2Quaternion(0, 0, odom_.yaw, odom_.orientation);

    // pose: x, y
    float d_distance = odom_.speed_linear * dt_s; // m
    odom_.px += d_distance * std::cos(odom_.yaw); // m
    odom_.py += d_distance * std::sin(odom_.yaw);
}

odom_t &Kinematics::get_odom()
{
    return odom_;
}

// others_calculate
void Kinematics::angle2PI(float angle, float &out_angle)
{
    if (angle > PI)
    {
        out_angle = angle - (2 * PI);
    }
    else if (angle < -PI)
    {
        out_angle = angle + (2 * PI);
    }

    // if (angle > PI)
    // {
    //     out_angle -= 2 * PI;
    // }
    // else if (angle < -PI)
    // {
    //     out_angle += 2 * PI;
    // }
}

void Kinematics::Euler2Quaternion(float roll, float pitch, float yaw, quaternion_t &q)
{
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
}
