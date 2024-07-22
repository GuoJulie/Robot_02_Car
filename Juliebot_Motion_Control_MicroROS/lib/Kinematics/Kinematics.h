#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__
#include <Arduino.h>

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} quaternion_t;


typedef struct
{
    // set
    uint8_t id;
    float diameter_wheel; // mm
    uint16_t reduction_ratio; // 45
    uint16_t pulse_ratio; // 44

    // calculate
    float distance_per_pulse; // mm
    uint32_t speed_factor; // mm/s

    // read
    int16_t speed_motor; // mm/s
    int64_t last_encoder_tick;
    uint64_t last_update_time; //us

} motor_param_t;

typedef struct
{
    // position
    float px; 
    float py;

    // euler
    float yaw;

    // quaternion
    quaternion_t orientation;

    // twist
    float speed_linear;
    float speed_angular;
} odom_t;



class Kinematics
{
private:
    motor_param_t motor_param_[2];
    float distance_wheels_; // m
    odom_t odom_;

public:
    Kinematics() = default;
    ~Kinematics() = default;

    // motor
    void set_motor_param(uint8_t motor_id, uint16_t reduction_ratio, uint16_t pulse_ratio, float diameter_wheel);
    void update_ticks_speed(uint64_t time_current, int32_t tick0, int32_t tick1);
    float get_speed_motor(uint8_t motor_id);


    // kinematics
    void set_kinematic_param(float distance_wheels);
    void kinematics_forward(float speed_wheel0, float speed_wheel1, float &out_speed_linear, float &out_speed_angular);
    void kinematics_inverse(float speed_linear, float speed_angular, float &out_speed_wheel0, float &out_speed_wheel1);

    // odom
    void update_odom(uint32_t dt);
    odom_t & get_odom();


    // others_calculate
    void angle2PI(float angle, float &out_angle);
    void Euler2Quaternion(float roll, float pitch, float yaw, quaternion_t &q);
};


#endif