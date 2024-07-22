/**
 * Juliebot
 * 
 * func:
 * <1> sub_Twist: control robot's movement
 * <2> pub_Odom: get robot's odom
 * 
 * to add:
 * <3> OLED display data like voltage
 * <4> pub /IMU
 * <5> pub /ultrasonic
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <WiFi.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <PidController.h>
#include <Kinematics.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// #include <rcl/subscription.h>
#include <geometry_msgs/msg/twist.h>

// #include <rcl/publisher.h>
#include <nav_msgs/msg/odometry.h>
#include <micro_ros_utilities/string_utilities.h>

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_subscription_t sub_Twist;
geometry_msgs__msg__Twist msg_sub_Twist; // topic: /cmd_vel

rcl_publisher_t pub_Odom;
nav_msgs__msg__Odometry msg_pub_Odom; // topic: /odom

#define MOTOR_0 0
#define AIN1_PIN 23
#define AIN2_PIN 22
#define MOTOR_1 1
#define BIN1_PIN 12
#define BIN2_PIN 13
Esp32McpwmMotor motor;

#define ENCODER_0 0
#define A_0_PIN 32
#define B_0_PIN 33
#define ENCODER_1 1
#define A_1_PIN 26
#define B_1_PIN 25
Esp32PcntEncoder encoder[2];

// #define D0 0.10354 // mm/pulse
// #define D1 0.10338
// int64_t tick_last[2];
// int64_t d_tick[2]; // pulses difference
// uint64_t time_last;
// uint64_t d_time;              // ms, time difference
// float current_motor_speed[2]; // m/s
float out_motor_speed[2];
PidController pid_controller[2];
Kinematics kinematics;

unsigned long previousMillis = 0; // last time(pub msg_pub_Odom)
unsigned long interval = 50;      // ms
// #define interval 50
// #define timeout_ms 1000

/**
 * Here: callbak_sub with Twist
 */
void callback_sub_Twist(const void *msg_in)
{
  const geometry_msgs__msg__Twist *msg_temp = (const geometry_msgs__msg__Twist *)msg_in;

  float linear_x = msg_temp->linear.x;
  float angular_z = msg_temp->angular.z;
  float target_speed_motor0, target_speed_motor1;

  kinematics.kinematics_inverse(linear_x * 1000, angular_z, target_speed_motor0, target_speed_motor1);

  Serial.printf("target_speed: l_a <%f mm/s, %f>, wheel <%f mm/s, %f mm/s>\n", linear_x * 1000, angular_z, target_speed_motor0, target_speed_motor1);

  pid_controller[0].update_target(target_speed_motor0); // mm/s
  pid_controller[1].update_target(target_speed_motor1);
}

/**
 * backend task :  configure and traite the MicroROS's communication with Agent
 */
void microros_task(void *param)
{
  /**
   * 1. Agent: WiFi
   */
  IPAddress agent_ip;
  // agent_ip.fromString("192.168.1.11");
  // set_microros_wifi_transports("CMCC-f6su", "xp4bx5c7", agent_ip, 8888);
  agent_ip.fromString("192.168.0.107");
  set_microros_wifi_transports("TP-888888", "12345678", agent_ip, 8888);
  delay(2000); // s, wait 2 seconds for the network connection

  /**
   * 2. MicroROS node, subscriber, publisher
   */
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "node_Juliebot", "", &support);
  rclc_subscription_init_default(&sub_Twist, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
  rclc_publisher_init_best_effort(&pub_Odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs,msg,Odometry),"odom");
  // rclc_publisher_init_default(&pub_Odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");


  // 2-> executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &sub_Twist, &msg_sub_Twist, &callback_sub_Twist, ON_NEW_DATA);

  // 2-> loops through the executor to process incoming msgs(sub) and publish odom data(pub).
  // pub: executor one time <initialize some parts of msg_pub_Odom>
  msg_pub_Odom.header.frame_id = micro_ros_string_utilities_set(msg_pub_Odom.header.frame_id, "odom");
  msg_pub_Odom.child_frame_id = micro_ros_string_utilities_set(msg_pub_Odom.child_frame_id, "base_link");

  
  /**
   * 3. Executor
   */
  while (true)
  {
    // 3-> sync time
    if (!rmw_uros_epoch_synchronized()) // RMW stands for "ROS Middleware"
    {
      rmw_uros_sync_session(1000); // RMW_RET_OK when success. RMW_RET_ERROR If no session is running or the synchronization fails.
      // continue;
      delay(10);
    }

    delay(100);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); // 3-> loops through the executor to process incoming msgs
  }
}

void setup()
{
  Serial.begin(115200);

  // Motor inilization
  motor.attachMotor(MOTOR_0, AIN2_PIN, AIN1_PIN);
  motor.attachMotor(MOTOR_1, BIN1_PIN, BIN2_PIN);

  // Encoder inilization
  encoder[0].init(ENCODER_0, A_0_PIN, B_0_PIN);
  encoder[1].init(ENCODER_1, A_1_PIN, B_1_PIN);

  // PID inilization
  pid_controller[0].update_pid(0.625, 0.125, 0.0);
  pid_controller[1].update_pid(0.625, 0.125, 0.0);
  pid_controller[0].out_limit(-100, 100); // MPCNT ranges from minus 100 to 100
  pid_controller[1].out_limit(-100, 100);

  // KInematics

  kinematics.set_motor_param(0, 45, 44, 65);
  kinematics.set_motor_param(1, 45, 44, 65);
  kinematics.set_kinematic_param(150);

  // Backend task, Core 0, Size_stack: 10240
  xTaskCreatePinnedToCore(microros_task, "microros_task", 10240, NULL, 1, NULL, 0);
}

void loop()
{

  // calculate the current motor speed
  kinematics.update_ticks_speed(micros(), encoder[0].getTicks(), encoder[1].getTicks());
  // Serial.printf("Ticks: tick0 = %d, tick1 = %d\n", encoder[0].getTicks(), encoder[1].getTicks());

  Serial.printf("current_speed: wheel <%f mm/s, %f mm/s>\n", kinematics.get_speed_motor(0), kinematics.get_speed_motor(1));

  // calculate the new output to control motor
  out_motor_speed[0] = pid_controller[0].update_control(kinematics.get_speed_motor(0)); // mm/s
  out_motor_speed[1] = pid_controller[1].update_control(kinematics.get_speed_motor(1));

  // Serial.printf("control: <%f, %f>\n", out_motor_speed[0], out_motor_speed[1]);

  // use the new output to control motor
  motor.updateMotorSpeed(MOTOR_0, out_motor_speed[0]);
  motor.updateMotorSpeed(MOTOR_1, out_motor_speed[1]);

  // get odom data per interval (50 ms)
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) // interval = 50
  {
    previousMillis = currentMillis;

    // float speed_linear, speed_angular;
    // kinematics.kinematics_forward(kinematics.get_speed_motor(0), kinematics.get_speed_motor(1), speed_linear, speed_angular);
    // Serial.printf("[%ld] linear: %f m/s, angular: %f, px: %f, py: %f, euler_yaw: %f\n", currentMillis, speed_linear, speed_angular, kinematics.get_odom().px, kinematics.get_odom().py, kinematics.get_odom().yaw);

    // header
    odom_t odom = kinematics.get_odom();
    int64_t stamp = rmw_uros_epoch_millis();
    msg_pub_Odom.header.stamp.sec = static_cast<int32_t>(stamp / 1000); // s part
    msg_pub_Odom.header.stamp.nanosec = static_cast<uint32_t>((stamp % 1000) * 1e6); // ns part

    // pose
    msg_pub_Odom.pose.pose.position.x = odom.px;
    msg_pub_Odom.pose.pose.position.y = odom.py;
    msg_pub_Odom.pose.pose.orientation.w = odom.orientation.w;
    msg_pub_Odom.pose.pose.orientation.x = odom.orientation.x;
    msg_pub_Odom.pose.pose.orientation.y = odom.orientation.y;
    msg_pub_Odom.pose.pose.orientation.z = odom.orientation.z;

    // twist
    msg_pub_Odom.twist.twist.linear.x = odom.speed_linear;
    msg_pub_Odom.twist.twist.angular.z = odom.speed_angular;

    // publish
    rcl_publish(&pub_Odom, &msg_pub_Odom, NULL);
  }

  delay(10);
}
