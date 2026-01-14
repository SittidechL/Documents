
// ============================
// micro-ROS Node (Teensy 4.0) + MotorController (L298N)
// Topics:
//  - Sub:  /wheel_command_left  (std_msgs/Float32)   -> RPM setpoint left
//          /wheel_command_right (std_msgs/Float32)   -> RPM setpoint right
//  - Pub:  /left_tick  (std_msgs/Int32)
//          /right_tick (std_msgs/Int32)
//          /feedback_vel (geometry_msgs/Twist) -> linear.x=RPM_L, linear.y=RPM_R,
//                                                 angular.x=pwmL, angular.y=pwmR
// ============================

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include "MotorController.h"

// ----------------------------
// Pin mapping (ปรับตามการต่อของคุณ)
// ----------------------------
#define LED_PIN      13

#define R_DIR1_PIN   1
#define R_DIR2_PIN   2
#define R_PWM_PIN    0

#define L_DIR1_PIN   3
#define L_DIR2_PIN   4
#define L_PWM_PIN    5

#define L_ENC_A      7
#define L_ENC_B      8
#define R_ENC_A      9
#define R_ENC_B      10

// ***หมายเหตุ*** พิน 0/1 บน Teensy 4.0 เป็น Serial1 (RX/TX)
// ถ้าใช้ Serial1 กับอย่างอื่น ให้ย้าย PWM ไปพินอื่นที่รองรับ PWM

// ----------------------------
// Controller instance
// ----------------------------

// กำหนด PPR (เช็คกับ encoder + อัตราทดจริง)
static constexpr float PPR = 840.0f * 4.0f;  // ตัวอย่าง

MotorController::Pins pins{
  .l_dir1 = L_DIR1_PIN,
  .l_dir2 = L_DIR2_PIN,
  .l_pwm  = L_PWM_PIN,
  .r_dir1 = R_DIR1_PIN,
  .r_dir2 = R_DIR2_PIN,
  .r_pwm  = R_PWM_PIN,
  .l_enc_a = L_ENC_A,
  .l_enc_b = L_ENC_B,
  .r_enc_a = R_ENC_A,
  .r_enc_b = R_ENC_B
};

MotorController controller(pins, PPR, /*pwm_max*/ 4095);

// ----------------------------
// ROS entities
// ----------------------------
rcl_publisher_t left_tick_pub;
rcl_publisher_t right_tick_pub;
rcl_publisher_t feedback_vel_pub;

std_msgs__msg__Int32 left_tick_msg;
std_msgs__msg__Int32 right_tick_msg;
geometry_msgs__msg__Twist feedback_vel_msg;

rcl_subscription_t wl_sub;
rcl_subscription_t wr_sub;
rcl_subscription_t pid_gain_sub;

std_msgs__msg__Float32 wl_cmd_msg;
std_msgs__msg__Float32 wr_cmd_msg;
geometry_msgs__msg__Vector3 pid_gain_msg;

rcl_timer_t timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

bool micro_ros_init_successful = false;

// Desired RPM (อัปเดตจาก ROS)
volatile float desiredRPM_L = 0.0f;
volatile float desiredRPM_R = 0.0f;

// ----------------------------
// Callbacks
// ----------------------------
void left_cmd_cb(const void* msgin) {
  auto* m = (const std_msgs__msg__Float32*) msgin;
  desiredRPM_L = m->data;
}

void right_cmd_cb(const void* msgin) {
  auto* m = (const std_msgs__msg__Float32*) msgin;
  desiredRPM_R = m->data;
}

void pid_gain_cb(const void* msgin) {
  auto* g = (const geometry_msgs__msg__Vector3*) msgin;
  auto gains = controller.getGains();
  // เปิดใช้งานถ้าจะจูนผ่าน ROS (ตอนนี้แค่ดูค่า)
  // gains.kp = g->x;
  // gains.ki = g->y;
  // gains.kd = g->z;
  controller.setGains(gains);
}

// Timer: ส่ง feedback_vel + ticks เป็นคาบ
void timer_cb(rcl_timer_t* /*timer*/) {
  // ตีค่าตาม state ล่าสุดของ controller
  auto st = controller.getState();

  feedback_vel_msg.linear.x  = st.rpm_left;
  feedback_vel_msg.linear.y  = st.rpm_right;
  feedback_vel_msg.angular.x = st.pwm_left;
  feedback_vel_msg.angular.y = st.pwm_right;

  rcl_publish(&feedback_vel_pub, &feedback_vel_msg, nullptr);

  left_tick_msg.data  = controller.leftTicks();
  right_tick_msg.data = controller.rightTicks();

  rcl_publish(&left_tick_pub, &left_tick_msg, nullptr);
  rcl_publish(&right_tick_pub, &right_tick_msg, nullptr);
}

// ----------------------------
// micro-ROS boilerplate
// ----------------------------
bool create_entities() {
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "base_control_node", "", &support);

  // pubs
  rclc_publisher_init_default(
    &left_tick_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "left_tick");

  rclc_publisher_init_default(
    &right_tick_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "right_tick");

  rclc_publisher_init_default(
    &feedback_vel_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "feedback_vel");

  // subs
  rclc_subscription_init_default(
    &wl_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel_command_left");

  rclc_subscription_init_default(
    &wr_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel_command_right");

  rclc_subscription_init_default(
    &pid_gain_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "pid_gain");

  // timer 50ms
  rclc_timer_init_default(
    &timer, &support, RCL_MS_TO_NS(50), timer_cb);

  // executor
  rclc_executor_init(&executor, &support.context, 4, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &wl_sub, &wl_cmd_msg, &left_cmd_cb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &wr_sub, &wr_cmd_msg, &right_cmd_cb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &pid_gain_sub, &pid_gain_msg, &pid_gain_cb, ON_NEW_DATA);

  micro_ros_init_successful = true;
  return true;
}

void destroy_entities() {
  rcl_publisher_fini(&left_tick_pub, &node);
  rcl_publisher_fini(&right_tick_pub, &node);
  rcl_publisher_fini(&feedback_vel_pub, &node);

  rcl_subscription_fini(&wl_sub, &node);
  rcl_subscription_fini(&wr_sub, &node);
  rcl_subscription_fini(&pid_gain_sub, &node);

  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  micro_ros_init_successful = false;
}

// ----------------------------
// Arduino setup/loop
// ----------------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // เริ่มต้น Controller (มอเตอร์ + encoder)
  controller.begin();

  // ตั้ง micro-ROS transport (USB CDC)
  set_microros_transports();

  // initial entities = false; สร้างเมื่อพบ agent
  micro_ros_init_successful = false;
}

void loop() {
  // ตรวจว่า agent ตอบไหม
  uint32_t delay_rec_us = 100000;
  if (RMW_RET_OK == rmw_uros_ping_agent(50, 2)) {
    delay_rec_us = 5000;
    if (!micro_ros_init_successful) {
      create_entities();
    } else {
      // อัปเดตมอเตอร์ตามสัญญาณ ROS ที่ได้
      controller.update(desiredRPM_L, desiredRPM_R);

      // หมุน executor ช่วงสั้น ๆ
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
    }
  } else if (micro_ros_init_successful) {
    destroy_entities();
  }

  delayMicroseconds(delay_rec_us);
}
``
