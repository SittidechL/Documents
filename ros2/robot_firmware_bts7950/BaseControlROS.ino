
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

#define LED_PIN 13

// ---- ใช้พินตามที่คุณกำหนด ----
MotorController::MotorPins leftMotor  = {8, 9, 6, 7, 15, 14};
MotorController::MotorPins rightMotor = {4, 5, 2, 3, 16, 17};

// PPR ของ encoder (ตรวจสอบให้ตรงกับฮาร์ดแวร์จริง)
static constexpr float PPR = 840.0f * 4.0f;

// Controller instance (BTS7960)
MotorController controller(leftMotor, rightMotor, PPR, /*pwm_max*/ 4095);

// ROS entities
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

// desired RPM (จาก ROS)
volatile float desiredRPM_L = 0.0f;
volatile float desiredRPM_R = 0.0f;

// Callbacks
void left_cmd_cb(const void* msgin)  { desiredRPM_L = ((std_msgs__msg__Float32*)msgin)->data; }
void right_cmd_cb(const void* msgin) { desiredRPM_R = ((std_msgs__msg__Float32*)msgin)->data; }
void pid_gain_cb(const void* msgin) {
  auto* g = (const geometry_msgs__msg__Vector3*)msgin;
  auto gains = controller.getGains();
  // gains.kp = g->x; gains.ki = g->y; gains.kd = g->z; // เปิดใช้เมื่อต้องการจูนออนไลน์
  controller.setGains(gains);
}

void timer_cb(rcl_timer_t*) {
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

bool create_entities() {
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "base_control_node", "", &support);

  rclc_publisher_init_default(&left_tick_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "left_tick");
  rclc_publisher_init_default(&right_tick_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "right_tick");
  rclc_publisher_init_default(&feedback_vel_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "feedback_vel");

  rclc_subscription_init_default(&wl_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel_command_left");
  rclc_subscription_init_default(&wr_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel_command_right");
  rclc_subscription_init_default(&pid_gain_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "pid_gain");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_cb);

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

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  controller.begin();
  set_microros_transports();    // USB CDC transport
  micro_ros_init_successful = false;
}

void loop() {
  uint32_t delay_rec_us = 100000;
  if (RMW_RET_OK == rmw_uros_ping_agent(50, 2)) {
    delay_rec_us = 5000;
    if (!micro_ros_init_successful) {
      create_entities();
    } else {
      controller.update(desiredRPM_L, desiredRPM_R);
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
    }
  } else if (micro_ros_init_successful) {
    destroy_entities();
  }
  delayMicroseconds(delay_rec_us);
}
