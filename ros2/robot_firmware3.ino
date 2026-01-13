// base_control_bts760.ino
#include <micro_ros_arduino.h>
#include <stdio.h>

#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include <rmw_microros/rmw_microros.h>
#include <Encoder.h>

#define LED_PIN 13

// Error / check macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

// MotorPins structure and mapping (BTS760)
struct MotorPins {
  uint8_t RPWM, LPWM, REN, LEN;
  uint8_t ENC_A, ENC_B;
};
MotorPins leftMotor  = {8, 9, 6, 7, 15, 14};
MotorPins rightMotor = {4, 5, 2, 3, 16, 17};

//////////////// ROS entities ////////////////
rcl_publisher_t left_tick_pub;
std_msgs__msg__Int32 left_tick;

rcl_publisher_t right_tick_pub;
std_msgs__msg__Int32 right_tick;

rcl_publisher_t feedback_vel_pub;
geometry_msgs__msg__Twist feedback_vel_msg;

rcl_subscription_t wr_command_sub;
std_msgs__msg__Float32 wr_command;

rcl_subscription_t wl_command_sub;
std_msgs__msg__Float32 wl_command;

rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_subscription_t pid_gain_sub;
geometry_msgs__msg__Vector3 pid_gain;

rcl_timer_t timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

//////////////// Variables for control ////////////////
bool micro_ros_ready = false;

unsigned int SPIN_FREQ = 20;
float windup_guard = 20;

// PID control parameters
float kp = 0.4;
float ki = 0.001;
float kd = 0.08;

// Motor pulse parameters
float PPR = 840.0 * 4.0;
long pl = 0, pr = 0;

// Left Wheel parameters
unsigned long curTimeL = 0, prevTimeL = 0, dtL = 0;
long curTickL = 0, prevTickL = 0, diffTickL = 0;
double errL = 0, prev_errL = 0, sumErrL = 0, dErrL = 0, setRPML = 0;
double control_outL = 0;
double measuredRPML = 0;
double desiredRPML = 0;

// Right Wheel parameters
unsigned long curTimeR = 0, prevTimeR = 0, dtR = 0;
long curTickR = 0, prevTickR = 0, diffTickR = 0;
double errR = 0, prev_errR = 0, sumErrR = 0, dErrR = 0, setRPMR = 0;
double control_outR = 0;
double measuredRPMR = 0;
double desiredRPMR = 0; // <-- เพิ่มการประกาศตัวแปรที่ขาดไป

// encoder objects (พินมาจาก MotorPins)
Encoder r_encoder(rightMotor.ENC_A, rightMotor.ENC_B);
Encoder l_encoder(leftMotor.ENC_A, leftMotor.ENC_B);

// RPM counter vars
long old_pl = -999;
long old_pr = -999;

long curRPM_time = 0;
long prevRPM_time = 0;
long curPl = 0;
long curPr = 0;
long prevPl = 0;
long prevPr = 0;
long diffRPM_time = 0;
float RPM_l = 0, RPM_r = 0;

//////////////// helpers //////////////////
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

static int sign_of(double v) {
  if (v > 0.0) return 1;
  if (v < 0.0) return -1;
  return 0;
}

static double clamp_double(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ฟังก์ชันสำหรับขับมอเตอร์บน BTS760 (สมมติพฤติกรรมตามที่คุยไว้)
// พฤติกรรม: RPWM/LPWM สลับกันเพื่อกำหนดทิศทาง, REN/LEN เป็น enable/direction extra pins
void setMotor(const MotorPins &m, int direction_sign, int pwm_value)
{
  pwm_value = constrain(pwm_value, 0, 2000);

  if (direction_sign > 0) {
    // forward
    digitalWrite(m.REN, HIGH);
    digitalWrite(m.LEN, LOW);
    analogWrite(m.RPWM, pwm_value);
    analogWrite(m.LPWM, 0);
  } else if (direction_sign < 0) {
    // reverse
    digitalWrite(m.REN, LOW);
    digitalWrite(m.LEN, HIGH);
    analogWrite(m.RPWM, 0);
    analogWrite(m.LPWM, pwm_value);
  } else {
    // stop
    digitalWrite(m.REN, LOW);
    digitalWrite(m.LEN, LOW);
    analogWrite(m.RPWM, 0);
    analogWrite(m.LPWM, 0);
  }
}

//////////////// PID compute //////////////////
void computePIDL(double control_cmd, long inTick)
{
  int dirs = sign_of(control_cmd);

  curTickL = inTick;
  curTimeL = millis();

  setRPML = control_cmd;

  diffTickL = curTickL - prevTickL;
  dtL = (curTimeL - prevTimeL);

  if (dtL <= 0) {
    prevTickL = curTickL;
    prevTimeL = curTimeL;
    return;
  }

  measuredRPML = ((double)diffTickL / PPR) / ((double)dtL * 0.001) * 60.0;

  errL = fabs(setRPML) - fabs(measuredRPML);

  sumErrL += errL * ((double)dtL * 0.001);
  // anti-windup
  if (sumErrL > windup_guard) sumErrL = windup_guard;
  if (sumErrL < -windup_guard) sumErrL = -windup_guard;

  dErrL = (errL - prev_errL) / ((double)dtL * 0.001);

  control_outL = kp * errL + ki * sumErrL + kd * dErrL;
  control_outL = clamp_double(control_outL, 0.0, 255.0);

  prev_errL = errL;
  prevTickL = curTickL;
  prevTimeL = curTimeL;

  int pwm_to_use = (int)(control_outL + 0.5);
  setMotor(leftMotor, dirs, pwm_to_use);
}

void computePIDR(double control_cmd, long inTick)
{
  int dirs = sign_of(control_cmd);

  curTickR = inTick;
  curTimeR = millis();

  setRPMR = control_cmd;

  diffTickR = curTickR - prevTickR;
  dtR = (curTimeR - prevTimeR);

  if (dtR <= 0) {
    prevTickR = curTickR;
    prevTimeR = curTimeR;
    return;
  }

  measuredRPMR = ((double)diffTickR / PPR) / ((double)dtR * 0.001) * 60.0;

  errR = fabs(setRPMR) - fabs(measuredRPMR);

  sumErrR += errR * ((double)dtR * 0.001);
  if (sumErrR > windup_guard) sumErrR = windup_guard;
  if (sumErrR < -windup_guard) sumErrR = -windup_guard;

  dErrR = (errR - prev_errR) / ((double)dtR * 0.001);

  control_outR = kp * errR + ki * sumErrR + kd * dErrR;
  control_outR = clamp_double(control_outR, 0.0, 255.0);

  prev_errR = errR;
  prevTickR = curTickR;
  prevTimeR = curTimeR;

  int pwm_to_use = (int)(control_outR + 0.5);
  setMotor(rightMotor, dirs, pwm_to_use);
}

//////////////// Callbacks for subscriptions //////////////////
void pid_gain_callback(const void * msgin)
{
  const geometry_msgs__msg__Vector3 * gain_value = (const geometry_msgs__msg__Vector3 *)msgin;
  (void)gain_value; // ปิด unused warning ถ้าไม่ใช้ค่า
  // หากต้องการปรับค่าจาก topic ให้ยกเลิกคอมเมนต์
  /*kp = gain_value->x;
  ki = gain_value->y;
  kd = gain_value->z;*/
}

void rightwheel_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * power = (const std_msgs__msg__Float32 *)msgin;
  desiredRPMR = power->data;
}

void leftwheel_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * power = (const std_msgs__msg__Float32 *)msgin;
  desiredRPML = power->data;
}

void cmd_vel_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * tw = (const geometry_msgs__msg__Twist *)msgin;
  double v = tw->linear.x;     // linear velocity (m/s)
  double w = tw->angular.z;    // angular velocity (rad/s)

  // Robot-specific constants (adjust to your robot)
  const double wheel_base = 0.18;   // distance between wheels (meters)
  const double wheel_radius = 0.033; // wheel radius (meters)

  // Differential drive inverse kinematics
  double v_l = v - (w * wheel_base * 0.5);
  double v_r = v + (w * wheel_base * 0.5);

  // convert linear m/s to RPM: rpm = v / (2*pi*r) * 60
  double rpm_l = (v_l / (2.0 * M_PI * wheel_radius)) * 60.0;
  double rpm_r = (v_r / (2.0 * M_PI * wheel_radius)) * 60.0;

  desiredRPML = rpm_l;
  desiredRPMR = rpm_r;

  // debug
  Serial.print("cmd_vel -> v:"); Serial.print(v); Serial.print(" w:"); Serial.print(w);
  Serial.print(" rpm_l:"); Serial.print(rpm_l); Serial.print(" rpm_r:"); Serial.println(rpm_r);
}

//////////////// Utility: ticks & RPM //////////////////
void counter_RPM(long inPl, long inPr) {
  curPl = inPl;
  curPr = inPr;
  curRPM_time = millis();
  diffRPM_time = curRPM_time - prevRPM_time;
  if (diffRPM_time <= 0) diffRPM_time = 1;

  RPM_l = (((curPl - prevPl) / PPR) / (diffRPM_time * 0.001)) * 60.0;
  RPM_r = (((curPr - prevPr) / PPR) / (diffRPM_time * 0.001)) * 60.0;

  prevPl = curPl;
  prevPr = curPr;
  prevRPM_time = curRPM_time;

  feedback_vel_msg.linear.x = RPM_l;
  feedback_vel_msg.linear.y = RPM_r;
  feedback_vel_msg.angular.x = control_outL;
  feedback_vel_msg.angular.y = control_outR;
  feedback_vel_msg.angular.z = ki;
}

void counter_tick()
{
  pl = l_encoder.read();
  if (pl != old_pl) {
    old_pl = pl;
  }

  pr = r_encoder.read();
  if (pr != old_pr) {
    old_pr = pr;
  }

  left_tick.data = pl;
  right_tick.data = pr;
}

//////////////// Timer callback (ตาม template micro-ROS) //////////////////
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&feedback_vel_pub, &feedback_vel_msg, NULL));
  }
}

//////////////// Setup / Loop //////////////////
void setup() {
  // ตั้งค่า transport ของ micro-ROS (ต้อง implement ในโปรเจคของคุณ)
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // ตั้งพินมอเตอร์
  pinMode(leftMotor.RPWM, OUTPUT);
  pinMode(leftMotor.LPWM, OUTPUT);
  pinMode(leftMotor.REN, OUTPUT);
  pinMode(leftMotor.LEN, OUTPUT);

  pinMode(rightMotor.RPWM, OUTPUT);
  pinMode(rightMotor.LPWM, OUTPUT);
  pinMode(rightMotor.REN, OUTPUT);
  pinMode(rightMotor.LEN, OUTPUT);

  delay(200);

  // Serial for debug
  Serial.begin(115200);
  Serial.println("base_control_node starting");

  allocator = rcl_get_default_allocator();

  // init micro-ROS (ตาม template)
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "base_control_node", "", &support));

  // subscriptions
  RCCHECK(rclc_subscription_init_default(
    &pid_gain_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "pid_gain"));

  RCCHECK(rclc_subscription_init_default(
    &wl_command_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "wheel_command_left"));

  RCCHECK(rclc_subscription_init_default(
    &wr_command_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "wheel_command_right"));

  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // publishers
  RCCHECK(rclc_publisher_init_default(
    &left_tick_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "left_tick"));

  RCCHECK(rclc_publisher_init_default(
    &right_tick_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "right_tick"));

  RCCHECK(rclc_publisher_init_default(
    &feedback_vel_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "feedback_vel"));

  // timer (ใช้สำหรับส่ง feedback_vel เป็นตัวอย่าง)
  const unsigned int timer_timeout = 50; // ms
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &pid_gain_sub, &pid_gain, &pid_gain_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &wr_command_sub, &wr_command, &rightwheel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &wl_command_sub, &wl_command, &leftwheel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

  // เริ่มค่า default ของ message
  left_tick.data = 0;
  right_tick.data = 0;
  feedback_vel_msg.linear.x = 0;
  feedback_vel_msg.linear.y = 0;
  feedback_vel_msg.linear.z = 0;
  feedback_vel_msg.angular.x = 0;
  feedback_vel_msg.angular.y = 0;
  feedback_vel_msg.angular.z = 0;

  // เซ็ตเวลาเริ่มต้น
  prevTimeL = millis();
  prevTimeR = millis();
  prevRPM_time = millis();

  micro_ros_ready = true;
}

void loop()
{
  // อ่าน encoder, คำนวณ PID และเผยแพร่ tick/feedback เป็นรอบ ๆ
  counter_tick();

  // เผยแพร่ tick (non-blocking, ไม่ fatal ถ้าผิดพลาด)
  RCSOFTCHECK(rcl_publish(&left_tick_pub, &left_tick, NULL));
  RCSOFTCHECK(rcl_publish(&right_tick_pub, &right_tick, NULL));

  // ทำ PID และคำนวณ RPM จาก encoder
  computePIDR(desiredRPMR, pr);
  computePIDL(desiredRPML, pl);
  counter_RPM(pl, pr);

  // ประมวลผล callbacks ของ micro-ROS
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));

  // delay เล็กน้อย เพื่อให้ loop ไม่มี busy-wait มากเกินไป
  delay(5);
}
